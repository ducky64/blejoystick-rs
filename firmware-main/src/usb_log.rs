//! USB log port (CDC ACM), always present in the USB device.
//!
//! Without the `defmt-rtt` feature (`--no-default-features`), it carries the defmt stream.
//! A full pipe drops bytes silently, like defmt-rtt's `disable-blocking-mode`.
//! The pipe keeps logs from boot until a host opens the port.
//!
//! With `defmt-rtt`, nothing writes the pipe, and the port only sends [`RTT_NOTICE`] (plain text,
//! for a terminal), once per USB connection, when the host first reads.
//!
//! Host side: `defmt-print -e <elf> serial --path <port>`, or `cargo console` (see the README).

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Sender, State};
use embassy_usb::driver::{Driver, EndpointError};
use embassy_usb::Builder;
use static_cell::StaticCell;

/// CDC ACM bulk endpoint packet size (full speed maximum).
const MAX_PACKET_SIZE: u16 = 64;

/// Largest packet actually sent: one short of [`MAX_PACKET_SIZE`], so every packet is short and
/// ends its transfer by itself. A full packet would need a zero-length packet (ZLP) after to flush.
const SEND_SIZE: usize = MAX_PACKET_SIZE as usize - 1;

/// Encoded defmt frames waiting for the host.
static PIPE: Pipe<CriticalSectionRawMutex, 4096> = Pipe::new();

/// Sent (instead of logs) once per USB connection, when built with `defmt-rtt`.
const RTT_NOTICE: &[u8] = b"\r\nblejoystick-rs: built with defmt-rtt, logs are on RTT (SWD)\r\n";

/// The defmt global logger, into [`PIPE`]. Mutually exclusive with defmt-rtt's.
#[cfg(not(feature = "defmt-rtt"))]
mod logger {
    use core::cell::UnsafeCell;
    use core::sync::atomic::{AtomicBool, Ordering};

    use super::PIPE;

    #[defmt::global_logger]
    struct Logger;

    /// Set while a frame is being encoded, to catch reentrant use (which would corrupt the encoder).
    static TAKEN: AtomicBool = AtomicBool::new(false);

    /// Logger state, only accessed between `acquire` and `release`, within a critical section.
    struct LoggerState {
        cs_restore: UnsafeCell<critical_section::RestoreState>,
        encoder: UnsafeCell<defmt::Encoder>,
    }

    // SAFETY: only accessed inside the critical section taken by `acquire`, guarded by TAKEN
    unsafe impl Sync for LoggerState {}

    static STATE: LoggerState = LoggerState {
        cs_restore: UnsafeCell::new(critical_section::RestoreState::invalid()),
        encoder: UnsafeCell::new(defmt::Encoder::new()),
    };

    fn write_pipe(bytes: &[u8]) {
        // drops whatever doesn't fit
        let _ = PIPE.try_write(bytes);
    }

    unsafe impl defmt::Logger for Logger {
        fn acquire() {
            // SAFETY: released in `release`, which defmt always calls after `acquire`
            let restore = unsafe { critical_section::acquire() };
            if TAKEN.load(Ordering::Relaxed) {
                panic!("defmt logger taken reentrantly");
            }
            TAKEN.store(true, Ordering::Relaxed);
            // SAFETY: within the critical section, and TAKEN guarantees exclusive access
            unsafe {
                *STATE.cs_restore.get() = restore;
                (*STATE.encoder.get()).start_frame(write_pipe);
            }
        }

        unsafe fn flush() {
            // nothing to do: the pipe is drained asynchronously by `writer`
        }

        unsafe fn release() {
            // SAFETY: defmt guarantees this follows `acquire`, so we're in its critical section
            unsafe {
                (*STATE.encoder.get()).end_frame(write_pipe);
                TAKEN.store(false, Ordering::Relaxed);
                critical_section::release(*STATE.cs_restore.get());
            }
        }

        unsafe fn write(bytes: &[u8]) {
            // SAFETY: defmt guarantees this is between `acquire` and `release`
            unsafe { (*STATE.encoder.get()).write(bytes, write_pipe) }
        }
    }
}

/// The log port's sending half, from [`add_class`], run by [`writer`].
pub(crate) struct UsbLog<D: Driver<'static>> {
    sender: Sender<'static, D>,
}

/// Adds the CDC ACM function (an IAD plus communication and data interfaces) to the USB device.
pub(crate) fn add_class<D: Driver<'static>>(builder: &mut Builder<'static, D>) -> UsbLog<D> {
    static CDC_STATE: StaticCell<State<'static>> = StaticCell::new();
    let class = CdcAcmClass::new(builder, CDC_STATE.init(State::new()), MAX_PACKET_SIZE);
    // the OUT endpoint is unused: the host's writes are ignored
    let (sender, _receiver) = class.split();
    UsbLog { sender }
}

/// Sends logs to the host whenever the device is configured.
pub(crate) async fn writer<D: Driver<'static>>(log: UsbLog<D>) -> ! {
    let UsbLog { mut sender } = log;
    loop {
        sender.wait_connection().await;
        // Err: disabled by a USB reset, suspend or VBUS loss
        let _ = stream(&mut sender).await;
    }
}

/// Streams the pipe to the host until the endpoint is disabled.
async fn stream<D: Driver<'static>>(sender: &mut Sender<'static, D>) -> Result<(), EndpointError> {
    if cfg!(feature = "defmt-rtt") {
        for chunk in RTT_NOTICE.chunks(SEND_SIZE) {
            sender.write_packet(chunk).await?;
        }
    }

    let mut buf = [0u8; SEND_SIZE];
    loop {
        let n = PIPE.read(&mut buf).await;
        sender.write_packet(&buf[..n]).await?;
    }
}
