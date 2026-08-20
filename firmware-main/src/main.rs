#![no_std]
#![no_main]

/// [ax1=P0.28, 8,
/// ax2=P0.29, 9,
/// trig=P0.03, 6,
/// i2c=TWIM0,
/// i2c.scl=P0.22, 26,
/// i2c.sda=P1.00, 27,
/// led=P0.30, 10,
/// sw=P0.05, 13,
/// gate_ctl=P0.07, 15,
/// bumper=P0.31, 11,
/// stick_pwr_gate=P1.10, 3,
/// trig_pwr_gate=P0.04, 12,
/// chg=P1.11, 2,
/// btns_io0=P0.19, 20,
/// swd=SWD,
/// swd.swclk=SWCLK, 31,
/// swd.swdio=SWDIO, 32,
/// 0=USBD,
/// 0.dp=D+, 24,
/// 0.dm=D-, 23]
use defmt_rtt as _;
use panic_probe as _;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::{ConstStaticCell, StaticCell};

mod battery;
mod ble_descriptors;
mod ble_peripheral;
mod bus;
mod leds;
mod imu;
mod joystick;
mod prelude;
mod util;
mod expander;
use crate::leds::leds_task;
use crate::battery::battery_task;
use crate::imu::imu_task;
use crate::joystick::{btns_task, joystick_task};
use crate::expander::Expander;
use crate::prelude::*;

use embassy_executor::Spawner;

// App-specific imports
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::saadc::{ChannelConfig, Config, Saadc};
use embassy_nrf::twim::{self, Twim};
use embassy_time::{Duration, Timer, Ticker};

use embassy_nrf::mode::Async;
use embassy_nrf::peripherals::RNG;
use embassy_nrf::{bind_interrupts, peripherals, rng, saadc};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};

type I2cBus = Mutex<CriticalSectionRawMutex, Twim<'static>>;
static I2C_BUS: StaticCell<I2cBus> = StaticCell::new();

type SharedExpander = Mutex<CriticalSectionRawMutex, Expander<I2cDevice<'static, CriticalSectionRawMutex, Twim<'static>>>>;
static EXPANDER: StaticCell<SharedExpander> = StaticCell::new();

bind_interrupts!(struct Irqs {
    // BLE stack interrupts
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    // application interrupts
    SAADC => saadc::InterruptHandler;
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

/// How many outgoing L2CAP buffers per link
const L2CAP_TXQ: u8 = 3;

/// How many incoming L2CAP buffers per link
const L2CAP_RXQ: u8 = 3;

/// Size of L2CAP packets
const L2CAP_MTU: usize = 72;

fn build_sdc<'d, const N: usize>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<Async>,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?
        .support_adv()
        .support_peripheral()
        .peripheral_count(1)?
        .buffer_cfg(L2CAP_MTU as u16, L2CAP_MTU as u16, L2CAP_TXQ, L2CAP_RXQ)?
        .build(p, rng, mpsl, mem)
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = embassy_nrf::init(Default::default());
    info!("Starting!");

    // initialize global state and shared peripherals
    let flash = Nvmc::new(p.NVMC);
    let bus = bus::init(flash);

    // Shared I2C bus
    let mut twi_config = twim::Config::default();
    twi_config.frequency = twim::Frequency::K400;
    static RAM_BUFFER: ConstStaticCell<[u8; 16]> = ConstStaticCell::new([0; 16]);
    let twi = Twim::new(
        p.TWISPI0,
        Irqs,
        p.P1_00,
        p.P0_22,
        twi_config,
        RAM_BUFFER.take(),
    );
    let i2c_bus = I2C_BUS.init(Mutex::new(twi));

    // initialise peripherals and tasks
    // power gate for battery connection
    let pwr_gate = Output::new(p.P0_07, Level::High, OutputDrive::Standard);

    // pull high to enable charging, low to disable
    let chg_en = Output::new(p.P1_11, Level::Low, OutputDrive::Standard);
    spawner.spawn(unwrap!(battery_task(bus, i2c_bus, chg_en)));

    spawner.spawn(unwrap!(imu_task(bus, i2c_bus)));

    let led = Output::new(p.P0_30, Level::Low, OutputDrive::Standard);
    let expander = Expander::new(I2cDevice::new(i2c_bus));
    let shared_expander = EXPANDER.init(Mutex::new(expander));
    spawner.spawn(unwrap!(leds_task(led, shared_expander)));

    spawner.spawn(unwrap!(btns_task(bus, shared_expander)));

    let stick_gate = Output::new(p.P1_10, Level::High, OutputDrive::Standard);
    let trig_gate = Output::new(p.P0_04, Level::High, OutputDrive::Standard);

    let _bumper = Input::new(p.P0_31, Pull::Up);
    let stick_sw = Input::new(p.P0_05, Pull::Up);

    let x_pin = ChannelConfig::single_ended(p.P0_28.reborrow());
    let y_pin = ChannelConfig::single_ended(p.P0_29.reborrow());
    let trig_pin = ChannelConfig::single_ended(p.P0_03.reborrow());
    let adc = Saadc::new(p.SAADC, Irqs, Config::default(), [x_pin, y_pin, trig_pin]);
    spawner.spawn(unwrap!(joystick_task(
        bus, stick_gate, trig_gate, adc, stick_sw
    )));

    // initialize BLE - black magic from trouble example
    let mpsl_p =
        mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::new(
        mpsl_p, Irqs, lfclk_cfg
    )));
    spawner.spawn(unwrap!(mpsl_task(&*mpsl)));

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let mut rng = rng::Rng::new(p.RNG, Irqs);

    let mut sdc_mem = sdc::Mem::<4720>::new();
    let sdc = unwrap!(build_sdc(sdc_p, &mut rng, mpsl, &mut sdc_mem));

    ble_peripheral::run(bus, sdc).await;
}

