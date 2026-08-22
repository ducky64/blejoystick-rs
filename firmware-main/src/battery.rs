use core::sync::atomic::Ordering;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_futures::select::select;
use embassy_nrf::gpio::Output;
use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::semaphore::Semaphore;
use embassy_time::{Duration, Timer, Ticker};
type I2cBus = Mutex<CriticalSectionRawMutex, Twim<'static>>;

use crate::bus::GlobalBus;
use crate::prelude::*;
use crate::util::interpolate_1d;
use ina219::address::Address;
use ina219::calibration::{IntCalibration, MicroAmpere};
use ina219::configuration::{Configuration, MeasuredSignals, OperatingMode};
use ina219::AsyncIna219;

// generic approximation; TODO characterization or authoritative source
const LIPO_V_SOC: [(u16, u16); 7] = [
    (3300, 0),
    (3600, 8),
    (3700, 18),
    (3770, 40),
    (3850, 60),
    (4000, 84),
    (4200, 100),
];

#[embassy_executor::task]
pub(crate) async fn battery_task(
    bus: &'static GlobalBus,
    i2c_bus: &'static I2cBus,
) {
    let vbus_mv_sender = bus.vbat_mv.sender();
    let vbus_soc_sender = bus.vbat_soc.sender();
    let calib = IntCalibration::new(MicroAmpere(100_000), 100_000).unwrap();
    let config = Configuration {
        operating_mode: OperatingMode::Triggered(MeasuredSignals::BusVoltage),
        ..Configuration::default()
    };

    let mut ina = match async {
        // discard errors since they are not consistently typed
        let mut dev = AsyncIna219::new_calibrated(
            I2cDevice::new(&i2c_bus),
            Address::from_byte(0x40).unwrap(),
            calib,
        )
        .await
        .map_err(|_| ())?;
        dev.set_configuration(config).await.map_err(|_| ())?;
        Ok::<_, ()>(dev)
    }
    .await
    {
        Ok(dev) => dev,
        Err(_) => {
            error!("INA219: failed to initialize");
            return;
        }
    };

    let conversion_time_us = config.conversion_time_us().unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(5000));
    loop {
        ina.trigger().await.unwrap();
        Timer::after_micros(conversion_time_us as u64).await;

        let meas = ina.next_measurement().await.unwrap();
        if let Some(meas) = meas {
            let soc = interpolate_1d(&LIPO_V_SOC, meas.bus_voltage.voltage_mv()) as u8;
            info!(
                "INA219: v={}, i={}, soc={}",
                meas.bus_voltage, meas.current, soc
            );
            vbus_mv_sender.send(meas.bus_voltage.voltage_mv());
            vbus_soc_sender.send(soc);
        } else {
            warn!("INA219: measurement unavailable");
        }

        ticker.next().await;
    }
}


#[embassy_executor::task]
pub(crate) async fn charger_task(
    bus: &'static GlobalBus,
    mut chg_en: Output<'static>,
) { 
    let usb_powered_snd = bus.usb_powered.sender();
    let charging_snd = bus.charging.sender();
    chg_en.set_low();  // disabled by default

    let mut ticker = Ticker::every(Duration::from_millis(250));
    loop {
        let vbus_present = embassy_nrf::pac::POWER.usbregstatus().read().vbusdetect();
        if bus.usb_powered.try_get().map(|x| x != vbus_present).unwrap_or(true) {
            usb_powered_snd.send(vbus_present);
        }
        if vbus_present {
            bus.activity();  // keep alive if USB power present
        }

        // TODO configable charge threshold
        let charging = if vbus_present && bus.vbat_soc.try_get().map(|soc| soc <= 80).unwrap_or(false) {
            chg_en.set_high();
            true
        } else {
            chg_en.set_low();
            false
        };
        if bus.charging.try_get().map(|x| x != charging).unwrap_or(true) {
            charging_snd.send(charging);
        }

        ticker.next().await;
    }
}


#[embassy_executor::task]
pub(crate) async fn power_task(
    bus: &'static GlobalBus,
    mut pwr_gate: Output<'static>,
) {
    let mut shutdown_rcv = bus.shutdown_requested.receiver().unwrap();

    pwr_gate.set_high();

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        select(shutdown_rcv.changed(), ticker.next()).await;

        let soc_shutoff = bus.vbat_soc.try_get().map(|soc| soc <= 5).unwrap_or(false);
        let activity_shutoff = embassy_time::Instant::now().as_micros() as u64 > bus.last_activity.load(Ordering::Relaxed) + 5_000_000;
        let requested_shutoff = bus.shutdown_requested.try_get().unwrap_or(false);

        if soc_shutoff || activity_shutoff || requested_shutoff {
            break;
        }
    }

    info!("shutdown requested");
    bus.shutdown_requested.sender().send(true);

    select(
        bus.shutdown_locks.acquire_all(GlobalBus::SHUTDOWN_LOCKS),
        Timer::after(Duration::from_millis(1000)),  // timeout
    ).await;
    
    info!("power gate off");
    pwr_gate.set_low();
}
