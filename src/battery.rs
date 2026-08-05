use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::gpio::Output;
use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
type I2cBus = Mutex<CriticalSectionRawMutex, Twim<'static>>;

use crate::bus::GlobalBus;
use crate::prelude::*;
use ina219::address::Address;
use ina219::calibration::{IntCalibration, MicroAmpere};
use ina219::configuration::{Configuration, MeasuredSignals, OperatingMode};
use ina219::AsyncIna219;

#[embassy_executor::task]
pub(crate) async fn battery_task(
    bus: &'static GlobalBus,
    i2c_bus: &'static I2cBus,
    mut chg_en: Output<'static>,
) {
    let vbus_sender = bus.vbat.sender();
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

    // TODO charging control with voltage cutoff
    chg_en.set_high();

    let conversion_time_us = config.conversion_time_us().unwrap();
    loop {
        ina.trigger().await.unwrap();
        Timer::after_micros(conversion_time_us as u64).await;

        let meas = ina.next_measurement().await.unwrap();
        if let Some(meas) = meas {
            info!("INA219: v={}, i={}", meas.bus_voltage, meas.current);
            vbus_sender.send(meas.bus_voltage.voltage_mv());
        } else {
            warn!("INA219: measurement unavailable");
        }
        Timer::after_millis(1000).await;
    }
}
