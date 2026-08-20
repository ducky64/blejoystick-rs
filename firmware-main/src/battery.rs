use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::gpio::Output;
use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
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
    mut chg_en: Output<'static>,
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

    // TODO charging control with voltage cutoff
    chg_en.set_high();

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
