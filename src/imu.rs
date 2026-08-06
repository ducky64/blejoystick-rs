use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
type I2cBus = Mutex<CriticalSectionRawMutex, Twim<'static>>;

use crate::async_to_blocking::AsyncToBlockingI2c;
use crate::bus::GlobalBus;
use crate::prelude::*;
use defmt::Display2Format;

use lsm6ds3tr::{interface::I2cInterface, LsmSettings, LSM6DS3TR};

#[embassy_executor::task]
pub(crate) async fn imu_task(bus: &'static GlobalBus, i2c_bus: &'static I2cBus) {
    let i2c = AsyncToBlockingI2c::new(i2c_bus);
    let mut imu = LSM6DS3TR::new(I2cInterface::new(i2c)).with_settings(LsmSettings::basic());
    match imu.init() {
        Ok(dev) => dev,
        _ => {
            error!("IMU: failed to initialize");
            return;
        }
    };

    loop {
        if let (Ok(xyz_a), Ok(xyz_g)) = (imu.read_accel(), imu.read_gyro()) {
            info!("IMU {} {}", Display2Format(&xyz_a), Display2Format(&xyz_g));
        }
        Timer::after_millis(1000).await;
    }
}
