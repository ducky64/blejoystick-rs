use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
type I2cBus = Mutex<CriticalSectionRawMutex, Twim<'static>>;

use crate::bus::GlobalBus;
use crate::prelude::*;

#[embassy_executor::task]
pub(crate) async fn imu_task(bus: &'static GlobalBus, i2c_bus: &'static I2cBus) {
    // let mut imu = Lsm6ds33::new(I2cDevice::new(&i2c_bus), 0x6A).await.unwrap();
    info!("IMU: initialized");
    loop {
        Timer::after_millis(1000).await;
    }
}
