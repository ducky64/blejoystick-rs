use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
type I2cBus = Mutex<CriticalSectionRawMutex, Twim<'static>>;

use crate::bus::GlobalBus;
use crate::lsm6ds3tr::Lsm6ds3tr;
use crate::prelude::*;

#[embassy_executor::task]
pub(crate) async fn imu_task(bus: &'static GlobalBus, i2c_bus: &'static I2cBus) {
    let mut imu = match async {
        let mut dev = Lsm6ds3tr::new(I2cDevice::new(&i2c_bus));
        // let id = dev.read_whoami().await.map_err(|_| ())?;
        // info!("IMU: got id {}", id);
        if !dev.check().await.map_err(|_| ())? {
            warn!("IMU check failed"); // TODO should be fatal
        }
        Ok::<_, ()>(dev)
    }
    .await
    {
        Ok(dev) => dev,
        Err(_) => {
            error!("IMU: failed to initialize");
            return;
        }
    };

    info!("IMU: initialized");
    loop {
        Timer::after_millis(1000).await;
    }
}
