use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
type I2cBus = Mutex<CriticalSectionRawMutex, Twim<'static>>;

use crate::bus::GlobalBus;
use crate::lsm6ds3tr::{self, Lsm6ds3tr};
use crate::prelude::*;

#[embassy_executor::task]
pub(crate) async fn imu_task(bus: &'static GlobalBus, i2c_bus: &'static I2cBus) {
    let mut imu = match async {
        let mut dev = Lsm6ds3tr::new(I2cDevice::new(&i2c_bus));
        dev.config_xl(lsm6ds3tr::OdrXl::Hz104, lsm6ds3tr::FsXl::G4)
            .await
            .map_err(|_| ())?;
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
        Timer::after_millis(500).await;
        let new_data = match imu.new_data().await {
            Ok(new_data) => new_data,
            Err(_) => {
                error!("IMU: failed to read status");
                continue;
            }
        };
        let temp_raw = match imu.read_temp_raw().await {
            Ok(data) => data,
            Err(_) => {
                error!("IMU: failed to read temperature");
                continue;
            }
        };
        let (x, y, z) = match imu.read_xl_raw().await {
            Ok(data) => data,
            Err(_) => {
                error!("IMU: failed to read accelerometer");
                continue;
            }
        };
        info!(
            "IMU: nd {}, accel temp={}, x={} y={} z={}",
            new_data, temp_raw, x, y, z
        );
    }
}
