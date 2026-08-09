use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
type I2cBus = Mutex<CriticalSectionRawMutex, Twim<'static>>;

use crate::bus::GlobalBus;
use crate::prelude::*;

use lsm6ds3trc::{self, Lsm6ds3tr};

#[embassy_executor::task]
pub(crate) async fn imu_task(bus: &'static GlobalBus, i2c_bus: &'static I2cBus) {
    let mut imu = match async {
        let mut dev = Lsm6ds3tr::new_i2c(I2cDevice::new(&i2c_bus));
        if !dev.check().await.map_err(|_| ())? {
            error!("IMU ID check failed"); // TODO should be fatal
        }
        dev.reset().await.map_err(|_| ())?;
        dev.config_xl(lsm6ds3trc::OdrXl::new_at_least(5), lsm6ds3trc::FsXl::G4)
            .await
            .map_err(|_| ())?;
        dev.config_g(lsm6ds3trc::OdrG::new_at_least(100), lsm6ds3trc::FsG::Dps500)
            .await
            .map_err(|_| ())?;
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
        Timer::after_millis(10).await;
        async {
            let new_data = imu
                .new_data()
                .await
                .inspect_err(|e| error!("IMU: failed to read status: {:?}", e))
                .map_err(|_| ())?;

            if new_data.accelerometer {
                let (x, y, z) = imu
                    .read_xl_g()
                    .await
                    .inspect_err(|e| error!("IMU: failed to read accelerometer: {:?}", e))
                    .map_err(|_| ())?;
                debug!("IMU: x={}, y={}, z={}", x, y, z);
            }
            if new_data.gyro {
                let (pitch, roll, yaw) = imu
                    .read_g_dps()
                    .await
                    .inspect_err(|e| error!("IMU: failed to read gyro: {:?}", e))
                    .map_err(|_| ())?;
                debug!("IMU: p={}, r={}, y={}", pitch, roll, yaw);
            }

            Ok::<(), ()>(())
        }
        .await
        .ok();
    }
}
