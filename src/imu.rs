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
    const SAMPLE_RATE_HZ: u16 = 100;

    let mut imu = match async {
        let mut dev = Lsm6ds3tr::new(I2cDevice::new(&i2c_bus));
        if !dev.check().await.map_err(|_| ())? {
            error!("IMU ID check failed"); // TODO should be fatal
        }
        dev.reset().await.map_err(|_| ())?;
        dev.config_xl(
            lsm6ds3tr::OdrXl::new_at_least(SAMPLE_RATE_HZ),
            lsm6ds3tr::FsXl::G4,
        )
        .await
        .map_err(|_| ())?;
        dev.config_g(
            lsm6ds3tr::OdrG::new_at_least(SAMPLE_RATE_HZ),
            lsm6ds3tr::FsG::Dps500,
        )
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
        Timer::after_millis(500).await;
        async {
            let new_data = imu
                .new_data()
                .await
                .inspect_err(|e| error!("IMU: failed to read status: {:?}", e))
                .map_err(|_| ())?;

            let (x, y, z) = imu
                .read_xl_g()
                .await
                .inspect_err(|e| error!("IMU: failed to read status: {:?}", e))
                .map_err(|_| ())?;
            let (p, r, y) = imu
                .read_g_dps()
                .await
                .inspect_err(|e| error!("IMU: failed to read status: {:?}", e))
                .map_err(|_| ())?;

            info!(
                "IMU: nd {}, accel x={} y={} z={}, p={}, r={}, y={}",
                new_data, x, y, z, p, r, y
            );
            Ok::<(), ()>(())
        }
        .await
        .ok();
    }
}
