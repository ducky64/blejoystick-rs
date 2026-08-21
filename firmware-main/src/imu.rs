use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
type I2cBus = Mutex<CriticalSectionRawMutex, Twim<'static>>;

use crate::bus::GlobalBus;
use crate::prelude::*;

use lsm6ds3trc::{self, Lsm6ds3tr};
// use qmc5883p::{
//     Mode, OutputDataRate, OverSampleRate, OverSampleRatio1, Qmc5883PConfig, Qmc5883p, Range,
// };

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

    // let mut mag = match async {
    //     let config = Qmc5883PConfig::default()
    //         .with_mode(Mode::Continuous)
    //         .with_odr(OutputDataRate::Hz10)
    //         .with_range(Range::Gauss8)
    //         .with_osr1(OverSampleRatio1::Ratio4)
    //         .with_osr2(OverSampleRate::Rate4);
    //     let mut sensor = Qmc5883p::new(I2cDevice::new(&i2c_bus));
    //     sensor.init(config).await.unwrap();
    //     Ok::<_, ()>(sensor)
    // }
    // .await
    // {
    //     Ok(dev) => dev,
    //     Err(_) => {
    //         error!("Magnetometer: failed to initialize");
    //         return;
    //     }
    // };

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

        // async {
        //     // Read magnetic field data
        //     let [x, y, z] = mag
        //         .read_x_y_z()
        //         .await
        //         .inspect_err(|e| error!("Mag: failed to read XYZ: {:?}", e))
        //         .map_err(|_| ())?;
        //     let magnitude = mag
        //         .read_magnitude()
        //         .await
        //         .inspect_err(|e| error!("Mag: failed to read magnitude: {:?}", e))
        //         .map_err(|_| ())?;
        //     debug!(
        //         "Magnetometer: x={}, y={}, z={}, magnitude={}",
        //         x, y, z, magnitude
        //     );

        //     Ok::<(), ()>(())
        // }
        // .await
        // .ok();
    }
}
