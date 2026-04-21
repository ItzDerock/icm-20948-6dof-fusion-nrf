use ahrs::{Ahrs, AhrsError, Madgwick};
use embassy_nrf::twim::Twim;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Sender};
use embassy_time::{Duration, Ticker};
use embedded_hal::i2c::Error as I2CError;
use icm20948_async::{I2cDevice, Icm20948};
use nalgebra::{Quaternion, Unit, Vector3};
use thiserror::Error;

// Run IMU at 1kHz
const IMU_SAMPLE_HZ: f32 = 1000.0;

// decrease = trust gyro more (if jitter, decrease beta)
// increase = trust accel more (if drift, increase beta)
const IMU_DMP_BETA: f32 = 0.1;

// How often to send data down the slip ring
const COMMS_HZ: u32 = 100;
const DECIMATION_FACTOR: u32 = (IMU_SAMPLE_HZ as u32) / COMMS_HZ;

#[derive(Debug, Error)]
pub enum ImuError {
    #[error("I2C Bus Error")]
    IcmFailure(embedded_hal_async::i2c::ErrorKind),

    #[error("Madgwick Computation Error")]
    DmpError(AhrsError),
}

pub struct IcmHandler<'d, MAG> {
    icm: Icm20948<I2cDevice<Twim<'d>>, MAG>,
    madgwick: Madgwick<f32>,
}

pub struct ImuData {
    pub accel: Vector3<f32>,
    pub quat: Unit<Quaternion<f32>>,
}

impl<'d, MAG> IcmHandler<'d, MAG> {
    pub fn new<'b>(icm: Icm20948<I2cDevice<Twim<'b>>, MAG>) -> IcmHandler<'b, MAG> {
        IcmHandler {
            icm,
            madgwick: Madgwick::new(1.0 / IMU_SAMPLE_HZ, IMU_DMP_BETA),
        }
    }

    pub async fn read_raw(&mut self) -> Option<([f32; 3], [f32; 3])> {
        self.icm.read_6dof().await.ok().map(|r| (r.acc, r.gyr))
    }

    /// Samples the IMU for one second and, if the sensor is stationary, writes the
    /// mean gyro bias to the ICM's hardware offset registers so it is cancelled in
    /// every subsequent read. Returns true if calibration was applied.
    pub async fn calibrate_gyro_if_still(&mut self) -> bool {
        const SAMPLES: usize = 1000;
        // Sum of per-axis variances in g², anything above this means the sensor
        // is being handled or motors are already running.
        const VARIANCE_THRESHOLD: f32 = 0.005;

        let mut acc_sum = Vector3::<f32>::zeros();
        let mut acc_sq_sum = Vector3::<f32>::zeros();
        let mut gyr_sum = Vector3::<f32>::zeros();
        let mut count = 0usize;

        let mut ticker = Ticker::every(Duration::from_hz(IMU_SAMPLE_HZ as u64));

        for _ in 0..SAMPLES {
            if let Ok(data) = self.icm.read_6dof().await {
                let a = Vector3::new(data.acc[0], data.acc[1], data.acc[2]);
                let g = Vector3::new(data.gyr[0], data.gyr[1], data.gyr[2]);
                acc_sum += a;
                acc_sq_sum += a.component_mul(&a);
                gyr_sum += g;
                count += 1;
            }
            ticker.next().await;
        }

        if count < SAMPLES / 2 {
            return false;
        }

        let n = count as f32;
        let acc_mean = acc_sum / n;
        let variance = (acc_sq_sum / n) - acc_mean.component_mul(&acc_mean);
        let total_variance = variance.x + variance.y + variance.z;

        if total_variance > VARIANCE_THRESHOLD {
            return false;
        }

        // Convert mean gyro bias (rad/s) back to raw ADC units for the offset registers.
        let gyr_mean = gyr_sum / n;
        let scalar = self.icm.gyr_scalar();
        let offsets = [
            (gyr_mean.x / scalar) as i16,
            (gyr_mean.y / scalar) as i16,
            (gyr_mean.z / scalar) as i16,
        ];

        self.icm.set_gyr_offsets(offsets).await.is_ok()
    }

    pub async fn process_single(&mut self) -> Result<ImuData, ImuError> {
        let result = self
            .icm
            .read_6dof()
            .await
            .map_err(|e| ImuError::IcmFailure(e.kind()))?;

        let acc = Vector3::new(result.acc[0], result.acc[1], result.acc[2]);
        let gyr = Vector3::new(result.gyr[0], result.gyr[1], result.gyr[2]);

        let quat = self
            .madgwick
            .update_imu(&gyr, &acc)
            .map_err(|e| ImuError::DmpError(e))?;

        let data = ImuData {
            accel: acc,
            quat: quat.clone(),
        };

        Ok(data)
    }

    pub async fn run(&mut self, comms_tx: Sender<'d, ThreadModeRawMutex, ImuData, 16>) -> ! {
        let mut ticker = Ticker::every(Duration::from_hz(IMU_SAMPLE_HZ as u64));
        let mut tick_counter: u32 = 0;

        loop {
            match self.process_single().await {
                Ok(data) => {
                    tick_counter += 1;
                    if tick_counter >= DECIMATION_FACTOR {
                        tick_counter = 0;
                        let _ = comms_tx.try_send(data);
                    }
                }
                Err(ImuError::IcmFailure(_)) => {
                    // skip this sample
                }
                Err(ImuError::DmpError(_)) => {
                    // filter got bad data, reset.
                    self.madgwick = Madgwick::new(1.0 / IMU_SAMPLE_HZ, IMU_DMP_BETA);
                }
            }

            ticker.next().await;
        }
    }
}
