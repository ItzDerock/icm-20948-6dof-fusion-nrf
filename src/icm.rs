use ahrs::{Ahrs, AhrsError, Madgwick};
use defmt::info;
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

// Auto-calibration tunables. continuously re-measure gyro bias
// whenever the sensor is stationary, accumulate the residual into the ICM's
// hardware offset registers so yaw drift stays bounded as bias changes with
// temperature/time.
const AUTO_CAL_STILL_VAR_THRESHOLD: f32 = 0.005;
// Per-sample |acc - EMA mean|^2 abort threshold (g^2); catches transients before
// the EMA variance can rise.
const AUTO_CAL_JERK_THRESHOLD_SQ: f32 = 0.02;
const AUTO_CAL_STILL_REQUIRED: u32 = 2_000;
const AUTO_CAL_SAMPLES: u32 = 500;
const AUTO_CAL_COOLDOWN: u32 = 10_000;
// EMA weight: ~128-sample effective window, fast enough to detect motion mid-pass.
const AUTO_CAL_EMA_ALPHA: f32 = 1.0 / 128.0;

#[derive(Debug, Error)]
pub enum ImuError {
    #[error("I2C Bus Error")]
    IcmFailure(embedded_hal_async::i2c::ErrorKind),

    #[error("Madgwick Computation Error")]
    DmpError(AhrsError),
}

enum AutoCalState {
    Monitoring,
    Calibrating { sum: Vector3<f32>, count: u32 },
}

pub struct IcmHandler<'d, MAG> {
    icm: Icm20948<I2cDevice<Twim<'d>>, MAG>,
    madgwick: Madgwick<f32>,

    // Current hardware gyro offsets (raw ADC counts). Auto-cal accumulates into
    // this rather than overwriting, because set_gyr_offsets is absolute.
    gyr_offsets: [i16; 3],

    acc_ema_mean: Vector3<f32>,
    acc_ema_var: Vector3<f32>,
    ema_initialised: bool,

    still_samples: u32,
    cooldown: u32,
    auto_cal: AutoCalState,
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
            gyr_offsets: [0; 3],
            acc_ema_mean: Vector3::zeros(),
            acc_ema_var: Vector3::zeros(),
            ema_initialised: false,
            still_samples: 0,
            cooldown: 0,
            auto_cal: AutoCalState::Monitoring,
        }
    }

    pub async fn read_raw(&mut self) -> Option<([f32; 3], [f32; 3])> {
        self.icm.read_6dof().await.ok().map(|r| (r.acc, r.gyr))
    }

    /// Samples the IMU for one second and, if the sensor is stationary, writes the
    /// mean gyro bias to the ICM's hardware offset registers so it is cancelled in
    /// every subsequent read. Returns the absolute offsets that were applied, or
    /// None if motion or I/O errors aborted the calibration.
    pub async fn calibrate_gyro_if_still(&mut self) -> Option<[i16; 3]> {
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
            return None;
        }

        let n = count as f32;
        let acc_mean = acc_sum / n;
        let variance = (acc_sq_sum / n) - acc_mean.component_mul(&acc_mean);
        let total_variance = variance.x + variance.y + variance.z;

        if total_variance > VARIANCE_THRESHOLD {
            return None;
        }

        // Convert mean gyro bias (rad/s) back to raw ADC units for the offset registers.
        let gyr_mean = gyr_sum / n;
        let scalar = self.icm.gyr_scalar();
        let offsets = [
            (gyr_mean.x / scalar) as i16,
            (gyr_mean.y / scalar) as i16,
            (gyr_mean.z / scalar) as i16,
        ];

        if self.icm.set_gyr_offsets(offsets).await.is_ok() {
            self.gyr_offsets = offsets;
            Some(offsets)
        } else {
            None
        }
    }

    async fn update_auto_cal(&mut self, acc: &Vector3<f32>, gyr: &Vector3<f32>) {
        if !self.ema_initialised {
            self.acc_ema_mean = *acc;
            self.acc_ema_var = Vector3::zeros();
            self.ema_initialised = true;
        } else {
            self.acc_ema_mean += AUTO_CAL_EMA_ALPHA * (acc - self.acc_ema_mean);
            let dev = acc - self.acc_ema_mean;
            let sq = dev.component_mul(&dev);
            self.acc_ema_var += AUTO_CAL_EMA_ALPHA * (sq - self.acc_ema_var);
        }

        if self.cooldown > 0 {
            self.cooldown -= 1;
        }

        let var_sum = self.acc_ema_var.x + self.acc_ema_var.y + self.acc_ema_var.z;
        let jerk = acc - self.acc_ema_mean;
        let jerk_sq = jerk.dot(&jerk);
        let motion = var_sum > AUTO_CAL_STILL_VAR_THRESHOLD || jerk_sq > AUTO_CAL_JERK_THRESHOLD_SQ;

        if motion {
            self.still_samples = 0;
            self.auto_cal = AutoCalState::Monitoring;
            return;
        }

        self.still_samples = self.still_samples.saturating_add(1);

        match &mut self.auto_cal {
            AutoCalState::Monitoring => {
                if self.cooldown == 0 && self.still_samples >= AUTO_CAL_STILL_REQUIRED {
                    self.auto_cal = AutoCalState::Calibrating {
                        sum: Vector3::zeros(),
                        count: 0,
                    };
                }
            }
            AutoCalState::Calibrating { sum, count } => {
                *sum += gyr;
                *count += 1;
                if *count >= AUTO_CAL_SAMPLES {
                    let mean = *sum / (*count as f32);
                    self.auto_cal = AutoCalState::Monitoring;
                    self.commit_gyr_bias_delta(mean).await;
                }
            }
        }
    }

    async fn commit_gyr_bias_delta(&mut self, delta_rad_s: Vector3<f32>) {
        let scalar = self.icm.gyr_scalar();
        let dx = (delta_rad_s.x / scalar) as i32;
        let dy = (delta_rad_s.y / scalar) as i32;
        let dz = (delta_rad_s.z / scalar) as i32;

        let new_offsets = [
            (self.gyr_offsets[0] as i32 + dx).clamp(i16::MIN as i32, i16::MAX as i32) as i16,
            (self.gyr_offsets[1] as i32 + dy).clamp(i16::MIN as i32, i16::MAX as i32) as i16,
            (self.gyr_offsets[2] as i32 + dz).clamp(i16::MIN as i32, i16::MAX as i32) as i16,
        ];

        if self.icm.set_gyr_offsets(new_offsets).await.is_ok() {
            self.gyr_offsets = new_offsets;
            info!(
                "auto-cal: dxyz=[{},{},{}] offsets=[{},{},{}]",
                dx, dy, dz, new_offsets[0], new_offsets[1], new_offsets[2]
            );
        }
        self.cooldown = AUTO_CAL_COOLDOWN;
    }

    pub async fn process_single(&mut self) -> Result<ImuData, ImuError> {
        let result = self
            .icm
            .read_6dof()
            .await
            .map_err(|e| ImuError::IcmFailure(e.kind()))?;

        let acc = Vector3::new(result.acc[0], result.acc[1], result.acc[2]);
        let gyr = Vector3::new(result.gyr[0], result.gyr[1], result.gyr[2]);

        self.update_auto_cal(&acc, &gyr).await;

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
