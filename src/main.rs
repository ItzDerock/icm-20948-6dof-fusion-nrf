#![no_std]
#![no_main]

mod comms;
mod icm;

use core::fmt::Write;
use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::{
    bind_interrupts,
    config::{Config, HfclkSource},
    gpio::{Level, Output},
    pac::{self},
    peripherals,
    twim::{self, Frequency, Twim},
    uarte,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Delay, Timer};
use heapless::String;
use icm20948_async::{AccRange, GyrUnit, IcmBuilder};
use static_cell::ConstStaticCell;

use crate::{
    comms::comms_run,
    icm::{IcmHandler, ImuData},
};

use {defmt_rtt as _, panic_reset as _};

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    UARTE0 => uarte::InterruptHandler<peripherals::UARTE0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.hfclk_source = HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);

    pac::CLOCK.tasks_hfclkstart().write_value(1);
    while pac::CLOCK.events_hfclkstarted().read() != 1 {}

    let mut led = Output::new(
        p.P1_15,
        Level::Low,
        embassy_nrf::gpio::OutputDrive::Standard,
    );

    led.set_high();

    info!("Initializing UART...");
    let mut config = uarte::Config::default();
    config.parity = uarte::Parity::EXCLUDED;
    config.baudrate = uarte::Baudrate::BAUD115200;
    let mut uarte = uarte::Uarte::new(p.UARTE0, p.P1_12, p.P1_11, Irqs, config);
    let _ = uarte.write("Init I2C...\r\n".as_bytes()).await;

    info!("Initializing I2C...");
    let mut i2c_config = twim::Config::default();
    i2c_config.frequency = Frequency::K400; // use fast mode
    i2c_config.scl_pullup = true;
    i2c_config.sda_pullup = true;

    static I2C_RAM_BUF: ConstStaticCell<[u8; 16]> = ConstStaticCell::new([0; 16]);
    let twi = Twim::new(
        p.TWISPI0,
        Irqs,
        // p.P0_04,
        // p.P0_05,
        p.P1_04,
        p.P1_06,
        i2c_config,
        I2C_RAM_BUF.take(),
    );

    let _ = uarte.write("Init ICM...\r\n".as_bytes()).await;
    info!("Initializing ICM...");

    let delay = Delay;
    let icm = IcmBuilder::new_i2c(twi, delay)
        .set_address(0x69)
        .gyr_unit(GyrUnit::Rps)
        .acc_range(AccRange::Gs16)
        .initialize_6dof()
        .await;

    if let Err(err) = icm {
        let mut err_msg: String<128> = String::new();
        let _ = core::write!(&mut err_msg, "IMU Setup Error: {:?}\r\n", err);
        let _ = uarte.write(err_msg.as_bytes()).await;
        panic!("IMU initialization failed");
    }

    let channel = Channel::<ThreadModeRawMutex, ImuData, 16>::new();
    let sender = channel.sender();
    let reciever = channel.receiver();

    let mut icm_handler = IcmHandler::new(icm.unwrap());

    // Wait for ICM sensor data pipeline to fill after wake-from-sleep
    Timer::after_millis(100).await;

    let _ = uarte.write("Calibrating gyro...\r\n".as_bytes()).await;
    if icm_handler.calibrate_gyro_if_still().await {
        let _ = uarte.write("Gyro calibrated.\r\n".as_bytes()).await;
    } else {
        let _ = uarte
            .write("Gyro cal skipped (sensor not still).\r\n".as_bytes())
            .await;
    }

    let _ = uarte.write("Program start.\r\n".as_bytes()).await;
    let imu_fut = icm_handler.run(sender);
    let comms_fut = comms_run(reciever, uarte);

    join(imu_fut, comms_fut).await;
}
