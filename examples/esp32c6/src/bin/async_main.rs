#![no_std]
#![no_main]

use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::Timer;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, Level, Output, Pull},
    spi::master::Spi,
    timer::{timg::TimerGroup, OneShotTimer},
};
use paa5100je_pmw3901::{MotionDelta, PixArtSensor, RotationDegrees};
use {defmt_rtt as _, esp_backtrace as _};

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    // TODO: Spawn some tasks
    let _ = spawner;

    let frame_capture = Input::new(peripherals.GPIO8, Pull::Up);
    let wake = Input::new(peripherals.GPIO9, Pull::Up);

    let spi_bus: Mutex<NoopRawMutex, Spi<'_, esp_hal::Async>> = Mutex::new(
        Spi::new(peripherals.SPI2, esp_hal::spi::master::Config::default())
            .unwrap()
            .with_mosi(peripherals.GPIO21)
            .with_miso(peripherals.GPIO20)
            .with_sck(peripherals.GPIO19)
            .into_async(),
    );
    let cs = Output::new(peripherals.GPIO18, Level::High);
    let spi = SpiDevice::new(&spi_bus, cs);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0);
    let mut sensor_timer = OneShotTimer::new(timer_group0.timer0).into_async();

    let mut sensor = PixArtSensor::new_paa5100je(spi, &mut sensor_timer)
        .await
        .unwrap();

    sensor.set_rotation(RotationDegrees::_0).await.unwrap();

    info!("Sensor ID: {:?}", sensor.id().await.unwrap());

    loop {
        match sensor.get_motion().await {
            Ok(MotionDelta { x: 0, y: 0 }) => (),
            Ok(motion) => {
                let mut buffer = [0; 32];
                let string = format_no_std::show(
                    &mut buffer,
                    format_args!("x: {:4}, y: {:4}", motion.x, motion.y),
                )
                .unwrap();
                info!("{}", string)
            }
            Err(_) => (),
        }

        if frame_capture.is_low() {
            info!("Capturing frame...");
            let frame = sensor.capture_frame(&mut sensor_timer).await.unwrap();
            // I'd like to figure out how to draw this to the terminal with ascii but I don't have time right now
            info!("Frame:");
            for row in 0..35 {
                let start = row * 35;
                info!("{:?}", frame[start..start + 34]);
            }
            Timer::after_secs(2).await;
            info!("Continuing motion capture");
        }

        if wake.is_low() {
            info!("Waking the sensor...");
            sensor.wake(&mut sensor_timer).await.unwrap();
            Timer::after_secs(1).await;
            info!("Resuming motion capture");
        }
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}
