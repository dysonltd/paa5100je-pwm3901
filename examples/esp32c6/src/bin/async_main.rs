#![no_std]
#![no_main]

use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiDevice as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output},
    spi::master::Spi,
    timer::{timg::TimerGroup, OneShotTimer},
};
use paa5100je_pmw3901::{register, Paa5100je};
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

    let mut sensor = Paa5100je::new(spi, &mut sensor_timer).await.unwrap();

    // let mut buffer = [0u8; 5];
    // sensor.read(register::MOTION, &mut buffer).await.unwrap();

    // info!("{:?}", buffer);

    // for reg in 0..buffer.len() {
    //     sensor
    //         .read(register::MOTION + reg as u8, &mut [buffer[reg]])
    //         .await
    //         .unwrap();
    //     info!("{:?}", buffer[reg]);
    // }

    loop {
        // info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}
