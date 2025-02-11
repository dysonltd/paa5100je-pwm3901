#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{clock::CpuClock, spi::master::Spi};
use paa5100je_pmw3901::Paa5100je;
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

    let spi = Spi::new(peripherals.SPI2, esp_hal::spi::master::Config::default())
        .unwrap()
        .with_mosi(peripherals.GPIO21)
        .with_miso(peripherals.GPIO20)
        .with_sck(peripherals.GPIO19)
        .with_cs(peripherals.GPIO18)
        .into_async();

    Paa5100je::new(spi, &mut timer0);

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}
