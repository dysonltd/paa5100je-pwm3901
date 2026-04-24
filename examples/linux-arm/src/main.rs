use linux_embedded_hal::Delay;
use linux_embedded_hal::SpidevDevice;
use linux_embedded_hal::spidev::SpidevOptions;
use paa5100je_pmw3901::PixArtSensor;
use std::thread::sleep;
use std::time::Duration;

fn main() {
    let mut options = SpidevOptions::default();
    options.max_speed_hz = Some(1_000_000);

    let mut spi = SpidevDevice::open("/dev/spidev0.0").unwrap();
    spi.configure(&options).unwrap();

    let mut sensor = PixArtSensor::new_paa5100je(spi, &mut Delay).unwrap();

    println!("PixArtSensor ID: {:?}", sensor.id());

    loop {
        match sensor.get_motion() {
            Ok(motion) => {
                println!("Motion delta - x:{}, y:{}", motion.x, motion.y);
            }
            Err(e) => {
                println!("Error: {:?}", e);
            }
        }

        sleep(Duration::from_millis(1_000));
    }
}
