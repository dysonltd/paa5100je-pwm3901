//! A no_std driver for the PAA5100JE near-field optical flow sensor
//!
#![no_std]

use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    spi::{SpiBus, SpiDevice},
};

pub struct Paa5100je<SPI>
where
    SPI: SpiDevice,
{
    spi: SPI,
}

impl<SPI> Paa5100je<SPI>
where
    SPI: SpiDevice,
{
    pub fn new<BUS: SpiBus, CS: OutputPin>(
        bus: BUS,
        mut cs_pin: CS,
        delay_source: &mut impl DelayNs,
    ) -> Result<Self, Error<CS::Error>> {
        cs_pin.set_low()?;
        delay_source.delay_ms(50);
        cs_pin.set_high()?;

        let spi = SPI::new(bus, cs_pin);
        todo!()
    }
}

pub enum Error<GPIOE> {
    Gpio(GPIOE),
}

impl<GPIOE> From<GPIOE> for Error<GPIOE> {
    fn from(value: GPIOE) -> Self {
        Self::Gpio(value)
    }
}

trait PixArtSensor {}

impl<SPI> PixArtSensor for Paa5100je<SPI> where SPI: SpiDevice {}

enum Register {
    ProductId = 0x00,
    RevisionId = 0x01,
    Motion = 0x02,
    DeltaXL = 0x03,
    DeltaXH = 0x04,
    DeltaYL = 0x05,
    DeltaYH = 0x06,
    Squal = 0x07,
    RawDataSum = 0x08,
    MaximumRawData = 0x09,
    MinimumRawData = 0x0A,
    ShutterLower = 0x0B,
    ShutterUpper = 0x0C,
    Observation = 0x15,
    MotionBurst = 0x16,

    PowerUpReset = 0x3A,
    Shutdown = 0x3B,

    RawDataGrab = 0x58,
    RawDataGrabStatus = 0x59,

    Orientation = 0x5B,
    InverseProductId = 0x5F,
}

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
