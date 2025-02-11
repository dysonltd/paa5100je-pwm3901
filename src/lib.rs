//! A no_std driver for the PAA5100JE near-field optical flow sensor
//!
#![no_std]

use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

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
    pub async fn new(spi: SPI, delay_source: &mut impl DelayNs) -> Result<Self, SPI::Error> {
        let mut instance = Self { spi };

        instance.write(register::POWER_UP_RESET, 0x5A).await?;
        delay_source.delay_ms(20).await;

        Ok(instance)
    }
}

impl<SPI> Paa5100je<SPI>
where
    SPI: SpiDevice,
{
    async fn write(&mut self, register: u8, value: u8) -> Result<(), SPI::Error> {
        self.spi.write(&[register, value]).await
    }

    pub async fn read(&mut self, register: u8, buffer: &mut [u8]) -> Result<(), SPI::Error> {
        self.spi.transfer(buffer, &[register]).await
    }
}

trait PixArtSensor {}

impl<SPI> PixArtSensor for Paa5100je<SPI> where SPI: SpiDevice {}

#[allow(dead_code)]
pub mod register {
    pub const PRODUCT_ID: u8 = 0x00;
    pub const REVISION_ID: u8 = 0x01;
    pub const MOTION: u8 = 0x02;
    pub const DELTAX_L: u8 = 0x03;
    pub const DELTAX_H: u8 = 0x04;
    pub const DELTAY_L: u8 = 0x05;
    pub const DELTAY_H: u8 = 0x06;
    pub const SQUAL: u8 = 0x07;
    pub const RAW_DATA_SUM: u8 = 0x08;
    pub const MAXIMUM_RAW_DATA: u8 = 0x09;
    pub const MINIMUM_RAW_DATA: u8 = 0x0A;
    pub const SHUTTER_LOWER: u8 = 0x0B;
    pub const SHUTTER_UPPER: u8 = 0x0C;
    pub const OBSERVATION: u8 = 0x15;
    pub const MOTION_BURST: u8 = 0x16;

    pub const POWER_UP_RESET: u8 = 0x3A;
    pub const SHUTDOWN: u8 = 0x3B;

    pub const RAW_DATA_GRAB: u8 = 0x58;
    pub const RAW_DATA_GRAB_STATUS: u8 = 0x59;

    pub const ORIENTATION: u8 = 0x5B;
    pub const INVERSE_PRODUCT_ID: u8 = 0x5F;
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
