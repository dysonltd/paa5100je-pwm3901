//! A no_std driver for the PAA5100JE near-field optical flow sensor
//!
#![no_std]

use defmt::{debug, error, trace, Format};
use embedded_hal_async::{
    delay::DelayNs,
    spi::{Operation, SpiDevice},
};

pub enum PixArtSensor<SPI: SpiDevice> {
    Paa5100je(SPI),
}

impl<SPI: SpiDevice> PixArtSensor<SPI> {
    pub async fn new_paa5100je(
        spi: SPI,
        delay_source: &mut impl DelayNs,
    ) -> Result<Self, SensorError> {
        let mut instance = Self::Paa5100je(spi);
        instance.init(delay_source).await?;

        Ok(instance)
    }

    pub async fn id(&mut self) -> Result<Id, SensorError> {
        Ok(Id {
            product_id: self.read(register::PRODUCT_ID).await?,
            revision: self.read(register::PRODUCT_REVISION).await?,
        })
    }

    pub async fn set_rotation(&mut self, rotation: RotationDegrees) -> Result<(), SensorError> {
        const SWAP_XY: u8 = 0b1000_0000;
        const INVERT_Y: u8 = 0b0100_0000;
        const INVERT_X: u8 = 0b0010_0000;

        let orientation = match rotation {
            RotationDegrees::_0 => SWAP_XY | INVERT_X | INVERT_Y,
            RotationDegrees::_90 => INVERT_Y,
            RotationDegrees::_180 => SWAP_XY,
            RotationDegrees::_270 => INVERT_X,
        };

        self.write(register::ORIENTATION, orientation).await
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum SensorError {
    Spi(embedded_hal_async::spi::ErrorKind),
    InvalidId(Id),
}

impl<T: embedded_hal_async::spi::Error> From<T> for SensorError {
    fn from(value: T) -> Self {
        Self::Spi(value.kind())
    }
}

#[derive(Debug, Clone, PartialEq, Format)]
pub struct Id {
    pub product_id: u8,
    pub revision: u8,
}

pub enum RotationDegrees {
    _0 = 0,
    _90 = 90,
    _180 = 180,
    _270 = 270,
}

impl<SPI: SpiDevice> PixArtSensor<SPI> {
    fn spi(&mut self) -> &mut SPI {
        match self {
            Self::Paa5100je(spi) => spi,
        }
    }
    async fn init(&mut self, delay_source: &mut impl DelayNs) -> Result<(), SensorError> {
        self.write(register::POWER_UP_RESET, 0x5A).await?;
        delay_source.delay_ms(20).await;

        // Not sure if this is necessary, but this is what the Python driver does and the datasheet is no help whatsoever so...
        for offset in 0..5u8 {
            self.read(register::MOTION + offset).await?;
        }

        self.calibrate(delay_source).await?;

        let id = self.id().await?;
        if id.product_id != 0x49 || id.revision > 0x01 {
            error!("Invalid product ID or revision for PAA5100JE: {:?}", id);
            return Err(SensorError::InvalidId(id));
        }
        debug!("Product ID: {}", id.product_id);
        debug!("Revision: {}", id.revision);
        Ok(())
    }

    async fn write(&mut self, register: u8, value: u8) -> Result<(), SensorError> {
        trace!("Writing {:02x} to register {:02x}", value, register);
        self.spi().write(&[register | 0x80, value]).await?;
        Ok(())
    }

    async fn read(&mut self, register: u8) -> Result<u8, SPI::Error> {
        let mut buffer = [0];
        self.spi()
            .transaction(&mut [Operation::Write(&[register]), Operation::Read(&mut buffer)])
            .await?;
        trace!("Read {} from register {:02x}", buffer[0], register);
        Ok(buffer[0])
    }

    async fn write_bulk(&mut self, reg_value_pairs: &[(u8, u8)]) -> Result<(), SensorError> {
        for (register, value) in reg_value_pairs {
            self.write(*register, *value).await?;
        }

        Ok(())
    }

    async fn calibrate(&mut self, delay_source: &mut impl DelayNs) -> Result<(), SensorError> {
        trace!("Injecting the secret sauce...");
        self.write_bulk(&[
            (0x7F, 0x00),
            (0x55, 0x01),
            (0x50, 0x07),
            (0x7F, 0x0E),
            (0x43, 0x10),
        ])
        .await?;

        let read_value = self.read(0x67).await?;
        let write_value = if 0 != read_value & 0b1000_0000 {
            0x04
        } else {
            0x02
        };
        self.write(0x48, write_value).await?;

        self.write_bulk(&[
            (0x7F, 0x00),
            (0x51, 0x7B),
            (0x50, 0x00),
            (0x55, 0x00),
            (0x7F, 0x0E),
        ])
        .await?;

        let read_value = self.read(0x73).await?;
        if read_value == 0 {
            let mut value1 = self.read(0x70).await?;

            // The logic of these following tweaks to value1 seem sus, but hey I've got no way of verifying so in Pimoroni we trust...
            if value1 <= 28 {
                value1 += 14
            }
            if value1 > 28 {
                value1 += 11
            }
            value1 = value1.clamp(0, 0x3F);

            let mut value2 = self.read(0x71).await?;

            value2 = (value2 * 45) / 100;

            self.write_bulk(&[
                (0x7F, 0x00),
                (0x61, 0xAD),
                (0x51, 0x70),
                (0x7F, 0x0E),
                (0x70, value1),
                (0x71, value2),
            ])
            .await?;
        }

        self.write_bulk(&[
            (0x7F, 0x00),
            (0x61, 0xAD),
            (0x7F, 0x03),
            (0x40, 0x00),
            (0x7F, 0x05),
            (0x41, 0xB3),
            (0x43, 0xF1),
            (0x45, 0x14),
            (0x5B, 0x32),
            (0x5F, 0x34),
            (0x7B, 0x08),
            (0x7F, 0x06),
            (0x44, 0x1B),
            (0x40, 0xBF),
            (0x4E, 0x3F),
            (0x7F, 0x08),
            (0x65, 0x20),
            (0x6A, 0x18),
            (0x7F, 0x09),
            (0x4F, 0xAF),
            (0x5F, 0x40),
            (0x48, 0x80),
            (0x49, 0x80),
            (0x57, 0x77),
            (0x60, 0x78),
            (0x61, 0x78),
            (0x62, 0x08),
            (0x63, 0x50),
            (0x7F, 0x0A),
            (0x45, 0x60),
            (0x7F, 0x00),
            (0x4D, 0x11),
            (0x55, 0x80),
            (0x74, 0x21),
            (0x75, 0x1F),
            (0x4A, 0x78),
            (0x4B, 0x78),
            (0x44, 0x08),
            (0x45, 0x50),
            (0x64, 0xFF),
            (0x65, 0x1F),
            (0x7F, 0x14),
            (0x65, 0x67),
            (0x66, 0x08),
            (0x63, 0x70),
            (0x7F, 0x15),
            (0x48, 0x48),
            (0x7F, 0x07),
            (0x41, 0x0D),
            (0x43, 0x14),
            (0x4B, 0x0E),
            (0x45, 0x0F),
            (0x44, 0x42),
            (0x4C, 0x80),
            (0x7F, 0x10),
            (0x5B, 0x02),
            (0x7F, 0x07),
            (0x40, 0x41),
            (0x70, 0x00),
        ])
        .await?;

        delay_source.delay_ms(10).await;

        self.write_bulk(&[
            (0x32, 0x44),
            (0x7F, 0x07),
            (0x40, 0x40),
            (0x7F, 0x06),
            (0x62, 0xF0),
            (0x63, 0x00),
            (0x7F, 0x0D),
            (0x48, 0xC0),
            (0x6F, 0xD5),
            (0x7F, 0x00),
            (0x5B, 0xA0),
            (0x4E, 0xA8),
            (0x5A, 0x50),
            (0x40, 0x80),
        ])
        .await?;

        delay_source.delay_ms(240).await;

        self.write_bulk(&[
            (0x7F, 0x14), // Enable LED_N pulsing
            (0x6F, 0x1C),
            (0x7F, 0x00),
        ])
        .await?;

        Ok(())
    }
}

#[allow(dead_code)]
pub mod register {
    pub const PRODUCT_ID: u8 = 0x00;
    pub const PRODUCT_REVISION: u8 = 0x01;
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
