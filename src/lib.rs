//! A no_std driver for the PAA5100JE near-field optical flow sensor
//!
//! Yes, there are magic numbers everywhere in this driver.
//! No, I do not know what they mean.
//! The datasheet gives next to no useful information, the best I can do is point you to the
//! Pimoroni Python driver at https://github.com/pimoroni/pmw3901-python/blob/main/pmw3901/__init__.py
#![no_std]

use bytemuck::{Pod, Zeroable};
use defmt::{debug, error, trace, Format};
use embedded_hal_async::{
    delay::DelayNs,
    spi::{Operation, SpiDevice},
};

/// Size of the sensor: 35 x 35 pixels (presumably. The data sheet doesn't say...)
const FRAME_SIZE: usize = 1225;

/// Enumeration of the sensor type. This acts as the sensor instance.
pub enum PixArtSensor<SPI: SpiDevice> {
    /// PAA5100JE Near-field Optical Flow Sensor
    Paa5100je(SPI),
}

impl<SPI: SpiDevice> PixArtSensor<SPI> {
    /// Instantiate and initialise a new PAA5100JE Near-field Optical Flow Sensor
    pub async fn new_paa5100je(
        spi: SPI,
        delay_source: &mut impl DelayNs,
    ) -> Result<Self, SensorError> {
        let mut instance = Self::Paa5100je(spi);
        instance.init(delay_source).await?;

        Ok(instance)
    }

    /// Get the product ID and revision number
    pub async fn id(&mut self) -> Result<Id, SensorError> {
        Ok(Id {
            product_id: self.read(register::PRODUCT_ID).await?,
            revision: self.read(register::PRODUCT_REVISION).await?,
        })
    }

    /// Set the sensor's rotation
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

        debug!("Setting rotation to {:?}", rotation);
        self.write(register::ORIENTATION, orientation).await
    }

    /// Retrieve motion data from the sensor.
    ///
    /// The raw motion data is validated before being returned.
    /// If the raw data fails validation, SensorError::InvalidMotion is returned.
    pub async fn get_motion(&mut self) -> Result<MotionDelta, SensorError> {
        let mut buffer = [0; 12];
        self.spi()
            .transaction(&mut [
                Operation::Write(&[register::MOTION_BURST]),
                Operation::Read(&mut buffer),
            ])
            .await?;
        let motion_raw: MotionRaw = *bytemuck::from_bytes(&buffer);

        if (0 != motion_raw.dr & 0b1000_0000)
            && !((motion_raw.quality < 0x19) && (motion_raw.shutter_upper == 0x1F))
        {
            Ok(MotionDelta {
                x: motion_raw.x,
                y: motion_raw.y,
            })
        } else {
            debug!("Motion data failed validation: {}", motion_raw);
            Err(SensorError::InvalidMotion)
        }
    }

    /// Capture a full frame from the sensor.
    ///
    /// Warning: this is very slow.
    pub async fn capture_frame(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<[u8; FRAME_SIZE], SensorError> {
        debug!("Capturing frame...");
        self.write_bulk(&[
            (0x7F, 0x07),
            (0x4C, 0x00),
            (0x7F, 0x08),
            (0x6A, 0x38),
            (0x7F, 0x00),
            (0x55, 0x04),
            (0x40, 0x80),
            (0x4D, 0x11),
        ])
        .await?;

        delay_source.delay_ms(10).await;

        self.write_bulk(&[(0x7F, 0x00), (0x58, 0xFF)]).await?;

        // I am slightly suspicious that we're checking against two bits here but this follows the logic
        // of the Pimoroni driver so we've just got to assume it's correct
        while self.read(register::RAW_DATA_GRAB_STATUS).await? & 0b1100_0000 == 0 {}

        self.write(register::RAW_DATA_GRAB, 0x00).await?;

        let mut buffer = [0; FRAME_SIZE];
        let mut index = 0;

        while index < FRAME_SIZE {
            let value = self.read(register::RAW_DATA_GRAB).await?;
            match value & 0b1100_0000 {
                0b0100_0000 => {
                    buffer[index] &= 0b0000_0011;
                    buffer[index] |= value << 2; // The Python driver masks the 2 most significant bits before shifting, but I don't think that's necessary in a strongly typed language
                }
                0b1000_0000 => {
                    buffer[index] &= 0b1111_1100;
                    buffer[index] |= (value & 0b0000_1100) >> 2;
                    index += 1;
                }
                _ => (),
            }
        }

        Ok(buffer)
    }

    /// Resets and restarts the sensor.
    ///
    /// It appears that the sensor shuts itself down if no change has been detected for an extended period.
    /// Use this function to wake it up again.
    pub async fn wake(&mut self, delay_source: &mut impl DelayNs) -> Result<(), SensorError> {
        debug!("Waking the sensor");
        self.init(delay_source).await
    }
}

/// Enumeration of possible errors encountered by the sensor driver
#[derive(Debug, Clone, PartialEq, Format)]
pub enum SensorError {
    /// An error occurred during SPI comms.
    Spi(embedded_hal_async::spi::ErrorKind),
    /// ID or revision number retrieved from the sensor was invalid.
    InvalidId(Id),
    /// Motion data retrieved from sensor did not pass validation.
    InvalidMotion,
}

impl<T: embedded_hal_async::spi::Error> From<T> for SensorError {
    fn from(value: T) -> Self {
        Self::Spi(value.kind())
    }
}

/// Defines product identification information
#[derive(Debug, Clone, PartialEq, Format)]
pub struct Id {
    /// Product ID number retrieved from the PRODUCT_ID register
    pub product_id: u8,
    /// Product revision number retrieved from the PRODUCT_REVISION register
    pub revision: u8,
}

/// Enumeration of valid rotation settings
pub enum RotationDegrees {
    _0 = 0,
    _90 = 90,
    _180 = 180,
    _270 = 270,
}

/// Defines the structure of motion data
pub struct MotionDelta {
    /// Motion in the x direction
    pub x: i16,
    /// Motion in the y direction
    pub y: i16,
}

#[repr(C)]
#[derive(Pod, Clone, Copy, Zeroable, Debug, PartialEq)]
struct MotionRaw {
    dr: u8,
    obs: u8,
    x: i16,
    y: i16,
    quality: u8,
    raw_sum: u8,
    raw_max: u8,
    raw_min: u8,
    shutter_upper: u8,
    shutter_lower: u8,
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

        // The default settings do not conform to any orientation, so we set rotation to 0 here to provide a known default.
        self.set_rotation(RotationDegrees::_0).await?;
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

    /// Here be dragons...
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
mod register {
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
