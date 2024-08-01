#![doc = include_str!("../README.md")]
#![warn(unsafe_code)]
#![no_std]

mod accelerometer;
mod gyroscope;

use accelerometer::AccelScaleRange;
use gyroscope::GyroScaleRange;

use embedded_hal::i2c::{Error as I2cError, ErrorKind as I2cErrorKind, I2c};

const MPU6886_ADDR: u8 = 0x68;

/// MPU6886 error type.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// An I2C error occurred during the transaction.
    I2cError(I2cErrorKind),
    /// Unknown chip detect.
    UnknownChip(u8),
    /// Other error. The original error converted from may contain more information.
    Other,
}

impl<T: I2cError> From<T> for Error {
    fn from(value: T) -> Self {
        Self::I2cError(value.kind())
    }
}

impl embedded_hal::digital::Error for Error {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}

#[derive(Debug)]
pub struct Mpu6886<I2C> {
    i2c: I2C,
    acc_range: AccelScaleRange,
    gyro_range: GyroScaleRange,
}

impl<I2C: I2c> Mpu6886<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c: i2c,
            acc_range: AccelScaleRange::Range2g,
            gyro_range: GyroScaleRange::Range250Dps,
        }
    }

    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Checks chip version and load current state.
    pub fn init(&mut self) -> Result<(), Error> {
        let chip_id = self.read_u8(0x75)?;
        if chip_id != 0x19 {
            Err(Error::UnknownChip(chip_id))
        } else {
            self.acc_range = self.get_accel_scale_range()?;
            self.gyro_range = self.get_gyro_scale_range()?;
            Ok(())
        }
    }

    /// Resets the sensor to initial state.
    pub fn reset(&mut self) -> Result<(), Error> {
        self.write_u8(0x6B, 0b10000000)?;
        // also reset internal state
        self.acc_range = AccelScaleRange::Range2g;
        self.gyro_range = GyroScaleRange::Range250Dps;
        Ok(())
    }

    pub fn sleep(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6B)?;
        let new_value = original_value | 0b01000000;
        self.write_u8(0x6B, new_value)
    }

    /// Wakes the inertial sensor up.
    ///
    /// The sensor is in sleep mode by default. For lazy people who don't
    /// want to check sensor's version, this method also loads current range
    /// states from chip so the values are calculated correctly.
    pub fn wake(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6B)?;
        let new_value = original_value & 0b10111111;
        self.write_u8(0x6B, new_value)?;
        // also load state from chip
        self.acc_range = self.get_accel_scale_range()?;
        self.gyro_range = self.get_gyro_scale_range()?;
        Ok(())
    }

    pub fn use_best_clock(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6B)?;
        let new_value = original_value & 0b11111000 | 0b00000001;
        self.write_u8(0x6B, new_value)
    }

    pub fn use_internal_clock(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6B)?;
        let new_value = original_value & 0b11111000;
        self.write_u8(0x6B, new_value)
    }

    pub fn disable_temperature_sensor(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6B)?;
        let new_value = original_value | 0b00001000;
        self.write_u8(0x6B, new_value)
    }

    pub fn enable_temperature_sensor(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6B)?;
        let new_value = original_value & 0b11110111;
        self.write_u8(0x6B, new_value)
    }

    pub fn temperature(&mut self) -> Result<f32, Error> {
        let raw_value = self.read_u16(0x41)?;
        let temperature: f32 = raw_value as i16 as f32 / 326.8 + 25.0;
        Ok(temperature)
    }

    /// Reads one u8 integer.
    fn read_u8(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf: [u8; 1] = [0; 1];

        match self.i2c.write_read(MPU6886_ADDR, &[reg], &mut buf) {
            Ok(_) => Ok(buf[0]),
            Err(e) => Err(e.into()),
        }
    }

    fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), Error> {
        Ok(self.i2c.write(MPU6886_ADDR, &[reg, value])?)
    }

    fn read_u16(&mut self, reg: u8) -> Result<u16, Error> {
        let mut buf: [u8; 2] = [0; 2];
        self.read_buf(reg, &mut buf)?;
        let value: u16 = ((buf[0] as u16) << 8) + (buf[1] as u16);
        Ok(value)
    }

    #[inline]
    fn read_buf(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        Ok(self.i2c.write_read(MPU6886_ADDR, &[reg], buf)?)
    }
}
