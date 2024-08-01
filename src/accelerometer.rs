//! Accelerometer interface implementation.

use crate::{Error, I2c, Mpu6886};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccelScaleRange {
    Range2g,
    Range4g,
    Range8g,
    Range16g,
}

// #[repr(u8)]
// #[derive(Debug, Clone, Copy, PartialEq)]
// pub enum AccelRate {

// }

impl<I2C: I2c> Mpu6886<I2C> {
    pub fn get_accel_scale_range(&mut self) -> Result<AccelScaleRange, Error> {
        let raw_value = self.read_u8(0x1C)?;
        let selection = (raw_value & 0b00011000) >> 3;
        match selection {
            0 => Ok(AccelScaleRange::Range2g),
            1 => Ok(AccelScaleRange::Range4g),
            2 => Ok(AccelScaleRange::Range8g),
            _ => Ok(AccelScaleRange::Range16g),
        }
    }

    pub fn set_accel_scale_range(&mut self, value: AccelScaleRange) -> Result<(), Error> {
        let original_value = self.read_u8(0x1C)?;
        let choice_value = value as u8;
        let reg_value = (original_value & 0b11100111) | choice_value << 3;
        self.write_u8(0x1C, reg_value)?;
        self.acc_range = value;
        Ok(())
    }

    pub fn turn_off_accelerometer(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6C)?;
        let new_value = original_value | 0b00111000;
        self.write_u8(0x6C, new_value)
    }

    pub fn turn_on_accelerometer(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6C)?;
        let new_value = original_value & 0b11000111;
        self.write_u8(0x6C, new_value)
    }

    /// Returns measured acceleration, (X, Y, Z), in g.
    pub fn acceleration(&mut self) -> Result<(f32, f32, f32), Error> {
        let mut xyz_buf: [u8; 6] = [0; 6];
        self.read_buf(0x3B, &mut xyz_buf)?;
        let x_raw = (xyz_buf[0] as u16) << 8 | (xyz_buf[1] as u16);
        let y_raw = (xyz_buf[2] as u16) << 8 | (xyz_buf[3] as u16);
        let z_raw = (xyz_buf[4] as u16) << 8 | (xyz_buf[5] as u16);
        const GRAVITY: f32 = 9.80665;
        let factor: f32 = match self.acc_range {
            AccelScaleRange::Range2g => 16384.0,
            AccelScaleRange::Range4g => 8192.0,
            AccelScaleRange::Range8g => 4096.0,
            AccelScaleRange::Range16g => 2048.0,
        };
        let x_real = (x_raw as f32) / factor * GRAVITY;
        let y_real = (y_raw as f32) / factor * GRAVITY;
        let z_real = (z_raw as f32) / factor * GRAVITY;
        Ok((x_real, y_real, z_real))
    }

    pub fn acceleration_raw(&mut self) -> Result<(u16, u16, u16), Error> {
        let mut xyz_buf: [u8; 6] = [0; 6];
        self.read_buf(0x3B, &mut xyz_buf)?;
        let x_raw = (xyz_buf[0] as u16) << 8 | (xyz_buf[1] as u16);
        let y_raw = (xyz_buf[2] as u16) << 8 | (xyz_buf[3] as u16);
        let z_raw = (xyz_buf[4] as u16) << 8 | (xyz_buf[5] as u16);
        Ok((x_raw, y_raw, z_raw))
    }
}
