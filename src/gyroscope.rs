//! Gyroscope implementation.

use crate::{Error, Mpu6866, I2c};
use core::f32::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroScaleRange {
    Range250Dps,
    Range500Dps,
    Range1000Dps,
    Range2000Dps,
}

impl<I2C: I2c, const MPU6886_ADDR: u8> Mpu6866<I2C, MPU6886_ADDR> {
    pub fn get_gyro_scale_range(&mut self) -> Result<GyroScaleRange, Error> {
        let raw_value = self.read_u8(0x1B)?;
        let selection = (raw_value & 0b00011000) >> 3;
        match selection {
            0 => Ok(GyroScaleRange::Range250Dps),
            1 => Ok(GyroScaleRange::Range500Dps),
            2 => Ok(GyroScaleRange::Range1000Dps),
            _ => Ok(GyroScaleRange::Range2000Dps),
        }
    }

    pub fn set_gyro_scale_range(&mut self, value: GyroScaleRange) -> Result<(), Error> {
        let original_value = self.read_u8(0x1B)?;
        let choice_value = match value {
            GyroScaleRange::Range250Dps => 0,
            GyroScaleRange::Range500Dps => 1,
            GyroScaleRange::Range1000Dps => 2,
            GyroScaleRange::Range2000Dps => 3,
        };
        let reg_value = (original_value & 0b11100111) | choice_value;
        self.write_u8(0x1B, reg_value)?;
        self.gyro_range = value;
        Ok(())
    }

    pub fn gyro_standby(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6B)?;
        let new_value = original_value | 0b00010000;
        self.write_u8(0x6B, new_value)
    }

    pub fn gyro_active(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6B)?;
        let new_value = original_value & 0b11101111;
        self.write_u8(0x6B, new_value)
    }

    pub fn turn_off_gyro(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6C)?;
        let new_value = original_value | 0b00000111;
        self.write_u8(0x6C, new_value)
    }

    pub fn turn_on_gyro(&mut self) -> Result<(), Error> {
        let original_value = self.read_u8(0x6C)?;
        let new_value = original_value & 0b11111000;
        self.write_u8(0x6C, new_value)
    }

    /// Returns measured angular acceleration, (X, Y, Z), in rad/s
    pub fn gyro(&mut self) -> Result<(f32, f32, f32), Error> {
        const SENSITIVITY: f32 = 131.0; // degree per second
        let mut xyz_buf: [u8; 6] = [0; 6];
        self.read_buf(0x43, &mut xyz_buf)?;
        let x_raw = (xyz_buf[0] as u16) << 8 | (xyz_buf[1] as u16);
        let y_raw = (xyz_buf[2] as u16) << 8 | (xyz_buf[3] as u16);
        let z_raw = (xyz_buf[4] as u16) << 8 | (xyz_buf[5] as u16);
        let factor = match self.gyro_range {
            GyroScaleRange::Range250Dps => SENSITIVITY,
            GyroScaleRange::Range500Dps => SENSITIVITY / 2.0,
            GyroScaleRange::Range1000Dps => SENSITIVITY / 4.0,
            GyroScaleRange::Range2000Dps => SENSITIVITY / 8.0,
        };
        let x_real = (x_raw as f32) / factor * PI / 180.0;
        let y_real = (y_raw as f32) / factor * PI / 180.0;
        let z_real = (z_raw as f32) / factor * PI / 180.0;
        Ok((x_real,y_real,z_real))
    }
}
