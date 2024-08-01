# MPU6886 driver

This is a simple MPU6886 driver with embedded-hal 1.0 support.

What's implemented:

- set/get gyro rate
- read gyro
- set/get accelerometer rate
- read accelerometer
- read temperature
- wake
- sleep
- enable/disable accel/gyro/temperature
- use internal 20MHz clock or best clock

MPU6886 also supports i2c address 0x69, but this is not implemented in the driver, for simplicity.

## Example

minimum:

```rust
use mpu6886::Mpu6886;

let mut sensor = Mpu6886::new(i2c);
let _ = sensor.wake();
gyro = sensor.gyro().unwrap();
acc = sensor.acceleration().unwrap();
```

proper:

```rust
use mpu6886::{Mpu6886, Error};

let mut sensor = Mpu6886::new(i2c);
match sensor.init() {
    Ok(_) => {},
    Err(e) => match e {
        Error::UnknownChip(chip_id) => panic!("Unknown chip found, ID: {}", chip_id),
        _ => panic!("Communication error!"),
    },
};
let _ = sensor.wake();
gyro = sensor.gyro().unwrap();
acc = sensor.acceleration().unwrap();
```
