# wled-imu_MPU6050

WLED usermod driver for the MPU-6050 six-axis IMU (accelerometer + gyroscope).

Provides IMU data to the [wled-motion_reactive](https://github.com/willmmiles/wled-motion_reactive) usermod, which implements motion-reactive LED effects for staffs, wands, and other moving props.

The MPU-6050 has a built-in Digital Motion Processor (DMP) that fuses the gyro and
accelerometer readings on-chip, providing a gravity vector and gravity-free
("linear") acceleration directly -- no software filter is needed.

Original implementation by Jonathan Diamond <feros32@gmail.com>; this descendent maintained by Will Miles <will@willmiles.net>

_Jonathan's Original Story:_

As a memento to a long trip I was on, I built an icosahedron globe. I put lights inside to indicate cities I travelled to.

I wanted to integrate an IMU to allow either on-board, or off-board effects that would
react to the globes orientation. See the blog post on building it <https://www.robopenguins.com/icosahedron-travel-globe/> or a video demo <https://youtu.be/zYjybxHBsHM> .

## Wiring

The connections needed to the MPU6050 are as follows:
```
  VCC     VU (5V USB)   Not available on all boards so use 3.3V if needed.
  GND     G             Ground
  SCL     D1 (GPIO05)   I2C clock
  SDA     D2 (GPIO04)   I2C data
  XDA     not connected
  XCL     not connected
  AD0     not connected
  INT     D8 (GPIO15)   Interrupt pin (optional, see below)
```

Configure the I²C pins in WLED's LED Settings → Hardware page.

## Configuration

All settings are in WLED's Usermod Settings page under **MPU6050_IMU**:

| Setting | Default | Description |
|---|---|---|
| enabled | false | Enable the driver |
| interrupt_pin | -1 | GPIO wired to the MPU6050 INT pin; -1 polls the FIFO instead of using an interrupt |
| update_interval_ms | 20 | FIFO poll interval when `interrupt_pin` is -1 (ignored in interrupt mode) |
| x/y/z_gyro_bias | 0 | Gyro offset calibration (raw sensor units) |
| x/y/z_acc_bias | 0 | Accelerometer offset calibration (raw sensor units) |

An interrupt pin is not required -- the driver falls back to polling the FIFO
count at `update_interval_ms` when `interrupt_pin` is -1, rather than checking
on every WLED main-loop iteration.

## Building

Add both this driver and `wled-motion_reactive` to your `custom_usermods` in `platformio_override.ini`:

```ini
[env:myboard]
extends = env:esp32dev
custom_usermods =
  ${env:esp32dev.custom_usermods}
  file:///path/to/wled-motion_reactive
  file:///path/to/wled-imu_MPU6050
```

Or reference published repos directly by URL once available.
