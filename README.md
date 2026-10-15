# STM32 MPU6050: Raw Data, Tilt Calculation and Complementary Filter

Three step-by-step learning projects for the **MPU6050** (GY-521) accelerometer and gyroscope on the **STM32 NUCLEO-F446RE**, written with STM32CubeIDE and the HAL library. Each step builds on the previous one, and the results are printed over UART.

```
MPU6050 --I2C--> STM32 --UART--> PC terminal
```

## Branches

| Branch | Step | What it does |
|---|---|---|
| `main` | 1. Raw data | Reads accelerometer and gyroscope values and prints them over UART |
| `tilt-calculation` | 2. Tilt calculation | Computes Roll and Pitch from the accelerometer |
| `complementary-filter` | 3. Complementary filter | Fuses accelerometer and gyroscope into a stable angle |

Switch between steps with:

```bash
git checkout tilt-calculation
git checkout complementary-filter
```

## Hardware

- NUCLEO-F446RE
- MPU6050 module (GY-521)
- 4 jumper wires

| MPU6050 | NUCLEO-F446RE |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SCL | PB6 (I2C1_SCL) |
| SDA | PB7 (I2C1_SDA) |

Leave AD0 unconnected or tied to GND (I2C address `0x68`). INT, XDA and XCL stay unconnected. The GY-521 board already has pull-up resistors on the I2C lines.

## CubeMX / .ioc settings

All three steps use the same configuration:

- **SYS:** Debug = Serial Wire
- **I2C1:** I2C mode, PB6 / PB7, Standard Mode (100 kHz)
- **USART2:** Asynchronous, 115200 baud, 8 data bits, no parity, 1 stop bit (virtual COM port through ST-Link)

## Step 1: Raw data (`main`)

- Resets and wakes up the sensor, sets the accelerometer range to ±2 g
- Reads `WHO_AM_I` (expected `0x68`)
- Reads accelerometer, temperature and gyroscope registers in a single 14-byte burst
- Prints the raw 16-bit values over UART

Example output:

```
WHO=0x68 | AX=-328 AY=-3364 AZ=16384 | GX=-388 GY=82 GZ=3
```

With the board lying flat, the Z axis should read about 16384 (1 g at ±2 g range) and X and Y should be close to zero.

## Step 2: Tilt calculation (`tilt-calculation`)

Roll and Pitch are calculated from the accelerometer only, using the direction of gravity:

```
roll  = atan2(ay, az)
pitch = atan2(-ax, sqrt(ay^2 + az^2))
```

Angles are printed in degrees with one decimal place. Integer formatting is used, so no `printf` float support is needed.

Example output:

```
Roll : -0.5 deg | Pitch : +2.6 deg
```

Notes:

- The accelerometer measures all acceleration, so shaking the board makes the angles jump.
- A small constant offset (a few degrees) with the board lying flat is normal sensor offset.
- The code uses `atan2f` and `sqrtf`. If the linker reports an undefined reference, add `m` under Project > Properties > C/C++ Build > Settings > MCU GCC Linker > Libraries.
- A `Z_BROKEN` switch at the top of the file is a workaround for a module whose Z-axis accelerometer reads a stuck value (-32768). When it is set to `1`, Z is estimated from X and Y assuming the board faces up and the total acceleration is 1 g. Set it to `0` to use the real Z reading.

## Step 3: Complementary filter (`complementary-filter`)

The gyroscope is accurate over short periods but drifts over time. The accelerometer does not drift but is noisy during motion. The filter combines both:

```
angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
```

- `alpha = 0.98` (98% gyroscope, 2% accelerometer)
- Gyroscope offset is calibrated at startup from 200 samples, so keep the board still for about one second after reset
- The sensor's built-in low-pass filter is enabled (about 44 Hz)
- Gyro range ±250 deg/s, accelerometer range ±2 g
- The loop runs about every 10 ms and results are printed about every 100 ms

Example output (`filtre` means filtered):

```
Roll(acc): -27.8 | Roll(filtre): -23.5 | Pitch(filtre): +2.4
```

Printing the accelerometer-only roll next to the filtered roll makes the effect easy to see: the raw value jumps when the board moves quickly or is held at a steep angle, while the filtered value stays smooth. The trade-off is a short lag when the board moves. Lowering `alpha` (for example to 0.95) makes the filtered angle follow the accelerometer faster at the cost of more noise.

## How to run

1. Open the project in STM32CubeIDE and build it.
2. Flash the NUCLEO-F446RE.
3. Open a serial terminal on the ST-Link COM port at 115200 baud, 8N1.
4. Press the black RESET button to see the startup messages, then move the board and watch the values.

## Known limitations

- Roll and Pitch only. Yaw cannot be estimated without a magnetometer.
- Accelerometer offsets are not calibrated, so a few degrees of error at rest is expected.
- Only the gyroscope offset is calibrated at startup.
