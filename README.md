# BNO_Localizer
(The Readme was written by AI - Claude)

IMU-based localisation in ROS 2 using an error-state Kalman filter (ESKF).
A Raspberry Pi Pico reads a BNO085 IMU over I2C using the SHTP protocol and streams high-frequency
gyro, linear acceleration and rotation-vector data over USB serial. ROS 2 nodes publish this data,
calibrate the noise parameters, and run an ESKF that estimates position, velocity, orientation and
sensor biases. A zero-velocity update (ZUPT) driven by a SHOE detector limits drift while the sensor is at rest
(designed with foot-mounted use in mind).

> **Status:** work in progress. Hardware-tested on a BNO085 + Pico; quantitative results are
> still to be added (see [Results](#results)).

## How it works

```
BNO085 --I2C/SHTP--> Raspberry Pi Pico --USB serial--> ROS 2 driver node --topics--> ESKF localiser
 (gyro, lin. accel,    (pico_build/*.c)                  (usb_test_node.py)           (eskf_localiser.py)
  rotation vector)
```

**Sensor data**

| Topic | Content | Approx. rate |
|---|---|---|
| `/imu1/data/gyro` | angular velocity | ~200 Hz |
| `/imu1/data/lin_acc` | linear acceleration | ~100 Hz |
| `/imu1/data/rot_vec` | orientation (quaternion), acts as a drift-limiting reference | ~50 Hz |

The BNO085 can output gyro data at up to 1000 Hz. The I2C link is the bottleneck, and SPI would
remove it (see [`BNO085.md`](BNO085.md) for design notes).

**Filter.** [`eskf.py`](src/localiser/localiser/eskf.py) implements a 16-state nominal vector
(position, velocity, quaternion, accelerometer bias, gyro bias) with a 15-dimensional error state.
It provides:

- `predict`: propagates the state from IMU measurements
- `update_quat`: corrects orientation using the BNO085 rotation vector
- `update_zupt`: zero-velocity pseudo-measurement during stance
- `update_zero_pos`: position constraint

**Zero-velocity detection.** [`eskf_localiser.py`](src/localiser/localiser/eskf_localiser.py) computes a
SHOE (Stance Hypothesis Optimal dEtection) statistic over a sliding window of gyro and accelerometer
samples and applies a ZUPT when it falls below a calibrated threshold. The approach follows
Wagstaff et al., *Robust Data-Driven Zero-Velocity Detection for Foot-Mounted Inertial Navigation*
([arXiv:1910.00529](https://arxiv.org/abs/1910.00529), included in this repo).

**Calibration.** [`eskf_calibrator.py`](src/localiser/localiser/eskf_calibrator.py) records a still period and a
walking period, then estimates and writes to `config/calibration.json`:

- `sigma_a`, `sigma_omega`: noise floors at rest
- `zupt_gamma`: SHOE threshold, taken from the valley between the stance and motion peaks of the statistic
- `zupt_var`: ZUPT measurement noise
- accelerometer noise and bias random-walk parameters

## Repository layout

```
src/localiser/           ROS 2 (ament_python) package
  localiser/
    eskf.py              error-state Kalman filter
    eskf_localiser.py    ROS node: ESKF + SHOE/ZUPT
    eskf_calibrator.py   still/walk calibration of noise parameters and threshold
    usb_test_node.py     serial -> ROS driver (publishes /imu1/data/*)
    threshold_calculator.py   standalone SHOE threshold tool
  config/calibration.json
pico_build/              Pico firmware (C): I2C scanner, SHTP readers, IMU streaming
BNO085.md                design notes (data requirements, calibration, spike/bit-flip debugging)
test_topics.sh           prints the measured publish rate of each IMU topic
1910.00529v4.pdf         reference paper for the SHOE/ZUPT approach
```

## Requirements

- ROS 2 (`rclpy`, `geometry_msgs`) with `colcon`
- Python 3: `numpy`, `scipy`, `matplotlib`, `pyserial`
- Hardware: Raspberry Pi Pico + BNO085 (I2C address `0x4A`, SDA on GP2, SCL on GP3 in the current firmware)
- Pico SDK and CMake, to build the firmware

## Build and run

**1. Flash the Pico.** Set `PICO_SDK_PATH`, then build the firmware and copy the `.uf2` to the board in BOOTSEL mode:

```bash
cd pico_build
mkdir build && cd build
cmake .. && make pico_imu     # or imu / shtp_2 / imu_flipper_test, see CMakeLists.txt
```

**2. Build the ROS 2 package.**

```bash
colcon build --packages-select localiser
source install/setup.bash
```

**3. Start the driver** (expects the Pico at `/dev/ttyACM0`, 115200 baud) and check the rates:

```bash
ros2 run localiser usb_test_node
./test_topics.sh
```

**4. Calibrate** (hold the sensor still, then walk when prompted):

```bash
ros2 run localiser eskf_calibrator
```

**5. Run the localiser:**

```bash
ros2 run localiser eskf_localiser
```

The first few seconds (default 10 s) are spent in a static initialisation phase, so keep the sensor still.

**Outputs**

| Topic | Content |
|---|---|
| `/localisation/pose/position` | estimated position |
| `/localisation/pose/velocity` | estimated velocity |
| `/localisation/pose/orientation` | estimated orientation (quaternion) |
| `/localisation/pose/accel_bias`, `/gyro_bias` | estimated sensor biases |
| `/eskf/zupt_velocity` | velocity residual at ZUPT updates |

**Parameters** for `eskf_localiser` include `config_path`, `alpha`, `R_ori_init`, `shoe_window`,
`static_init_seconds` and `zupt_scale`. Override them with `--ros-args -p shoe_window:=40`, for example.

## Results

*To be added:* drift over a fixed-length walk with and without ZUPT, and a plot of the SHOE statistic
showing the stance/motion threshold (`shoe_threshold.png` is a first example).

## Known issues and notes

- **Config path.** `DEFAULT_CONFIG_PATH` in the nodes is hardcoded to a local path. Pass `config_path:=...`
  or edit the constant until this is made relative.
- **Bit flips on I2C reads.** Some apparent spikes in the data turned out to be bit flips from a race condition when
  reading without the interrupt line. The planned fix is to read only on the BNO085 INT signal (details in
  [`BNO085.md`](BNO085.md)).
- **Firmware targets.** `CMakeLists.txt` also lists an `imu_lib` target that expects an `sh2/` source directory
  that is not part of this repo.
- `files.zip` holds an earlier standalone ESKF implementation with its own validation script.

## References

- B. Wagstaff, V. Peretroukhin, J. Kelly, *Robust Data-Driven Zero-Velocity Detection for Foot-Mounted Inertial Navigation*, IEEE Sensors Journal, 2020. [arXiv:1910.00529](https://arxiv.org/abs/1910.00529)
- Bosch / CEVA BNO085 datasheet and SH-2 / SHTP reference manual

## License

Apache-2.0 (see [`src/localiser/LICENSE`](src/localiser/LICENSE)).
