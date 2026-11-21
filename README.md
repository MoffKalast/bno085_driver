# bno085_driver

One rospy node speaking SHTP over UART to a BNO085. Publishes `sensor_msgs/Imu` on `/imu/data`, runs magnetometer calibration on demand, and reports health on `/imu/diagnostics`.

The carrier board needs to have the soldering pad for SHTP bridged (PS1 high PS0 low) the host TX line reaching the BNO RX pin. The USB-Serial chip must be able to handle 3M baud, most don't.

## Udev rules

Create `/etc/udev/rules.d/99-bno.rules` with something that matches your usb-to-serial chip:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A50285BI", SYMLINK+="ttyBNO"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Launch

```
roslaunch bno085_driver bno085.launch
```

## Calibration

```
rostopic pub -1 /imu/calibrate std_msgs/Bool "data: true"
```

All dynamic calibration (accelerometer, gyroscope and magnetometer) is off during normal operation, so the stored calibration is used as-is and never drifts. It is only enabled for all three during this routine. The BNO's periodic background save is disabled too, so the calibration in flash only changes when you explicitly save it here.

First rest the sensor still for a couple of seconds on each of its six faces (accelerometer), then move it slowly through all orientations (magnetometer), in the area where the robot will operate. Watch the status under `/imu/diagnostics`: `accel_cal_status`, `gyro_cal_status`, `mag_cal_status` and `rv_status` all need to reach 2 before a save is allowed. Since the magnetic environment differs between locations, re-run the calibration whenever the robot moves to a new area.

```
rostopic pub -1 /imu/calibrate std_msgs/Bool "data: false"   # stop and save
```

On the falling edge the node issues Save DCD, waits for the command response, then soft-resets and restarts the stream with calibration off. The calibration is saved and autoloaded on the BNO itself. `calibration_enabled` in `/imu/diagnostics` shows the enable flags the sensor itself reported back.

If the fit did not converge the save is skipped and logged as a warning; set `~force_save` to save anyway.

## Frames

Two fixed rotations, both RPY in radians, both identity by default:

- `world_rotation` — BNO fusion reference frame to ENU.
- `mount_rotation` — your body frame to the sensor frame, i.e. how the chip sits on the robot.

Applied as `q_pub = world_rotation * q_sensor * mount_rotation`, with gyro and accel vectors rotated by the inverse of `mount_rotation` only.

## Timestamps

With `use_sensor_time: true` (default) the stamp comes from the sensor's own clock rather than the arrival time. 

Each report carries a delay field (100 µs units) relative to the batch's base timestamp report, so the actual sample instant is known in the BNO's timebase. The node estimates the offset between that timebase and ROS time by taking the minimum of `host_arrival - sensor_time` over a sliding window (`clock_window`), since queueing delay is always positive and the minimum is therefore the least-delayed sample.

If the spread of offsets across the window exceeds `clock_tolerance`, the sensor clock is treated as untrustworthy and the node silently falls back to host arrival time; `/imu/diagnostics` reports which one is in use as `sensor_clock`. That guard exists because the interpretation of the base timestamp delta is worth verifying against your actual hardware — if `sensor_clock` reads `host time` permanently, the sensor clock reconstruction is off and the fallback is doing its job.