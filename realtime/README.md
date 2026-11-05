# Real-time controller block `rb_vdc_imu.m`

One MATLAB Function block for Simulink (sample time 1 ms) that contains the complete control system of the article:
pairwise inertial joint observer and gyro-based slew estimate, automatic tuning of the four valve channels,
closed-loop check, velocity-level VDC with delay-matched reference and inverse valve map, online adaptation, strike-cycle planner
and mode logic. The code uses fixed-size data and persistent states only and is compatible with Simulink Coder.

## Interface
```
[u, q_ref, qd_ref, info, tune, vdc, imu] = rb_vdc_imu(q_meas, enable, acc1, gyr1, acc2, gyr2, acc3, gyr3)
```
| signal | size | meaning |
|---|---|---|
| `q_meas` | 4 | joint encoder angles [deg]: slew, lift, tilt, crowd (not used in mode 2) |
| `enable` | 1 | 0 stop (valves closed at once, run paused), 1 run with the encoders (observer in shadow mode), 2 run with the IMU observer |
| `acc1..3`, `gyr1..3` | 3 each | accelerometers [m/s^2] and gyros of IMU 1 (lift link), 2 (dipper), 3 (tool link) |
| `u` | 4 | valve commands of channels 1-4 (normalized, limited to +-0.8) |
| `q_ref`, `qd_ref` | 4 each | reference joint angles [deg] and rates [deg/s] |
| `info` | 10 | phase (0 wait, 1 tuning, 2 check, 3 strike cycles, 4 done, 5 no valve moved a joint), step, valve/joint/boulder, round, speed factor, segment time, wait time, largest error, update interval, warning bits |
| `tune` | 40 | per joint: valve channel, direction, gains K+ and K-, dead zones d+ and d-, loop delay, position gain, adaptation factors of K+ and K- |
| `vdc` | 14 | required joint rates, control errors, required piston rates, required cylinder forces |
| `imu` | 14 | observer: slew, lift, tilt, crowd angles, their rates, confidence weights, status, mode, rest time |

Selecting the other mode, or enabling again after a completed run, starts a new run with automatic tuning. The value
0 acts at once; 1 and 2 are accepted after 20 ms. In mode 2, the run starts after about 4 s of rest, during which the
slew gyro bias is estimated.

## Configuration
The design parameters are set in the function `rb_cfg`: speeds and approach heights of the task; settings of the
automatic tuning; controller limits and adaptation rates; and the gains, filters and fault handling of the IMU
observer.

The machine-specific values are read by `rb_cfg` from the function `rb_params`, which is not part of this repository:
link geometry and masses; the IMU installation and calibration (axis mappings `ob_C1..ob_C3`, gyro input unit
`ob_gunit`, lever arms, mounting angles, biases, gyro scale and cross-axis terms); and the home pose, joint limits and
strike points of the task. Copy `rb_params_template.m` to `rb_params.m` in this folder, enter the values of the
machine and delete the error line of the template; `rb_params.m` is listed in `.gitignore`. The template documents
the conventions and units of every value.

The calibration must be identified for each IMU installation from a log with slewing; the axis mappings and the gyro
unit should be verified in mode 1 (shadow mode) by comparing the `imu` outputs with the encoders before mode 2 is
used.

## Deployment used in the experiments
The block was placed in a Simulink model with the EtherCAT inputs and outputs of the machine, compiled with Simulink
Coder and the TwinCAT 3 target for MATLAB/Simulink, and executed as a TwinCAT 3 module in a 1-ms task on a Beckhoff
industrial PC. All outputs were logged at 1 kHz.
