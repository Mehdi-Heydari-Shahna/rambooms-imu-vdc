# IMU-Based and Auto-Tuned Virtual Decomposition Control for a Hydraulic Rock-Breaker Boom

**Mehdi Heydari Shahna, Pauli Mustalahti, Jouni Mattila**
Faculty of Engineering and Natural Sciences, Tampere University, Finland

Real-time controller and stability proofs for the article of the same title.

## Overview

Joint encoders of rock-breaker booms sit at exposed pins, where impacts and falling rock damage them, whereas inertial
measurement units (IMUs) can be mounted on protected link surfaces. This work closes the motion-control loop of a
10-t hydraulic rock-breaker boom (Rambooms X88-540-R) on industrial MEMS IMUs (Murata SCHA63T-K03) after a one-time
calibration.

- **Pairwise inertial joint observer.** Building on the pairwise complementary filter of Vihonen *et al.*, each joint
  observer transports the specific forces of the two adjacent IMUs to the joint point with each link's own gyro rate,
  so that the joint-angle measurement depends on no estimate.
- **Stability guarantee.** A common Lyapunov function proves the confidence-weighted proportional–integral observer
  input-to-state stable for arbitrary weight trajectories above a floor. Because control errors do not enter the
  observer, observers and virtual decomposition control (VDC) form a cascade: the boom-joint loops are proved
  regionally input-to-state stable with the pressure-compensated valves modeled as velocity sources with delays, gain
  mismatch and dead zones, including online tuning.
- **Automatic commissioning.** An auto-tuning procedure identifies each valve's joint assignment, flow gains, dead
  zones and loop delay through the feedback in use and sets the position gains to satisfy the stability condition.

## Main results

- The observer deviated from the joint encoders by 0.07–0.11° RMS, both in shadow mode and with its estimates fed back.
- With IMU-only feedback, the boom joints tracked a strike-cycle sequence within 0.12–0.21° RMS.
- Hammer impacts were studied in a MuJoCo digital twin of the boom (not included in this repository); the breaker was
  not fired on the machine.

## Repository structure

| folder | content |
|---|---|
| `realtime/` | `rb_vdc_imu.m`: the Simulink MATLAB Function block that ran on the machine (IMU observer, automatic tuning, velocity-level VDC, strike-cycle planner, mode logic); `rb_params_template.m`: template of the machine-specific parameters; see [`realtime/README.md`](realtime/README.md) |
| `docs/` | [`Supplementary_Material_Proofs.pdf`](docs/Supplementary_Material_Proofs.pdf): proofs of the stability results of the article |

## Machine-specific parameters

The link geometry and mass properties of the boom, the IMU installation and calibration, and the home pose, joint
limits and strike points of the experiments are not distributed with this repository. The controller reads them from
`realtime/rb_params.m`, which is created from `realtime/rb_params_template.m` and is listed in `.gitignore`.

## Requirements

MATLAB/Simulink with Simulink Coder. In the experiments, the block was compiled with the TwinCAT 3 target for
MATLAB/Simulink and executed as a TwinCAT 3 module in a 1-ms task on a Beckhoff industrial PC.

## Notes

- The IMU calibration must be identified for each installation from a log with slewing; the axis mappings and the gyro
  unit should be verified in shadow mode against the encoders before the IMU feedback is used
  (see [`realtime/README.md`](realtime/README.md)).
- Gravity does not observe the slew angle. In the IMU-only run, the slew angle was dead-reckoned from the gyro rates;
  an absolute slew sensor is recommended for sustained operation.

## Citation

If you use this code, please cite the article:

> M. Heydari Shahna, P. Mustalahti, and J. Mattila, "IMU-Based and Auto-Tuned Virtual Decomposition Control for a
> Hydraulic Rock-Breaker Boom," 2026.

The preprint link and a BibTeX entry will be added here when the preprint is available.

## Acknowledgment

This work was supported by the Finnish Foundation for Technology Promotion (Tekniikan Edistämissäätiö) through the
Post Docs in Companies (PoDoCo) program.

## Contact

Mehdi Heydari Shahna, Tampere University — mehdi.heydarishahna@tuni.fi
