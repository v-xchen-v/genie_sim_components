# Relaxed IK Integration

This document provides an overview of how to use the **Relaxed IK Solver** for the G1 robot, powered by Agibot's [`genie_sim`](https://github.com/AgibotTech/genie_sim) library. It covers features, configuration, and troubleshooting tips.

---

## ✅ What is Relaxed IK?

Relaxed IK is an inverse kinematics (IK) solver that goes beyond traditional hard constraint solvers by allowing *soft constraints*, *velocity smoothing*, and *joint limit regularization*. It is suitable for real-time and physically plausible motion generation in simulation or real robots, especially when:

- The target pose is unreachable.
- You need smooth transitions between steps.
- You want to stay within joint limits but not fail completely.

This solver is implemented in C++ and exposed via Python through a `.whl` file, built from the [AgibotTech/genie_sim](https://github.com/AgibotTech/genie_sim) repo.

---

## 📂 Integration: G1RelaxSolver
G1RelaxSolver is a lightweight wrapper around the Relaxed IK solver, designed to work seamlessly with the G1 robot's left and right arms.

### Features:
- Inverse Kinematics from SE(3) or position + quaternion

- Forward Kinematics from joint angles

- Supports 7-DOF left and right arms
## 🧭 Coordinate Frame Convention
The solver is configured with the following kinematic root frame:
```
arm_base_link
```
All IK input poses and FK output poses are expected to be in this frame.

⚠️ If your pose is defined in base_link, world, or another frame, you must transform it into the arm_base_link frame before solving.


## 🔧 Configuration Parameters (g1_solver.yaml)
Here are some important parameters in the solver config:

| Parameter                 | Description                                                    |
|--------------------------|----------------------------------------------------------------|
| `pos_weight`             | Weight on end-effector position accuracy                       |
| `ori_weight`             | Weight on end-effector orientation accuracy                    |
| `min_velocity_weight`    | Penalizes joint velocity changes (for smoothness)              |
| `min_acceleration_weight`| Penalizes sudden accelerations (jerk control)                 |
| `joint_limits_weight`    | Penalizes joint limit violations                               |
| `max_solver_time`        | Maximum computation time per solve (in seconds)                |
| `xtol_rel`, `ftol_rel`   | Convergence thresholds for the numerical optimizer             |


---

## 📚 References

- AgibotTech/genie_sim (GitHub)

- URDF used: genie_sim_components/kinematics/configs/g1/G1_NO_GRIPPER.urdf

- Config YAML: genie_sim_components/kinematics/configs/g1/g1_solver.yaml