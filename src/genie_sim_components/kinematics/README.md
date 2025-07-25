# 🦾 genie_sim_components.kinematics

This module provides reusable kinematics components for simulation and control in the Genie Sim stack. It includes robot-specific inverse and forward kinematics wrappers, with integration of third-party solvers for high-performance control.

---

## 📁 Structure

```
kinematics/
├── g1_relax_ik.py # Relaxed IK wrapper for G1 robot arms
├── configs/
│ └── g1/
│ ├── G1_NO_GRIPPER.urdf # IK-only URDF for G1 robot
│ └── g1_solver.yaml # Relaxed IK configuration
├── examples/
│ └── run_g1_relax_ik_example.py
└── docs/
└── g1_relax_ik.md
```


---

## 📌 Requirements

- Python 3.10+
- `scikit-learn`
- Relaxed IK wheel (`relaxed_ik_wrapper`) from AgiBot’s [genie_sim](https://github.com/AgibotTech/genie_sim)

Install dependencies using the provided `setup.sh` in the repo root:

```bash
./setup.sh
```

---


## 🧠 G1 Relaxed IK (g1_relax_ik.py)
A high-level wrapper around the AgiBot Relaxed IK solver (relaxed_ik_wrapper), specifically configured for the G1 robot.

### ✅ Features

- IK solving using target pose (4×4) or (pos + quat)

- Supports both left and right 7-DOF arms

- Forward kinematics (FK) from joint angles to SE(3) pose

- Based on robot-specific URDF and solver YAML config

---

### 📍 Kinematic Root Frame
All kinematic calculations — both IK input and FK output — are defined relative to the link:

```nginx
arm_base_link
```
This is hardcoded in the solver config:

```yaml
target_links:
  right_arm:
    base: arm_base_link
```
❗️Do not pass world or base_link poses directly into IK unless you've manually transformed them into arm_base_link frame first.

---

### 🧪 Example Usage
Run the example:
```
PYTHONPATH=src python src/genie_sim_components/kinematics/examples/run_g1_relax_ik_example.py
```

### 📚 Docs
See docs/g1_relax_ik.md for solver configuration, weights, joint limits, and integration tips.

