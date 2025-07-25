# Relaxed IK Solver Wheel

This folder contains a locally downloaded `.whl` file for the **Relaxed IK Solver** used in the [Agibot Genie Sim](https://github.com/AgibotTech/genie_sim) project.

## 🔗 Source

- Repository: [AgibotTech/genie_sim](https://github.com/AgibotTech/genie_sim)
- Commit: `66143533b818181b6c028cb2660b488820098664`  
- Wheel path: `3rdparty/ik_solver-0.4.3-cp310-cp310-linux_x86_64.whl`

This IK solver is implemented in C++ and exposed via Python bindings, built specifically for Agibot’s use cases in simulation and robot control.

## 📦 Usage

Install via pip from this folder:

```bash
pip install third_party/g1_relaxed_ik/ik_solver-0.4.3-cp310-cp310-linux_x86_64.whl
```
## Note

- The module is used by genie_sim_components/kinematics/relaxed_ik_wrapper.py to solve inverse kinematics tasks in a relaxed manner (soft constraints, velocity limits, etc.).

## 📌 Disclaimer
This file is a prebuilt binary from the AgibotTech repository. Please refer to their project for updates or source code:
https://github.com/AgibotTech/genie_sim

---