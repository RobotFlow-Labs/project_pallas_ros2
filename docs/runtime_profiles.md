# Runtime Profiles

PALLAS exposes two runtime profiles so deployment intent is explicit from the start.

## PALLAS Core

Lean realtime profile for production robotics.

- portable C++/Eigen ROS2 core
- lowest practical dependency surface
- optimized for deterministic execution and fast iteration
- intended for odometry, fusion, and map maintenance

Use this profile when the priority is stable runtime behavior on constrained hardware, including Apple Silicon and Jetson targets.

## PALLAS CT

Research profile for continuous-time and spline-based development.

- includes the continuous-time spline estimation path
- suited for experimentation, solver tuning, and trajectory refinement
- higher compute cost than Core
- intended for offline analysis, lab validation, and algorithm development

Use this profile when the priority is model fidelity and research velocity rather than minimal runtime cost.

## Selection Rule

- choose `PALLAS Core` for realtime deployment
- choose `PALLAS CT` for research and algorithm iteration
- keep MLX and other accelerator-specific work behind optional modules, not in the core profile
