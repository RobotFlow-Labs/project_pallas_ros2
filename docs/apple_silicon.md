# Apple Silicon and Jetson Portability

PALLAS is designed so the ROS2 core stays portable across standard ARM and x86 targets. The estimation path is written in portable C++ with Eigen and ROS2 primitives, so it should run anywhere the ROS2 toolchain is available, including Apple Silicon environments and Jetson-class devices, without requiring platform-specific rewrites.

## Portability Boundary

- Core odometry, motion integration, and map maintenance remain in portable C++.
- Eigen is the main numerical dependency for the core runtime.
- ROS2 is the integration layer, not the place where hardware-specific logic should live.

## Optional Acceleration

MLX is treated as an optional future accelerator for learned or perception-heavy modules only, especially on Apple Silicon.

- good fit: learned feature extraction, perception post-processing, and other model-driven components
- not in scope for core odometry
- not required for baseline deployment, benchmarking, or deterministic runtime validation

This keeps the core runtime deployable on Apple Silicon and Jetson while leaving room for future GPU/accelerator-specific work where it actually adds value.
