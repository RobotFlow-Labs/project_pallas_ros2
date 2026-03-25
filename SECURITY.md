# Security Policy

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 0.1.x   | :white_check_mark: |

## Reporting a Vulnerability

If you discover a security vulnerability in PALLAS, please report it responsibly.

**Do NOT open a public GitHub issue for security vulnerabilities.**

Instead, email **security@robotflowlabs.com** with:

1. Description of the vulnerability
2. Steps to reproduce
3. Impact assessment
4. Suggested fix (if any)

We will acknowledge receipt within 48 hours and aim to provide a fix or mitigation plan within 7 days.

## Scope

PALLAS is a perception module intended for research and development use. It processes sensor data (LiDAR point clouds and IMU measurements) and should be deployed with the following considerations:

- **Sensor data integrity**: PALLAS trusts incoming ROS2 messages. In adversarial environments, validate sensor data upstream.
- **Network exposure**: ROS2 DDS traffic is unencrypted by default. Use SROS2 or network segmentation for production deployments.
- **Demo assets**: Downloaded archives are verified by structure but not cryptographically signed. Use `PALLAS_DEMO_BASE_URL` only with trusted HTTPS endpoints.
- **Extrinsic calibration**: Incorrect sensor-to-body transforms can produce unsafe pose estimates. Always validate calibration before deploying on mobile platforms.

## Safety Notice

PALLAS provides odometry estimates, not safety-critical localization. Do not use PALLAS output as the sole input to safety-critical control systems without independent verification.
