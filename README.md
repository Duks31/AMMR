# Cika: An Autonomous Mobile Manipulator for Waste Collection and Sorting

![cika](/static/cika.jpeg)

## The problem

Waste gets mixed at the point of disposal, plastic and paper end up in the same bin, which kills recycling before it starts. Fixing that manually means more staff hours and more exposure to unsorted waste. Stationary smart bins don't help because they can't go collect litter that never made it into a bin, and most mobile waste robots on record either navigate or sort, not both, and almost none run on renewable power.

Cika is a 4WD skid-steer mobile robot with a 6-DOF arm that finds waste, classifies it as paper or plastic, picks it up, and sorts it.

## System overview

2 subsystems, one control loop:

- **Mobile base**: skid-steer drivetrain, RPLIDAR C1 for 360° scanning, OAK-D Lite for RGB-D and onboard neural inference
- **Manipulator**:  6-DOF serial arm (MG966R servos + a NEMA-17 stepper for base yaw), driven by a PCA9685 PWM controller

Control is split hierarchically: a Raspberry Pi runs ROS2 (Nav2, RTAB-Map, MoveIt2, perception), and talks over serial UART to two ESP32s, one for the drivetrain, one for the arm,  which handle the low-level PWM.

A five-state task manager (`IDLE → SELECTING → NAVIGATING → VERIFYING → PICKING`) ties perception, navigation, and manipulation together: it scores detections by combined detector + classifier confidence, sends the robot to the target, re-confirms it's still there over several frames before committing, then hands off to the arm.

![cika](/static/cika_high_level_block_diagram.png)

![cika](/static/cika_mobile_base_and_arm.png)

## Engineering decisions that mattered

**RTAB-Map over SLAM Toolbox.**: I ran both. SLAM Toolbox was the obvious first choice, but RTAB-Map's graph-based approach with appearance-based loop closure held up better on the fused odometry (wheel encoders + IMU + OAK-D Lite visual odometry) once the environment stopped being a clean lab floor. That's the version that shipped.

**27+ sim iterations before the first working build.**: The design phase bounced between Gazebo and Fusion360 then model in Fusion, test the kinematics and drive behavior in Gazebo, find what breaks, go back. That loop is why the final chassis and arm geometry actually match what the drivetrain and manipulator can do, instead of looking right in CAD and failing on hardware.

**Porting the OAK-D Lite pipeline from depthai v2 to v3.**: Not a design choice so much as a forced one, v2 had USB power issues and device crashes that weren't going away. Then I changed to v3 port which meant re-debugging the perception pipeline from scratch, including API breakages that weren't documented anywhere I could find.

**Serial UART instead of micro-ROS for the ESP32s.** micro-ROS was the original plan for the drivetrain and arm controllers. It didn't hold up reliably enough on the Pi ↔ ESP32 link, so I moved both to a plain serial bridge instead, simpler, and it's what's running on the final hardware.

**IMU sign convention (REP-103).** A small thing that cost real debugging time: getting the IMU's axis conventions to actually match ROS's REP-103 standard before the EKF fusion would converge properly.

## What the numbers actually say

I'm including the full metrics table rather than just the highlights, because the gaps are more informative than the wins.

| Metric | Expected | Actual | Deviation |
|---|---|---|---|
| Binary classification accuracy | 100% | 87.4% | 12.6% |
| Object detection accuracy | 75% | 75–81% | within range |
| Navigation success rate | 100% | 58.3% | 41.7% |
| Localization accuracy | 0.05 m | 0.15 m | 66.7% |
| Picking accuracy | 100% | 80% | 20% |
| Wireless power transfer efficiency | 71.4% | 64.3% | 9.9% |
| Positional repeatability | ±0.05 m | ±0.112 m | 55.4% |

The classification and detection numbers (87.4% / 75–81%) are solid for a 2-class YOLOv8 model trained on a TACO subset from a Raspberry Pi-class edge device. The object-detection mAP@0.5 (38.9%) is weaker than the raw accuracy numbers suggest, the confusion matrix shows the model is conservative, preferring to call things "background" over risking a false positive, which is a reasonable failure mode for a robot arm about to grab something.

The navigation numbers are the honest weak point: 58.3% goal success rate against a mean heading error of 16.7° and localization error of 0.15 m. That traces to skid-steer drivetrain slip and asymmetry feeding odometry drift, not a software failure. The Nav2 + RTAB-Map stack did what it was supposed to do with the odometry it was given; the odometry itself wasn't good enough. That's a mechanical problem (wheel bearings, motor bracket tolerances) as much as a perception one.

![cika](/static/cika_training_curves.png)
![cika](/static/cika_occupancy_grid.png)
![cika](/static/cika_normalized_confusion_matrix.png)
## Stack

ROS2 Humble · Nav2 · RTAB-Map · MoveIt2 · YOLOv8 · OAK-D Lite (depthai v3) · RPLIDAR C1 · Extended Kalman Filter sensor fusion · Raspberry Pi 4B · ESP32 · Docker · ONNX / OpenVINO