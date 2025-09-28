# Drone to Robot Package Delivery with Real Time Machine Vision 📦

## Project Overview📖
This repository presents the drone subsystem of an autonomous delivery system.
It integrates:
* Flight Control (Tello Talent + djitellopy)
* Real-Time Machine Vision (ArUco detection + pose estimation)
* Global Positioning (vector math + IMU correction)
* Human Interaction Module (audio feedback via Raspberry Pi speakers)
The goal: Detect and localize packages in GPS-denied indoor environments, and interact with recipients once delivered.
## System Architecture
* **Drone Subsystem (UAV):** Flight control (djitellopy), ArUco-based vision, positioning.
* **Ground Robot (UGV):** ROS2-based navigation, SLAM with LiDAR + odometry, package pickup with gripper.
* **Communication:** Wi-Fi + UDP, modular ROS2 nodes.
* **User Interaction:** Audio feedback for recipient, with applications for visually impaired users.
## Technical Details⚙️
### Flight Control✈️
* Implemented custom strategic traversal patterns (phased scanning).
* Achieved average distance error ~9 cm over 3 m flights.
* Real-time IMU data integration for stable orientation.
### Machine Vision📸
* Camera + mirror calibration (0.72 reprojection error).
* ArUco detection pipeline (30 FPS raw, optimized to 10 FPS on Pi, shifted to workstation).
* Pose estimation with cv2.solvePnP + IMU correction, reducing error to ~10 cm.
* Frame management to prevent lag from streaming.
### Positioning System🗺️
* Implemented vector-based global positioning:
* Drone position (A) + package relative vector (B) → package global position (C).
* Initially tested ORB-based visual odometry, abandoned due to noise.
* Final system uses Tello’s internal visual positioning system, achieving ~50 cm error in global package location.
### User Interaction🤝
* Integrated Raspberry Pi speaker system for audio delivery messages.
* Pre-recorded alerts for recipients, enhancing accessibility.

**Future extension: multi-modal feedback (visual + haptic).**
## Results🏆
* **Flight Accuracy:** ±9 cm error over test distances.
* **Pose Estimation:** IMU fusion reduced drift to ~10 cm.
* **Global Package Localization:** < 50 cm error from true position.
* **Real-Time Performance:** 30 FPS camera stream, ~10 FPS effective pipeline.
* **Recipient Interaction:** Clear audio alerts successfully tested.

## Applications🪛
* **Autonomous Warehousing** – drones + ground robots for package retrieval.
* **Last-Mile Delivery** – seamless package delivery in GPS-denied environments (indoor campuses, hospitals).
* **Accessibility** – package delivery for visually impaired users via audio feedback.
* **Logistics Automation** – scalable integration for real-world logistics networks.

