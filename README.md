# Drone to Robot Package Delivery with Real Time Machine Vision 📦

## Project Overview📖
This repository presents the drone subsystem of an autonomous delivery system.
<p align = "center"><img src ="https://github.com/user-attachments/assets/b97bcf9d-24f2-42f7-9929-ec5e83d0cffd" width = 720></p>

It integrates:
* Flight Control (Tello Talent + djitellopy)
  <p align = "center"><img src ="https://github.com/user-attachments/assets/f90057cf-da24-40a3-b5c5-8a742264deec" width = 480></p>
* Real-Time Machine Vision (ArUco detection + pose estimation)
* Global Positioning (vector math + IMU correction)
* Human Interaction Module (audio feedback via Raspberry Pi speakers)

**The goal:** Detect and localize packages in GPS-denied indoor environments, and interact with recipients once delivered.
## System Architecture
* **Drone Subsystem (UAV):** Flight control (djitellopy), ArUco-based vision, positioning.
* **Ground Robot (UGV):** ROS2-based navigation, SLAM with LiDAR + odometry, package pickup with gripper.
  
<p align = "center"><img src ="https://github.com/user-attachments/assets/e4ce6c78-abae-411b-99ca-4619e23d2d2c" width = 360></p>
<p align = "center"><img src ="https://github.com/user-attachments/assets/4f27392d-4a9a-4697-be79-8367c326e6e4" width = 360></p>

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
<p align = "center">  <img width="640" height="360" alt="Machine Vision Subsystem" src="https://github.com/user-attachments/assets/5f72b066-df04-47cc-be1b-2af53c774a3a" />
</p>

### Positioning System🗺️
* Implemented vector-based global positioning:
* Drone position (A) + package relative vector (B) → package global position (C).
* Initially tested ORB-based visual odometry, abandoned due to noise.
* Final system uses Tello’s internal visual positioning system, achieving ~50 cm error in global package location.
<p align = "center">  <img width="640" height="360" alt="Positioning Subsystem" src="https://github.com/user-attachments/assets/bf592aa0-801f-4c7f-8fa4-268c03263d50" />
</p>


### User Interaction🤝
* Integrated Raspberry Pi speaker system for audio delivery messages.
* Pre-recorded alerts for recipients, enhancing accessibility.
<p align = "center">  <img width="480" height="480" alt="User Interaction System" src="https://github.com/user-attachments/assets/e45bf7a2-b25d-41e7-ab16-e1a564a15452" />
</p>

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

