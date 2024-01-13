# Nvidia Kaya

## Overview

Kaya is a three-wheeled holonomic drive robot manufactured by Nvidia. This repository contains the control system developed for Kaya, created as part of a college assignment. Due to the unavailability of the old Isaac SDK, Docker and ROS2 Humble were used as the primary development tools.

## Key Features

### Robot Description

- **Name:** Kaya
- **Drive System:** Three-wheeled holonomic drive
- **Manufacturer:** Nvidia

### Assignment Context

- **Purpose:** Developed for a college assignment
- **Challenges:** Old Isaac SDK not available

### Development Environment

- **Tools Used:**
  - Docker
  - ROS2 Humble

### Achievements

#### Mapping

- Generated a 3D point cloud map using intelrealsense camera D435i
- Employed RtabMap for mapping and saved the generated map

#### Object Detection

- Implemented object detection using YOLOv8

#### Collision Avoidance

- Integrated collision avoidance mechanism into the control system by depth sensing of Intelrealsense camera D435i

#### Simulation and Visualization

- Integrated Isaac Sim for simulation
- Utilized RViz2 for real-time visualization

### Note

- **Isaac SDK:** Not used due to unavailability; Docker and ROS2 Humble were chosen as alternatives for development.


## Acknowledgments
Special thanks to Nvidia for the Kaya robot platform.
Acknowledgments for the college assignment given by Professor Sunil Jha inspired this project and to my teammate Varun Shukla
