# xArm Vision-Based Teleoperation & HMI

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![MediaPipe](https://img.shields.io/badge/Vision-MediaPipe-green.svg)](https://developers.google.com/mediapipe)
[![OpenCV](https://img.shields.io/badge/Vision-OpenCV-red.svg)](https://opencv.org/)
[![xArm](https://img.shields.io/badge/Hardware-uFactory_xArm_Lite_6-orange.svg)](https://www.ufactory.cc/)

## Overview
This repository explores advanced Human-Machine Interaction (HMI) by leveraging computer vision to teleoperate industrial robotics and PC peripherals. By eliminating the need for physical controllers or teach pendants, this project aims to create a more intuitive, real-time control interface using hand tracking and gesture recognition.

Developed at **Tecnológico de Monterrey** as part of a Mechatronics Engineering initiative, this system serves as a sandbox for testing non-traditional robotic control methods and HMI integration.

## Key Features
* **Real-Time Robot Teleoperation:** Uses Google's MediaPipe to track hand landmarks and maps 2D camera coordinates (pixels) into a bounded 3D workspace for the xArm Lite 6.
* **Gesture-Based Gripper Control:** Calculates the Euclidean distance between specific hand landmarks (e.g., thumb and index finger) to actuate the robotic gripper dynamically.
* **Safety First (Clamping & Bounds):** Implements workspace restrictions (`ROBOT_X_MIN`, `ROBOT_X_MAX`) to prevent the robot from colliding with its environment during teleoperation.
* **PC Peripheral Emulation:** Translates specific hand states (e.g., raised index vs. raised pinky) into keyboard keystrokes (`PyAutoGUI`) to control software interfaces or presentations touch-free.

## Repository Structure

```text
xArm-Vision-Teleoperation/
├── teleoperation/
│   └── hand_teleoperation.py         # Main script: Maps hand coordinates to xArm movement
├── experiments/
│   └── gesture_keyboard_mapping.py   # Utility: Controls PC media/slides using gestures
