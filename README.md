# 🤖 Robot Dog
 
Custom quadruped robot developed by Matt Robotics

Designed and developed in Italy
 
A long-term robotics project focused on:
 
- Artificial Intelligence
- Computer Vision
- Embedded Systems
- 3D Printed Mechanics
- Autonomous Navigation
- Human-Robot Interaction
 
---
 
## 🎯 Project Goal
 
Develop an intelligent quadruped robot capable of:
 
- Walking autonomously
- Understanding voice commands
- Recognizing people and objects
- Interacting with the environment
- Learning new behaviours through AI
 
This repository documents the entire development process, including successes, failures, tests and design decisions.
 
---
 
## 📸 Current Status
 
🟡 Under Development
 
Current focus:
 
- Mechanical design
- Quadruped kinematics
- Head and neck system
- Electronics architecture
- AI hardware evaluation
 
---
 
## ⚙️ Hardware
 
### Computing
 
- Jetson Orin (planned)
- Raspberry Pi Pico 2 W
 
### Vision
 
- NUWA 60C Depth Camera
- RGB Camera
 
### Actuators
 
- ST3215 Serial Bus Servos
 
### Manufacturing
 
- QIDI Q2 3D Printer
- PPA-CF Components
 
---
 
## 🧠 Software
 
- Python
- Computer Vision
- AI Models
- Embedded Control
- Linux
 
---
 
## 🗺️ Roadmap
 
### Phase 1
- [x] Concept definition
- [x] Mechanical architecture
- [x] Preliminary CAD
 
### Phase 2
- [ ] Chassis prototype
- [ ] Head prototype
- [ ] Electronics integration
 
### Phase 3
- [ ] First autonomous walking
- [ ] Computer vision integration
- [ ] Voice interaction
 
### Phase 4
- [ ] Fully autonomous robot
 
---
 
## 📂 Repository Structure
 
```
robot-dog/
│
├── docs/
├── cad/
├── electronics/
├── firmware/
├── software/
├── bom/
├── media/
└── tests/
```
 
---
 
## 🙂 About
 
This repository is part of the Matt Robotics project.
 
The goal is to openly document the development of advanced robotics systems and share the engineering journey from concept to working prototype.
 
Follow the project on:
 
- YouTube (coming soon)
- Instagram (coming soon)
- GitHub
 
---
 
Built with passion in Italy by Matt Robotics
 

---

## MATDOG URDF REV00 Kinematic Baseline

The first complete MATDOG kinematic URDF baseline was completed on 2026-06-30.

Canonical engineering package:

`cad/urdf/matt_robodog_rev00/`

The package includes:

- Final URDF model
- Final baked STL meshes
- Collision mesh configuration
- Servo mapping
- Joint limits
- Kinematic, mass and material workbook
- Integrity manifest

Status:

**Approved for IK and gait development.**

Dynamic inertial properties remain pending CAD extraction.
