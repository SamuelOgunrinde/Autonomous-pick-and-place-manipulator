# Autonomous Pick-and-Place Manipulator
##  Overview
This project implements a **4-DOF autonomous robotic arm** designed for pick-and-place tasks using SG90 servos.  
It integrates **Finite State Machine (FSM) control**, **Inverse Kinematics (IK)**, and **trapezoidal velocity profiling** for smooth and reliable motion.
##  Control Logic (FSM)
States:
- `IDLE`
- `MOVE_TO_PICK`
- `GRIP`
- `LIFT`
- `MOVE_TO_PLACE`
- `RELEASE`
- `RETURN_HOME`
*FSM diagram in `/docs`*

---

##  Inverse Kinematics
Inverse kinematics computes joint angles required to reach a desired end-effector position.

- Converts Cartesian coordinates *(x, y, z)* → joint space  
- Implemented using geometric relationships  
- Ensures targets lie within the manipulator workspace  

---

##  Motion Control
<img width="480" height="250" alt="Trapezoidal-velocity-profile" src="https://github.com/user-attachments/assets/f3c8481b-9321-4597-bc2d-94f721fe46ee" />

Motion is generated using a trapezoidal velocity profile:

- **Acceleration phase** – velocity increases smoothly  
- **Constant velocity phase** – steady motion  
- **Deceleration phase** – smooth stop at target  

This reduces mechanical stress and improves positioning stability. 

---

##  Hardware
- 4 × SG90 servos  
- IR proximity sensor  
- STM32F103RB Nucleo-board
- Buck-converter
- 7.4v lithium-ion battery
- Breadboard
- Jumper wires

---

##  Demo
*Add video/GIF here*

---

##  Limitations
- Limited payload (servo torque)  
- Short-range sensing  
