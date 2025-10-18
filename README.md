# Robotics Codebase – Autonomous & Control Utilities

This repository contains the core code I developed for a competitive robotics system, with a focus on control algorithms, autonomous routines, and subsystem management.  

The game objective was relatively straightforward: there were **two elevated goals (buckets)**, and the robot’s main task was to **shoot discs into its own goal** while preventing the opponent from scoring. There were other methods of scoring, but the code is mainly focused on this aspect of the game. Success relied on accuracy, efficiency, and coordination between subsystems like the drive, intake, and catapult.

The highlight of this project is the **`util` library**, which implements reusable math and control algorithms (PID controllers, Bezier curves, moving averages, etc.) that power higher-level autonomous behavior. Other files define subsystems (intake, catapult, drive) and orchestrate them into full autonomous routines.

---

## File Overview

### **Core Algorithms & Utilities**
- **`util.hpp`** – Core math and control utilities.  
  Implements PID controllers, moving averages, Bezier curves for path planning, coordinate/pose representations, and math helpers.  
  **Most important file** for showcasing algorithms.

### **Autonomous Logic**
- **`autons.hpp`** – Defines multiple autonomous routines (`wp`, `eightwp`, `near`, `far`, `skills`, etc.) built on top of subsystems and `util`’s algorithms.  
  Demonstrates how algorithms integrate into real match strategies.

- **`odom.hpp`** – Odometry tracking using encoders and the IMU to update robot position.  
  Shows applied math (rotation matrices, encoder deltas) in real-time localization.

### **Subsystems**
- **`chassis.hpp`** – Drive train logic and motion control (turning, driving, arc turns).  
- **`cata.hpp`** – Catapult control system with reload/fire logic.  
- **`intake.hpp`** – Intake system with states (`intaking`, `idling`, `awaiting`, etc.) and async actions.  
- **`controls.hpp`** – Driver control mappings and input handling.  
- **`sensors.hpp`** – Sensor interfaces (IMU, encoders, limit switches).  
- **`groups.hpp`** – Hardware abstraction layer: defines motor groups, pistons, and sensors as named objects so the rest of the code can use them cleanly.

### **Integration & Runtime**
- **`global.hpp`** – Global objects and namespaces that tie together subsystems.  
- **`lib.hpp`** – Umbrella include that brings in core robot files.  
- **`main.cpp`** – Entry point. Initializes sensors, launches background tasks (intake, cata control), and dispatches to autonomous or driver control.

### **Experimental / Less Central**
- **`opc.hpp`** – Prototype file for operator control experiments.  
- **`stager.hpp`** – Prototype file for staging/sequencing actions.  

These were used for testing concepts and are not part of the main runtime build. They remain in the repo as reference material but are not essential to the core system.

---

## What to Focus On
If you’re short on time reviewing this repo:  
1. **`util.hpp`** – Core algorithms I wrote from scratch.  
2. **`autons.hpp`** – How those algorithms get applied in real strategies.  
3. **`odom.hpp`** – Applied math for localization.  

The subsystem files (`intake`, `cata`, `chassis`) show the control flow, but the heart of my work lies in `util` and its integration into autonomous logic.
