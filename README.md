Core Algorithms & Utilities

util.hpp – Core math and control utilities.
Implements PID controllers, moving averages, Bezier curves for path planning, coordinate/pose representations, and math helpers.
👉 Most important file for showcasing algorithms.

Autonomous Logic

autons.hpp – Defines multiple autonomous routines (wp, eightwp, near, far, skills, etc.) built on top of subsystems and util’s algorithms.
👉 Demonstrates how algorithms integrate into real match strategies.

odom.hpp – Experimental odometry tracking using encoders and IMU to update robot position.
Shows applied math (rotation matrices, encoder deltas) in real-time localization.

Subsystems

chassis.hpp – Drive train logic and motion control (turning, driving, arc turns).

cata.hpp – Catapult control system with reload/fire logic.

intake.hpp – Intake system with states (intaking, idling, awaiting, etc.) and async actions.

controls.hpp – Driver control mappings and input handling.

sensors.hpp – Sensor interfaces (IMU, encoders, limit switches).

groups.hpp – Hardware abstraction layer: defines motor groups, pistons, and sensors as named objects so the rest of the code can use them cleanly.

Integration & Runtime

global.hpp – Global objects and namespaces that tie together subsystems.

lib.hpp – Umbrella include that brings in core robot files.

main.cpp – Entry point. Initializes sensors, launches background tasks (intake, cata control), and dispatches to autonomous or driver control.

Experimental / Less Central

opc.hpp – Unused; appears to be an experimental operator control test.

stager.hpp – Unused; likely an experimental attempt at staging actions.

These files aren’t referenced by lib.hpp (they’re commented out) and don’t affect the main build. They’re kept here for completeness but are not central to the project.
