#ifndef __GLOBAL__
#define __GLOBAL__

#include "lib/lib.hpp"
#include <numeric>

// Global hardware mappings and state variables
namespace glb
{
    // Motors
    pros::Motor frontLeft(7, pros::E_MOTOR_GEARSET_06, true); 
    pros::Motor midLeft(8, pros::E_MOTOR_GEARSET_06, false);
    pros::Motor backLeft(9, pros::E_MOTOR_GEARSET_06, true);
    pros::Motor frontRight(4, pros::E_MOTOR_GEARSET_06, false);
    pros::Motor backRight(2, pros::E_MOTOR_GEARSET_06, false);
    pros::Motor midRight(3, pros::E_MOTOR_GEARSET_06, true);
    pros::Motor yuuta(1, pros::E_MOTOR_GEARSET_18, false);
    pros::Motor saki(10, pros::E_MOTOR_GEARSET_18, true);

    // Pistons
    pros::ADIDigitalOut boost('B'); 
    pros::ADIDigitalOut derrick('C'); 
    pros::ADIDigitalOut expansion('D'); 
    pros::ADIDigitalOut release('E');

    // Sensors
    pros::Imu imu(5);
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    pros::ADIDigitalIn limit(1);
    pros::Optical optical(20);

    // Global variables
    bool red;
    util::coordinate pos(0,0);
}

// High-level robot objects, wrapping hardware into abstractions
namespace robot
{
    // Drive and subsystems
    lib::diffy chass(std::vector<pros::Motor>{glb::frontLeft, glb::midLeft, glb::backLeft, glb::frontRight, glb::midRight, glb::backRight}); 
    lib::diffy itsuki(std::vector<pros::Motor>{glb::yuuta, glb::saki});

    // Pistons wrapped with helper class
    lib::pis boost({glb::boost}, false, ""); 
    lib::pis tsukasa({glb::derrick}, false, "");
    lib::pis expansion({glb::expansion}, false, "");
    lib::pis release({glb::release}, false, "");

    // Sensors wrapped with helper class
    lib::imu imu(glb::imu, 0); 
    util::controller controller(glb::controller);
}

#endif
