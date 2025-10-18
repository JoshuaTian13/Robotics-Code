#include "global.hpp"
#include "lib/lib.hpp"
#include "lib/robot/util/util.hpp"

/**
 * @file odom.cpp
 * @brief Odometry tracking loop using encoders and IMU.
 */

void odom() {
    glb::leftEncoder.reset();
    glb::horizEncoder.reset();

    double prevRotation = glb::imu.get_heading();
    double deltaX = 0;
    double deltaY = 0;

    // Tracking constants
    double trackingDiameter = 2.75;
    double scaleFactor = 5.3625;
    double trackingCircumference = trackingDiameter * PI;
    double horizOffset = 0 * scaleFactor;
    double vertOffset = 0 * scaleFactor;

    while (true) {
        // Change in rotation
        double currRotation = robot::imu.degHeading();
        double deltaRotation = util::dirToSpin(prevRotation, currRotation) * 
                               util::minError(currRotation, prevRotation);

        prevRotation = currRotation;
        deltaRotation = util::dtr(deltaRotation);
        currRotation = util::dtr(currRotation);

        // Change in encoder values
        double deltaVert = (trackingCircumference / 360) * glb::leftEncoder.get_value() * scaleFactor;
        double deltaHoriz = (trackingCircumference / 360) * glb::horizEncoder.get_value() * scaleFactor;

        if (deltaRotation == 0) {
            deltaY = cos(2 * PI - currRotation) * deltaVert;
            deltaX = sin(2 * PI - currRotation) * deltaVert;
        } else {
            // Change in relative Y
            double sOverTheta = (deltaVert / deltaRotation) + horizOffset;
            double relativeY = 2 * sin(deltaRotation / 2) * sOverTheta;

            // Change in relative X
            sOverTheta = (deltaHoriz / deltaRotation) + vertOffset;
            double relativeX = 2 * sin(deltaRotation / 2) * sOverTheta;

            // Convert to absolute X and Y
            double rotationOffset = currRotation + (deltaRotation / 2);
            double theta = atan2(relativeY, relativeX);
            double radius = sqrt(relativeX * relativeX + relativeY * relativeY);

            theta -= rotationOffset;
            deltaX = radius * cos(theta);
            deltaY = radius * sin(theta);
        }

        // Update chassis position
        robot::chass.updatePos(deltaX, deltaY);

        // Reset encoders
        glb::horizEncoder.reset();
        glb::leftEncoder.reset();

        pros::delay(10);
    }
}
