#ifndef __INTAKE__
#define __INTAKE__

#include "global.hpp"

/**
 * @file intake.hpp
 * @brief Intake subsystem with state machine, async actions, and piston control.
 */

namespace intake
{
    // Intake operating states
    enum states { intaking, idling, awaiting, pistonUp, pistonDown };

    // State variables
    states curr;
    pros::Mutex smtx;
    int speed;
    int delay;
    states next;
    util::timer timer;

    /**
     * @brief Control loop for intake state machine.
     * Manages intake motor and piston based on current state.
     */
    void control() {
        while (true) {
            smtx.take();

            switch (curr) {
                case intaking:
                    if (glb::limit.get_value()) {
                        robot::itsuki.spin(speed);
                    }
                    break;

                case idling:
                    break;

                case awaiting:
                    robot::itsuki.spin(127);
                    if (timer.time() > delay) {
                        robot::itsuki.stop('c');
                        curr = next;
                    }
                    break;

                case pistonUp:
                    robot::tsukasa.setState(true);
                    curr = intaking;
                    break;

                case pistonDown:
                    robot::tsukasa.setState(false);
                    curr = intaking;
                    break;
            }

            smtx.give();
            pros::delay(20);
        }
    }

    /** @brief Begin intaking at a given speed. */
    void spin(int speed) {
        smtx.take();
        intake::speed = speed;
        curr = intaking;
        smtx.give();
    }

    /** @brief Stop the intake motor. */
    void stop() {
        smtx.take();
        robot::itsuki.stop('c');
        curr = idling;
        smtx.give();
    }

    /**
     * @brief Perform an asynchronous intake action with delay.
     * @param state Next state to transition to.
     * @param delay Duration (ms) before switching to next state.
     * @param speed Motor speed (default: current intake speed).
     */
    void asyncAction(states state, int delay, int speed = intake::speed) {
        smtx.take();
        intake::speed = speed;
        intake::delay = delay;
        intake::next = state;
        intake::timer.start();
        curr = awaiting;
        smtx.give();
    }

    /**
     * @brief Perform an asynchronous piston down action with delay.
     * @param delay Duration (ms) before transitioning back to intake.
     */
    void asyncPiston(int delay) {
        smtx.take();
        intake::delay = delay;
        intake::next = intake::pistonDown;
        intake::timer.start();
        curr = awaiting;
        smtx.give();
    }
}

#endif
