#ifndef __LIB_HPP__
#define __LIB_HPP__

#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "util.hpp"
#include <vector>
#include <functional>

/**
 * @file lib.hpp
 * @brief Library of generic input abstractions and event listeners.
 */

namespace lib {

    /* ---------------- Base Digital ---------------- */
    class digital {
    public:
        virtual bool getState() { return true; }
    };

    /* ---------------- Controller Button ---------------- */
    class controllerButton : public digital {
    private:
        pros::Controller con;
        pros::controller_digital_e_t button;

    public:
        controllerButton(pros::Controller cont, pros::controller_digital_e_t key) 
            : con(cont), button(key) {}

        bool getState() override {
            return con.get_digital(button);
        }
    };

    /* ---------------- Limit Switch ---------------- */
    class limit : public digital {
    private:
        pros::ADIDigitalIn sensor;

    public:
        limit(pros::ADIDigitalIn limitSwitch) : sensor(limitSwitch) {}

        bool getState() override {
            return sensor.get_value();
        }
    };

    /* ---------------- Action ---------------- */
    struct action {
        lib::digital* binaryIn;
        std::function<void(void)> pressed;
        std::function<void(void)> unpressed;

        action(lib::digital* digitalIn, 
               std::function<void(void)> pressAction, 
               std::function<void(void)> releaseAction)
            : binaryIn(digitalIn), pressed(pressAction), unpressed(releaseAction) {}
    };

    /* ---------------- Listener ---------------- */
    class listener {
    private:
        std::vector<lib::action> actions;

    public:
        listener(int) {}
        listener(std::vector<lib::action> action) : actions(action) {}

        void init(std::vector<lib::action> action) {
            actions = action;
        }

        void listen() {
            for (auto &a : actions) {
                if (a.binaryIn->getState()) {
                    a.pressed();
                } else {
                    a.unpressed();
                }
            }
        }
    };

} // namespace lib

#endif
