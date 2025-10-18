#ifndef __CATA__
#define __CATA__

#include "global.hpp"

/**
 * Catapult (cata) subsystem.
 * Handles firing, reloading, pausing, and boost logic.
 */
namespace cata
{
    enum states { idle, firing, reloading, paused };

    // Current state of the catapult
    states curr;

    // Boost flag (extra power when enabled)
    bool boost;

    // Timers for controlling reload and boost phases
    util::timer boostTimer;
    util::timer slowTimer;

    // Mutex to prevent race conditions in control loop
    pros::Mutex smtx;

    /**
     * Main control loop for the catapult.
     * Manages transitions between idle, firing, reloading, and paused.
     */
    void control()  
    {
        while(true)
        {
            smtx.lock();
            bool limit = glb::limit.get_value();

            switch(curr)
            {
                case idle:
                    std::cout << "cata idle! limit = " << limit << std::endl;
                    break;

                case firing:
                    std::cout << "cata firing! limit = " << limit << std::endl;
                    boost ? robot::boost.setState(true) : robot::boost.setState(false);
                    if(limit)
                    {
                        robot::itsuki.spin(-127);
                    }
                    else
                    {
                        curr = reloading;
                        boostTimer.start();
                        slowTimer.start();
                    }
                    break;
                
                case reloading:
                    std::cout << "cata reloading! limit = " << limit << std::endl;

                    if(boostTimer.time() >= 310)
                    {
                        robot::boost.setState(false);
                    }
        
                    if(slowTimer.time() >= 310)
                    {
                        if(slowTimer.time() <= 1310)
                        {
                            robot::itsuki.spin(-127);
                        }
                        else
                        {
                            robot::itsuki.spin(-75);
                        }
                    }
                    else
                    {
                        robot::itsuki.stop('b');
                    }
                    
                    if(limit)
                    {
                        robot::itsuki.stop('b');
                        curr = idle;
                    }
                    break;

                case paused:
                    std::cout << "cata paused!" << std::endl;
                    robot::itsuki.stop('b');
                    curr = reloading;
                    break;
            }

            smtx.unlock();
            pros::delay(20);
        }
    }

    /** Trigger a firing sequence */
    void fire()
    {
        smtx.lock();
        curr = firing;
        smtx.unlock();
    }  

    /** Pause the catapult (forces reload afterwards) */
    void pause()
    {
        smtx.lock();
        curr = paused;
        smtx.unlock();
    }
} 

#endif
