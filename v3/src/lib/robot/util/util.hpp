#ifndef __UTIL__
#define __UTIL__

#include "main.h"
#include "pros/misc.h"
#include <cmath>
#include <vector>

#define PI 3.14159265358979323846
#define ALLBUTTONS {pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_DOWN, pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT, pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_B, pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A}
typedef void(*fptr)();

/**
 * @file util.hpp
 * @brief Utility namespace with math helpers, PID controllers, 
 * moving averages, Bezier paths, timing utilities, and controller helpers.
 */
namespace util {
    class timer;
    class coordinate;
    class pose;
    class bezier;
    class pidConstants;
    class pid;
    class movingAverage;
    class timeRange;
    class controller;

    double dtr(double input);
    double rtd(double input);
    int dirToSpin(double target, double currHeading);
    double minError(double target, double current);
    double distToPoint(util::coordinate p1, util::coordinate p2);
    double mod(double a, double b);
    double absoluteAngleToPoint(util::coordinate pos, util::coordinate point);
    double imuToRad(double heading);
    double sign(double a);
}

/* ---------------- Timer ---------------- */
class util::timer {
public:
    int startTime = 0;

    timer() { start(); }
    timer(int) {}

    void start() { startTime = pros::millis(); }
    int time() { return (pros::millis() - startTime); }
};

/* ---------------- Coordinate ---------------- */
class util::coordinate {
public:
    double x, y;
    coordinate(double px, double py) : x(px), y(py) {}
    coordinate();
};

/* ---------------- Pose ---------------- */
class util::pose {
public:
    util::coordinate pos;
    double heading;
    pose(util::coordinate p, double h) : pos(p), heading(h) {}
};

/* ---------------- Bezier Curve ---------------- */
class util::bezier {
private:
    coordinate p0, p1, p2, p3;

public:
    bezier(coordinate first, coordinate last, double initialWeight, double finalWeight, 
           double initialHeading, double finalHeading) {
        p0 = first;
        p1 = coordinate(first.x + sin(initialHeading) * initialWeight,
                        first.y + cos(initialHeading) * initialWeight);
        p2 = coordinate(last.x + sin(PI/2 + (PI/2-finalHeading)) * -1 * finalWeight,
                        last.y + cos(PI/2 + (PI/2-finalHeading)) * -1 * finalWeight);
        p3 = last;
    }

    coordinate solve(double t) {
        double omt = 1 - t;
        double x0 = p0.x, x1 = p1.x, x2 = p2.x, x3 = p3.x;
        double y0 = p0.y, y1 = p1.y, y2 = p2.y, y3 = p3.y;
        return coordinate(pow(omt,3) * x0 + 3 * pow(omt,2) * t * x1 +
                          3 * omt * pow(t,2) * x2 + pow(t,3) * x3,
                          pow(omt,3) * y0 + 3 * pow(omt,2) * t * y1 +
                          3 * omt * pow(t,2) * y2 + pow(t,3) * y3);
    }

    std::vector<coordinate> createLUT(double resolution) {
        std::vector<coordinate> points;
        for (int i = 0; i < resolution; i++) {
            points.push_back(solve(i/resolution));
        }
        return points;
    }

    double approximateLength(std::vector<coordinate> lut, double resolution) {
        double length = 0;
        for (int i = 0; i < resolution; i++) {
            coordinate first = lut[i];
            coordinate second = lut[i+1];
            length += distToPoint(first, second);
        }
        return length;
    }
};

/* ---------------- PID Constants ---------------- */
class util::pidConstants {
public:
    double p, i, d, tolerance, integralThreshold, maxIntegral, kv;

    pidConstants() {}
    pidConstants(double kp, double ki, double kd, double tol, double intThresh, double maxI)
        : p(kp), i(ki), d(kd), tolerance(tol), integralThreshold(intThresh), maxIntegral(maxI), kv(0) {}

    pidConstants(double kp, double ki, double kd, double tol, double intThresh, double maxI, double kv)
        : p(kp), i(ki), d(kd), tolerance(tol), integralThreshold(intThresh), maxIntegral(maxI), kv(kv) {}
};

/* ---------------- PID Controller ---------------- */
class util::pid {
private:
    double prevError, derivative;
    double integral = 0;
    util::pidConstants constants;

public:
    pid() {}
    pid(util::pidConstants cons, double error) : constants(cons), prevError(error) {}

    double out(double error) {
        if (std::fabs(error) < constants.tolerance) integral = 0;
        else if (std::fabs(error) < constants.integralThreshold) integral += error;
        if (integral > constants.maxIntegral) integral = constants.maxIntegral;

        derivative = error - prevError;
        prevError = error;

        return error * constants.p + integral * constants.i + derivative * constants.d;
    }

    void update(util::pidConstants cons) { constants = cons; }
};

/* ---------------- Moving Average ---------------- */
class util::movingAverage {
private:
    int size;
    double integral;
    std::vector<double> window;

public:
    movingAverage(int Size) : size(Size), integral(0) {
        for (int i = 0; i < size; i++) {
            window.push_back(0);
            integral += pow(i * 1.0/size, 2);
        }
    }
    
    void push(double val) {
        for (int i = 0; i < size-1; i++) window[i] = window[i+1];
        window[size - 1] = val;
    }

    double simpleAverage() {
        double avg = 0;
        for (int i = 0; i < size; i++) avg += window[i];
        return avg / size;
    }

    double expAverage() {
        double avg = 0;
        for (int i = 1; i != size; i++) avg += window[i] * pow(i * 1.0/size, 2);
        return avg / integral;
    }
};

/* ---------------- Time Range ---------------- */
class util::timeRange {
private:
    int start, end;

public:
    timeRange(int s, int e) : start(s), end(e) {}

    bool inRange(int time) { return (time >= start && time <= end); }
    int getStart() { return start; }
};

/* ---------------- Controller Wrapper ---------------- */
class util::controller {
private:
    pros::Controller* cont;
    double leftCurve, rightCurve;

public:
    controller(pros::Controller& c) : cont(&c), leftCurve(0), rightCurve(0) {}

    enum driveMode { arcade, tank };

    int select(int num, std::vector<std::string> names) {
        int curr = 0;
        cont->clear();
        while (true) {
            if (cont->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                curr = (curr + 1) % num;
            }
            if (cont->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                curr = (curr == 0) ? num-1 : curr-1;
            }
            if (cont->get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
                pros::delay(200);
                return curr;
            }
            cont->print(0, 0, "%s         ", names[curr]);
            pros::delay(50);
        }
    }

    std::vector<bool> getAll(std::vector<pros::controller_digital_e_t> buttons) {
        std::vector<bool> out;
        for (auto button : buttons) {
            out.push_back(cont->get_digital(button));
            out.push_back(cont->get_digital_new_press(button));
        }
        return out;
    }

    double curve(double x, double scale) {
        return (scale != 0) ? (pow(2.718, (scale * (std::fabs(x) - 127)) / 1000) * x) : x;
    }

    std::vector<double> drive(int direction, controller::driveMode mode) {
        double lStick = curve(cont->get_analog(ANALOG_LEFT_Y) * direction, leftCurve);
        double rStick;
        switch (mode) {
            case arcade:
                rStick = curve(cont->get_analog(ANALOG_RIGHT_X), rightCurve);
                return { lStick + rStick, lStick - rStick };
            case tank:
                rStick = curve(cont->get_analog(ANALOG_RIGHT_Y), rightCurve);
                return { lStick, rStick };
        }
        return {0,0}; // fallback
    }

    void setCurves(double left, double right) {
        leftCurve = left;
        rightCurve = right;
    }
};

/* ---------------- Math Utilities ---------------- */
inline double util::dtr(double input) { return PI * input / 180; }
inline double util::rtd(double input) { return input * 180 / PI; }

inline int util::dirToSpin(double target,double currHeading) {
    double d = (target - currHeading);
    double diff = d < 0 ? d + 360 : d;
    return (diff > 180 ? 1 : -1);
}

inline double util::minError(double target, double current) {
    double b = std::max(target,current);
    double s = std::min(target,current);
    double diff = b - s;
    return (diff <= 180 ? diff : (360-b) + s);
}

inline double util::distToPoint(util::coordinate p1, util::coordinate p2) {
    return sqrt(pow((p2.x-p1.x),2) + pow((p2.y-p1.y),2));
}

inline double util::mod(double a, double b) {
    return fmod(360-std::abs(a), b);
}

inline double util::absoluteAngleToPoint(util::coordinate pos, util::coordinate point) {
    double t;
    try {
        t = atan2(point.x - pos.x, point.y - pos.y);
    } catch(...) {
        t = PI/2;
    }
    t = util::rtd(t);
    t = -t;
    return (t >= 0 ? t : 180 + 180 + t);
}

inline double util::imuToRad(double heading) {
    return (heading < 180) ? dtr(heading) : dtr(-(heading - 180));
}

inline double util::sign(double a) { return (a > 0 ? 1 : -1); }

#endif
