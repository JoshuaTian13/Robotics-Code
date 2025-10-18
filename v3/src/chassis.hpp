#ifndef __CHASSIS__
#define __CHASSIS__

// --- chassis specific macros ---
#define DL 252.5
#define DR -259.7
#define MAXSPEED 0.647953484803
#define MAXACCEL_FORWARD 0.647953484803 / 18
#define MAXDECEL_FORWARD 0.647953484803 / 24
#define MAXACCEL_BACKWARD 0.647953484803 / 21
#define MAXDECEL_BACKWARD 0.647953484803 / 48
#define INCHESTOROTATIONS 46.29961981
#define IP1MSTOMRPMINUTE 196.0017239

#include "global.hpp"

namespace chas
{
  // High-level chassis API
  class follower;
  void spinTo(double target, double timeout, util::pidConstants constants);
  void drive(double target, double timeout, double max, util::pidConstants constants); // target = encoder units
  void driveAngle(double target, double heading, double timeout, util::pidConstants lCons, util::pidConstants acons);
  void odomDrive(double distance, double timeout, double tolerance);
  std::vector<double> moveToVel(util::coordinate target, double lkp, double rkp, double rotationBias);
  void moveTo(util::coordinate target, double timeout, util::pidConstants lConstants, util::pidConstants rConstants, double rotationBias, double rotationScale, double rotationCut);
  void moveToPose(util::bezier curve, double timeout, double lkp, double rkp, double rotationBias);
  void timedSpin(double target, double speed, double timeout);
  void velsUntilHeading(double rvolt, double lvolt, double heading, double tolerance, double timeout);
  void arcTurn(double theta, double radius, double timeout, int dir, util::pidConstants cons);
  std::vector<double> trapezoidalProfile(double dist, double maxSpeed, double accel);
  std::vector<double> asymTrapezoidalProfile(double dist, double maxSpeed, double accel, double decel);
  void profiledDrive(double target, int endDelay);
}

/**
 * @brief Profile follower: feeds reference velocity profile into PID + kv.
 */
class chas::follower
{
  private:
    std::vector<double> profile;
    util::pid controller;
    util::pidConstants constants;
    double sf;
    double kv;
    double negbias;
    util::timer timer;

  public:
    follower(std::vector<double> profile, util::pidConstants constants, double kv, double sf, double negbias = 30)
      : profile(profile), kv(kv), constants(constants), sf(sf), negbias(negbias)
    {
      controller = util::pid(constants, 0);
    }

    double out(double curr)
    {
      int t = timer.time() / 10;
      double rpm = profile[t] * sf;
      double error = curr - rpm;
      if (error > 0)
      {
        error += error/(1);
      }
      return rpm * kv + controller.out(-error);
    }

    void start()
    {
      timer.start();
    }
};

/**
 * @brief Turn to absolute heading using PID.
 */
void chas::spinTo(double target, double timeout, util::pidConstants constants /*= util::pidConstants(3.7, 1.3, 26, 0.05, 2.4, 20)*/)
{
  util::timer timeoutTimer;
  util::pid pid(constants, 0);
  double currHeading;
  double vel;

  while (timeoutTimer.time() <= timeout)
  {
    currHeading = robot::imu.degHeading();
    vel = util::dirToSpin(target, currHeading) * pid.out(util::minError(target, currHeading));
    robot::chass.spinDiffy(-vel, vel);
    glb::controller.print(0,0,"%f", util::minError(target, currHeading) * util::dirToSpin(target, currHeading));
    pros::delay(10);
  }
  robot::chass.stop('b');
}

/**
 * @brief Drive a target encoder distance using PID.
 */
void chas::drive(double target, double timeout, double max /*=127*/, util::pidConstants constants /*= util::pidConstants(0.3,0.2,2.4,5,30,1000)*/)
{
  util::timer timeoutTimer;
  util::pid pid(constants, 0);
  robot::chass.reset();
  while (timeoutTimer.time() <= timeout)
  {
    robot::chass.spin(pid.out(target - robot::chass.getRotation()));
    glb::controller.print(0,0,"%f", target - robot::chass.getRotation());
    pros::delay(10);
  }
  robot::chass.stop('b');
}

/**
 * @brief Drive a distance while holding/approaching a heading using two PIDs.
 */
void chas::driveAngle(double target, double heading, double timeout,
                      util::pidConstants lCons /*= {0.3,0.2,2.4,5,30,1000}*/,
                      util::pidConstants acons /*= {4,0.7,4,0,190,20}*/)
{
  util::timer timer;

  double currHeading = robot::imu.degHeading();
  double rot;
  double error;
  double vl;
  double va;
  double sgn = target > 0 ? 1 : -1;

  int dir;

  util::pid linearController(lCons, 0);
  util::pid angularController(acons, target);

  robot::chass.reset();

  while (timer.time() <= timeout)
  {
    error = util::minError(heading, currHeading);
    if (error < 0.5)
    {
      acons.p = 0;
      angularController.update(acons);
    }

    currHeading = robot::imu.degHeading();
    rot = robot::chass.getRotation();

    va = angularController.out(error);
    vl = linearController.out(target - rot);
    dir = -util::dirToSpin(heading, currHeading);

    if (vl + std::abs(va) > 127)
    {
      vl = 127 - std::abs(va);
    }

    robot::chass.spinDiffy(vl + (dir * va * sgn),  vl - (dir * va * sgn));

    pros::delay(10);
    glb::controller.print(0, 0, "%f", util::minError(heading, currHeading));
  }

  robot::chass.stop('b');
}

/**
 * @brief Drive in global field frame using odometry toward a forward target.
 */
void chas::odomDrive(double distance, double timeout, double tolerance)
{
  util::timer endTimer;
  util::timer timeoutTimer;
  timeoutTimer.start();

  // PID constants
  double kP = 2.1;
  double kI = 0;
  double kD = 0.1;
  double endTime = 1;

  // General vars
  double dist = -distance;
  double heading = robot::imu.radHeading();
  util::coordinate target(sin(2*PI-heading) * dist + glb::pos.x,
                          cos(2*PI-heading) * dist + glb::pos.y);
  double prevRotation;
  double error;
  double prevError;
  bool end = false;

  // Integrator
  double integral = 0;
  double integralThreshold = 30;

  // Derivative
  double derivative;

  // PID loop
  while (!end)
  {
    // P
    error = dist - (dist - util::distToPoint(glb::pos, target));

    // I
    integral = error <= tolerance ? 0 : fabs(error) < integralThreshold ? integral += error : integral;

    // D
    derivative = error - prevError;
    prevError = error;

    // End conditions
    if (error >= tolerance) { endTimer.start(); }

    end = endTimer.time() >= endTime ? true
         : timeoutTimer.time() >= timeout ? true
         : false;

    // Drive
    double vel = (kP*error + kI*integral + kD*derivative);
    robot::chass.spin(vel);

    pros::delay(10);
  }
  robot::chass.stop('b');
}

/**
 * @brief Compute left/right velocities to move toward a coordinate with rotation biasing.
 */
std::vector<double> chas::moveToVel(util::coordinate target, double lkp, double rkp, double rotationBias)
{
  double linearError = distToPoint(glb::pos, target);
  double linearVel = linearError * lkp;

  double currHeading =  robot::imu.degHeading(); // 0-360
  double targetHeading = absoluteAngleToPoint(glb::pos, target); // -180..180
  targetHeading = targetHeading >= 0 ? targetHeading :  180 + fabs(targetHeading);

  int dir = -util::dirToSpin(targetHeading, currHeading);

  double rotationError = util::minError(targetHeading, currHeading);
  double rotationVel = rotationError * rkp * dir;

  // Reduce linear speed proportional to rotation error
  double lVel = (linearVel - (fabs(rotationVel) * rotationBias)) - rotationVel;
  double rVel = (linearVel - (fabs(rotationVel) * rotationBias)) + rotationVel;

  return std::vector<double> {lVel, rVel};
}

/**
 * @brief Move to a coordinate with coupled linear/angular control and adaptive angular P.
 */
void chas::moveTo(util::coordinate target, double timeout,
                  util::pidConstants lConstants, util::pidConstants rConstants,
                  double rotationBias, double rotationScale, double rotationCut)
{
  util::timer timeoutTimer;
  double rotationVel, linearVel;
  double linearError = distToPoint(glb::pos, target);
  double initError = linearError;
  double currHeading =  robot::imu.degHeading();
  double targetHeading = absoluteAngleToPoint(glb::pos, target);
  double rotationError = util::minError(targetHeading, currHeading);

  // Controllers
  util::pid linearController(lConstants, linearError);
  util::pid rotationController(rConstants, rotationError);

  // Scale angular p based on distance
  double slope = (rConstants.p) / (linearError - rotationCut);
  double initP = rConstants.p;

  while (timeoutTimer.time() < timeout)
  {
    linearError = distToPoint(glb::pos, target);
    currHeading =  robot::imu.degHeading();

    targetHeading = absoluteAngleToPoint(glb::pos, target);
    rotationError = util::minError(targetHeading, currHeading);

    rConstants.p = slope * (linearError - initError) + initP;
    rConstants.p = rConstants.p < 0 ? 0 : rConstants.p;
    rotationController.update(rConstants);

    int dir = -util::dirToSpin(targetHeading, currHeading);
    double cre = cos(rotationError <= 90 ? util::dtr(rotationError) : PI/2);

    rotationVel = dir * rotationController.out(rotationError);
    linearVel = cre * linearController.out(linearError);

    double rVel = (linearVel - (fabs(rotationVel) * rotationBias)) + rotationVel;
    double lVel = (linearVel - (fabs(rotationVel) * rotationBias)) - rotationVel;

    robot::chass.spinDiffy(rVel, lVel);
  }

  robot::chass.stop('b');
}

/**
 * @brief Follow a Bezier curve by stepping targets along a LUT.
 */
void chas::moveToPose(util::bezier curve, double timeout, double lkp, double rkp, double rotationBias)
{
  int resolution = 100;

  // Precompute LUT
  std::vector<util::coordinate> lut = curve.createLUT(resolution);

  double t;
  double distTraveled = 0;
  double ratioTraveled;
  double curveLength = curve.approximateLength(lut, resolution);

  util::coordinate prevPos = glb::pos;
  util::coordinate targetPos;

  while (1)
  {
    // Approximate traveled distance
    distTraveled += util::distToPoint(prevPos, glb::pos);
    prevPos = glb::pos;

    // Find next point in LUT
    ratioTraveled = distTraveled/curveLength;
    t = std::ceil(ratioTraveled * resolution);
    targetPos = lut[t-1];

    std::vector<double> velocities = moveToVel(targetPos, 0.1, 0.1, 0.1);
    robot::chass.spinDiffy(velocities[1], velocities[0]);

    if (t == (int)lut.size()) { break; }
  }
}

/**
 * @brief Spin at a given speed until heading crosses target or timeout.
 */
void chas::timedSpin(double target, double speed, double timeout)
{
  util::timer timeoutTimer;
  bool end = false;

  double currHeading = robot::imu.degHeading();
  int initDir = -util::dirToSpin(target, currHeading);

  while (!end)
  {
    currHeading = robot::imu.degHeading();
    int dir = -util::dirToSpin(target, currHeading);

    double error = util::minError(target, currHeading);

    if (initDir != dir) { end = true; }
    end = timeoutTimer.time() >= timeout ? true : end;

    robot::chass.spinDiffy(dir * speed, -speed * dir);
  }

  robot::chass.stop('b');
}

/**
 * @brief Apply fixed voltages until heading within tolerance or timeout.
 */
void chas::velsUntilHeading(double rvolt, double lvolt, double heading, double tolerance, double timeout)
{
  util::timer timeoutTimer;

  while (true)
  {
    if (util::minError(heading, robot::imu.degHeading()) < tolerance || timeoutTimer.time() >= timeout)
    {
      break;
    }

    robot::chass.spinDiffy(rvolt, lvolt);
  }
}

/**
 * @brief Arc turn by controlling wheel path lengths to reach a target angle.
 */
void chas::arcTurn(double theta, double radius, double timeout, int dir, util::pidConstants cons)
{
  util::timer timer;
  double curr;
  double rvel;
  double lvel;
  double vel;
  double ratio;

  double sl = theta * (radius + DL);
  double sr = theta * (radius + DR);

  theta = util::rtd(theta);
  ratio = sl / sr;
  curr = glb::imu.get_heading();
  util::pid controller(cons, 0);

  while (timer.time() < timeout)
  {
    glb::controller.print(0,0,"%f", util::minError(theta, curr));
    curr = glb::imu.get_heading();
    vel = controller.out(util::minError(theta, curr)) * util::dirToSpin(theta, curr);
    vel = std::abs(vel) >= 127 ? (127 * util::sign(vel)) : vel;
    rvel = (2 * vel) / (ratio + 1);
    lvel = ratio * rvel;

    if (util::sign(dir) == 1)
    {
      robot::chass.spinDiffy(rvel, lvel);
    }
    else
    {
      robot::chass.spinDiffy(-lvel, -rvel);
    }

    pros::delay(10);
  }
  robot::chass.stop('b');
}

/**
 * @brief Symmetric trapezoidal velocity profile (inches/10ms).
 */
std::vector<double> chas::trapezoidalProfile(double dist, double maxSpeed /*= MAXSPEED*/, double accel /*= MAXACCEL_FORWARD*/)
{
  double max = std::min(std::sqrt(dist * accel), maxSpeed);
  double accelTime = max / accel;
  double accelDist = (accel / 2) * std::pow(accelTime, 2);
  double coastDist = dist - (2 * accelDist);
  double coastTime = coastDist / max;
  double totalTime = 2 * accelTime + coastTime;
  double vel = 0;
  double diff;
  std::vector<double> profile;

  for (int i = 0; i < std::ceil(totalTime); i++)
  {
    if (i < std::floor(accelTime))
    {
      profile.push_back(vel);
      vel += accel;
    }
    else if (i < coastTime + accelTime)
    {
      profile.push_back(max);
    }
    else
    {
      profile.push_back(vel);
      vel -= accel;
    }
  }
  // second pass adjustment
  int size = profile.size();
  double traveled = std::reduce(profile.begin(), profile.end());
  diff = traveled - dist;
  double adj = diff / (size / 5);
  int num = std::ceil(size / 5);
  for (int i = 0; i < num; i++)
  {
    if (profile[size - i - 1] > adj)
    {
      profile[size - i - 1] -= adj;
    }
    else
    {
      num += 1;
    }
  }

  return profile;
}

/**
 * @brief Asymmetric trapezoidal profile with separate accel/decel.
 */
std::vector<double> chas::asymTrapezoidalProfile(double dist, double maxSpeed /*= MAXSPEED*/,
                                                 double accel /*= MAXACCEL_FORWARD*/,
                                                 double decel /*= MAXDECEL_FORWARD*/)
{
  double max = std::min(std::sqrt((2 * accel * decel * dist) / accel + decel), maxSpeed);
  double accelTime = max / accel;
  double decelTime = max / decel;
  double coastDist = (dist / max) - (max / (2 * accel)) - (max / (2 * decel));
  double coastTime = coastDist / max;
  double totalTime = accelTime + decelTime + coastTime;
  double vel = 0;
  std::vector<double> profile;

  for (int i = 0; i < std::ceil(totalTime); i++)
  {
    if (i < std::floor(accelTime))
    {
      profile.push_back(vel);
      vel += accel;
    }
    else if (i < coastTime + accelTime)
    {
      profile.push_back(max);
    }
    else
    {
      profile.push_back(vel);
      vel -= decel;
    }
  }
  return profile;
}

/**
 * @brief Drive using an asymmetric profile; converts profile to rpm then voltage.
 */
void chas::profiledDrive(double target, int endDelay /*= 500*/)
{
  // kv: rpm -> voltage
  // sf: in/ms -> rpm
  int sign = util::sign(target);
  target = fabs(target);
  double targetRot = target * INCHESTOROTATIONS;

  std::vector<double> profile;
  if (sign > 0) profile = chas::asymTrapezoidalProfile(target);
  else          profile = chas::asymTrapezoidalProfile(target, MAXSPEED, MAXACCEL_BACKWARD, MAXDECEL_BACKWARD);

  robot::chass.reset();
  robot::chass.reset();

  for (int i = 0; i < (int)profile.size(); i++)
  {
    robot::chass.spin(profile[i] * IP1MSTOMRPMINUTE * sign);
    glb::controller.print(0,0,"%f", profile[i] * IP1MSTOMRPMINUTE * sign);
    pros::delay(10);
  }
  robot::chass.stop('b');
  pros::delay(endDelay);
}

#endif
