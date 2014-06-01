/**
 * @file controllers.cpp
 * @author Can Erdogan
 * @date Feb 18, 2014
 * @brief Contains the controller information for the balancing acts.
 */

#pragma once

#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <kore.hpp>

#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>
#include <map>

using namespace Krang;

extern double extraUForw;
extern bool done;
extern bool firstDetect;
extern bool nextStep;
extern double extraUSpin;
extern bool dbg;
extern bool start;
extern bool startOver;
extern bool changePermitted;
extern bool withCinder;

/* ******************************************************************************************** */
extern somatic_d_t daemon_cx;				///< The context of the current daemon
extern Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
extern simulation::World* world;			///< the world representation in dart
extern dynamics::SkeletonDynamics* robot;			///< the robot representation in dart
extern ach_channel_t cinder_chan;

/* ******************************************************************************************** */
/// To represent the state machine which chooses Kinect and wheel actions
enum STEP {
	STEP_STANDUP = 0,
	STEP_TURN_1 = 1,
	STEP_TURN_DETECT_1 = 2,
	STEP_FORWARD = 3,
	STEP_FORWARD_DETECT = 4,
	STEP_TURN_2 = 5,
	STEP_TURN_DETECT_2 = 6,
	STEP_SIT = 7
};

/// To decide which control is in charge of the robot torques
enum CTRL_MODE {
	CTRL_BALANCE = 0,
	CTRL_TURN = 1,
	CTRL_FORWARD = 2,
	CTRL_SIT = 3
};

/// The specification of control mode for each step 
extern std::map <STEP, CTRL_MODE> stepCtrlModes;

/* ********************************************************************************************* */
// Functions

/// Exerts torques to the wheel in accordance with the desired control mode and state; returns
/// true if the error is within the bounds specified for the specific mode
bool controller (STEP step, Vector6d state, double dt, const Eigen::Vector3d& motionParams); 

/// Creates the velocity profile for the given distance and the accelerations
void createProfile(double dist, double maxVel, double acc, double dec, double& accTime, 
		double& decTime, double& cruiseTime);

/// Sets the reference position and velocities using the velocity profile
void getForwardReference(const Eigen::VectorXd& state0, double time, double acc, double dec, 
		double accTime, double decTime, double cruiseTime, double& refPos, double& refVel);

/// Get the mode input
void *kbhit(void *);

/// Returns the name of a step
const char* stepName (STEP step);

/// Returns the name of a control mode 
const char* ctrlModeName (CTRL_MODE ctrlMode);

/// Read file for gains
void readGains ();
