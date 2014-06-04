/**
 * @file helpers.h
 * @author Can Erdogan
 * @date Jun 02, 2014
 * @brief Common variables, functions, enums and etc.
 */

#include <kore.hpp>
#include <kore/ik.hpp>
#include <kore/safety.hpp>
#include <kore/util.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <somatic/msg.h>

#define SQ(x) ((x) * (x))

#pragma once

#define GRIP_ON 1
#include "simTab.h"

typedef Eigen::Matrix<double,6,1> Vector6d;
extern Vector6d state;					//< current state (x,x.,y,y.,th,th.)
extern Eigen::Vector3d locoGoal;
extern bool sending_commands;
extern dynamics::SkeletonDynamics* krang;
extern simulation::World* world;
extern Krang::Hardware* hw;     
extern somatic_d_t daemon_cx;   

extern Eigen::VectorXd smallGraspPose;

/// See todo for explanation of the enums
enum Mode {
	A1, A2, A3, A4, A5, A6, A7, A8, A9, A10,
	B1, B2, B3, B4, B5, B6, B7, 
	C1, C2, C3, C4, C5, C6, C7, C8,
	D1, D2, D3, D4, D5, D6, D7, D8
};


