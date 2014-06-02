/**
 * @file controllers.h
 * @author Can Erdogan
 * @date Jun 02, 2014
 * @brief Controller for the base.
 */

#include "helpers.h"

extern Eigen::Vector3d locoGoal;

/// Given a location (x,y,th) in the initial robot frame from start up
bool locomotion (Mode mode); 

