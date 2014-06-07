/**
 * @file motion.cpp
 * @author Can Erdogan
 * @date May 29, 2014
 * @brief Contains the motion planning code.
 */

#include "motion.h"
#include <dart/math/UtilsRotation.h>

ManipData* manipData = NULL;

/* ********************************************************************************************** */
bool ManipRRT::checkCollisions(const Eigen::VectorXd &q) {

	// Set the robot configuration
  robot->setConfig(dofs, q);

	// Set the manipulated object configuration if one exists
	if(manipData != NULL) {

		// Get the object frame in the world frame
		const char* name = manipData->right ? "rGripper" : "lGripper";
		Eigen::Matrix4d wTh = robot->getNode(name)->getWorldTransform();
		Eigen::Matrix4d wTo = wTh * manipData->hTo;

		// Set the object dofs
		Eigen::VectorXd vals (6);
		vals.block<3,1>(0,0) = wTo.topRightCorner<3,1>();
		Eigen::Matrix3d R = wTo.topLeftCorner<3,3>();
		vals.block<3,1>(3,0) = math::matrixToEuler(R, math::XYZ);
		double temp = vals(5); vals(5) = vals(3); vals(3) = temp;
		world->getSkeleton(manipData->objName)->setPose(vals);
	}

  return world->checkCollision();
}

/* ********************************************************************************************** */
//bool ManipShortener::checkCollisions(const Eigen::VectorXd &q) 
