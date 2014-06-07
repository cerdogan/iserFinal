/**
 * @file motion.h
 * @author Can Erdogan
 * @date May 29, 2014
 * @brief Contains the motion planning code.
 */

#include <list>

#include <Eigen/Dense>
#include <flann/flann.hpp>

#include <kinematics/BodyNode.h>
#include <simulation/World.h>
#include <planning/RRT.h>
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>
#include <math/UtilsRotation.h>

struct ManipData {
	char objName [256];
	bool right; 
	Eigen::Matrix4d hTo;		//< object frame in the hand frame
};

extern ManipData* manipData;

/* ********************************************************************************************* */
/// Ensures that an object that's held is accounted for in collision checking of the sampling.
class ManipRRT : public planning::RRT {
public:

  ManipRRT(simulation::World* world, dynamics::SkeletonDynamics* robot, 
      const std::vector<int> &dofs, const Eigen::VectorXd &root, double stepSize = 0.02) :
    planning::RRT(world, robot, dofs, root, stepSize) {}
  
  ManipRRT(simulation::World* world, dynamics::SkeletonDynamics* robot, const std::vector<int> &dofs, 
      const std::vector<Eigen::VectorXd> &roots, double stepSize = 0.02) :
    planning::RRT(world, robot, dofs, roots, stepSize) {}

  /// Check for collisions
	bool checkCollisions(const Eigen::VectorXd &q);
};

/* ********************************************************************************************* */
/// Ensures that an object that's held is accounted for in collision checking of the shortening.
class ManipShortener: public planning::PathShortener {
public:

  ManipShortener() : PathShortener () {}

  ManipShortener(simulation::World* world, dynamics::SkeletonDynamics* robot, 
			const std::vector<int> &dofs, double stepSize = 0.1) : 
		PathShortener(world, robot, dofs, stepSize) {}

  /// Check for collisions
	inline bool collisionChecks(const Eigen::VectorXd &q) {

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
};

