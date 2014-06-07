/**
 * @file kinematics.h
 * @author Can Erdogan
 * @date Oct 28, 2013
 * @brief This file contains the kinematic definitions and workspace control functions.
 */

#pragma once

#include <gtsam/base/LieScalar.h>
#include <gtsam/base/LieVector.h>
#include <dynamics/SkeletonDynamics.h>
#include <simulation/World.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>

extern std::vector <int> lower_dofs;
extern std::vector <int> lowerLeft_dofs;
extern std::vector <int> lowerRight_dofs;
extern std::vector <int> left_dofs;
extern std::vector <int> right_dofs;
extern std::vector <int> jacobIndices;

extern simulation::World* mWorld;
extern dynamics::SkeletonDynamics* krang;												///< The robot model in dart

void endEffectorJacobian(const Eigen::VectorXd& qb, const Eigen::VectorXd& qa, bool right, 
		Eigen::MatrixXd& J, Eigen::Matrix4d& T);

void computeArmCOM (dynamics::SkeletonDynamics* krang, const Eigen::Matrix4d& wTb, 
		const Eigen::VectorXd& qa, bool right, Eigen::Vector3d& com);

void computeLowerCOM (dynamics::SkeletonDynamics* krang, double imu, double waist, 
		Eigen::Vector3d& com, Eigen::Matrix4d& T);

void computeCOM (dynamics::SkeletonDynamics* krang, double imu, double waist, const Eigen::VectorXd& larm,
		const Eigen::VectorXd& rarm, Eigen::Vector3d& com);

/// Attempts to approach the goal workspace location
void approachGoal (dynamics::SkeletonDynamics* robot, 
		const Eigen::VectorXd& goal, size_t kNumIters, bool right, Eigen::VectorXd& error, Eigen::VectorXd& finalq);
void approachGoal2 (dynamics::SkeletonDynamics* robot, 
		const Eigen::VectorXd& goal, size_t kNumIters, bool right, Eigen::VectorXd& error, Eigen::VectorXd& finalq);

/// Given a workspace velocity, returns the joint space velocity
Eigen::VectorXd workToJointVelocity (kinematics::BodyNode* eeNode, const Eigen::VectorXd& xdot, bool right);

/// Returns the workspace velocity to get the end-effector to the desired goal transformation
/// If already there, returns true.
bool workVelocity (kinematics::BodyNode* eeNode, const Eigen::VectorXd& goal, 
		Eigen::VectorXd& xdot, Eigen::VectorXd& error, bool print = false);

bool workVelocity (const Eigen::Matrix4d& eeTransform, const Eigen::VectorXd& goal, 
		Eigen::VectorXd& xdot, Eigen::VectorXd& error, bool print = false);

void endEffectorInBracket (const Eigen::VectorXd& qa, bool right, 
		Eigen::Matrix4d& T);

/// Returns the bracket transform
void endEffectorTransform (const Eigen::VectorXd& qb, const Eigen::VectorXd& qa, bool right, 
		Eigen::Matrix4d& T);

/// Returns the 3x2 jacobian that shows the effect of the imu and waist angles
void endEffectorJacobianBase (const Eigen::VectorXd& qb, const Eigen::VectorXd& qa, bool right, 
		Eigen::MatrixXd& J);

/// Returns the bracket transform
void bracketTransform (const Eigen::VectorXd& conf, Eigen::Matrix4d& T);

/// Returns the 3x5 jacobian that shows the effect of the base dofs on the bracket velocity
void bracketJacobian (const Eigen::VectorXd& conf, Eigen::MatrixXd& J);

/* ********************************************************************************************* */
inline void createTransform (const gtsam::LieVector& t, Eigen::Matrix4d& T) {
	Eigen::VectorXd tv = t.vector();
	double x = tv(0), y = tv(1), z = tv(2), phi = tv(3), th = tv(4), psi = tv(5);
	double cphi = cos(phi), sphi = sin(phi);
	double cth = cos(th), sth = sin(th);
	double cpsi = cos(psi), spsi = sin(psi);
	T << cphi * cth, (-sphi * cpsi + cphi * sth * spsi), (sphi * spsi + cphi * sth * cpsi), x,
     sphi * cth, (cphi * cpsi + sphi * sth * spsi), (-cphi * spsi + sphi * sth * cpsi), y,
     -sth, cth * spsi, cth * cpsi, z, 0, 0, 0, 1;
}
/* ********************************************************************************************* */
