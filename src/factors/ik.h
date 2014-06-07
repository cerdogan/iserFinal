/**
 * @file ik.h
 * @author Can Erdogan
 * @date May 03, 2013
 * @brief Handles inverse-kinematics operation for a 7-dof arm with Krang's dimensions.
 */

#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <dynamics/SkeletonDynamics.h>
#include <simulation/World.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>
#include "geometry.h"

using namespace dynamics;
using namespace simulation;

typedef Eigen::Matrix <double, 7, 1> Vector7d;

extern bool ignoreIKDist;

/// Makes the small 1e-17 values in a matrix zero for printing
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat);

#define RAD2DEG(x) ((x) / M_PI * 180.0)
#define DEG2RAD(x) ((x) / 180.0 * M_PI)
#define pc(a) std::cout << #a << ": " << (a) << "\n" << std::endl
#define pm(a) std::cout << #a << ":\n " << fix((a).matrix()) << "\n" << std::endl
#define pmr(a) std::cout << #a << ":\n " << fix((a)) << "\n" << std::endl

// The link lengths
static const double theta1Offset = 0.1838;		

static const double L1 = .5074;				///> Distance between wheel axes and the waist module
static const double L2 = .1098;				///> Distance between the waist module and the spine
static const double L3 = .6694;				///> Dist. btw torso frame's origin and the bases of the arms
static const double L4 = .2856;				///> Arm link length between modules 0 and 2
static const double L5 = .3278;				///> Arm link length between modules 2 and 4
static const double L6 = .2763;				///> Arm link length between modules 4 and 6
static const double L7 = .2220;				///> Arm link length between modules 6 and end-effector
static const double L8_Schunk = 0.1750;	///> The length from f/t to middle of schunk gripper metals

/// The A matrix is the 4x4 transformation that represents the proximal elbow frame in the
/// distal shoulder frame. In Krang, this represents the 6th frame in the 5th frame.
static const Eigen::Matrix4d A = (Eigen::MatrixXd(4,4) << 1,0,0,0,0,0,1,0,0,-1,0,-L5,0,0,0,1).finished();

/// The B matrix is the 4x4 transformation that represents the proximal wrist frame in the
/// distal elbow frame. In Krang, this represents the 7th frame in the 'rotated' 6th frame.
static const Eigen::Matrix4d B = (Eigen::MatrixXd(4,4) << 1,0,0,0,0,0,-1,L6,0,1,0,0,0,0,0,1).finished();

/// Returns the 6d vector from 4d matrix
inline Eigen::Matrix<double, 6, 1> matToVec (const Eigen::Matrix4d& M) {
	Eigen::Matrix3d rot = M.topLeftCorner<3,3>();
	Eigen::Vector3d rotv = math::matrixToEuler(rot, math::XYZ) / M_PI * 180.0;
	Eigen::Matrix<double, 6, 1> res;
	res << M.topRightCorner<3,1>(), rotv;	
	return res;
}

/// Returns the angles of a rotation matrix with respect to three axes n1, n2 and n2 such that
/// n1 \perpto n2 and n2 \perpto n3. 
void anglesFromRotationMatrix(double &theta1, double &theta2, double &theta3, const Eigen::Vector3d &n1, 
		const Eigen::Vector3d &n2, const Eigen::Vector3d &n3, const Eigen::Matrix3d &A);

/// Computes a transformation given the DH parameters
Eigen::Matrix4d dh (double a, double alpha, double d, double theta);

/// Implements Sections 3.2.1 and 3.2.2
void getElbowPosAngle (const Eigen::Vector3d& relGoalLoc, double Lsw, double phi, Eigen::Vector3d& elbow, 
		double& theta4);

/// Returns the transformation between the proximal and distal shoulder frames (3.2.3)
void getT1 (const Eigen::Transform<double, 3, Eigen::Affine>& relGoal, const Eigen::Transform<double, 3, Eigen::Affine>& Ty,
		const Eigen::Vector3d& elbow, Eigen::Transform<double, 3, Eigen::Affine>& T1);

bool ik (const Eigen::Transform<double, 3, Eigen::Affine>& relGoal, double phi, Eigen::Matrix <double, 7, 1>& theta);

/// Computes the inverse-kinematics for the wrist-elbow-shoulder case. SingleArmIK calls this.
bool singleArmIK (const Eigen::VectorXd& base_conf, const Eigen::Matrix4d& Twee, bool rightArm, Vector7d& th,
		bool minimizeColl = false);

/// Returns the goal frame of the wrist in the shoulder frame
void getWristInShoulder (const Eigen::Matrix4d& Twb_, const Eigen::Matrix4d& Twee, bool rightArm, 
    Eigen::Matrix4d& TswM);

/// Given the goal configuration in the bracket frame, returns the goal location of the wrist
/// in the shoulder frame.
void getRelativeGoal(const Eigen::Transform<double, 3, Eigen::Affine>& goal, Eigen::Transform<double, 3, Eigen::Affine>& rel);

/// Returns the arm configuration for factors and display
bool getArmConf (Part* part, size_t faceId, const Eigen::VectorXd& tL, const Eigen::VectorXd& c, 
		const Eigen::VectorXd& qb, bool right, Eigen::VectorXd& qa);

/// Returns the arm configuration for factors and display (without barycentric coordinates)
bool getArmConfFull (Part* part, size_t faceId, const Eigen::VectorXd& tL, const Eigen::VectorXd& c, 
		const Eigen::VectorXd& qb, bool right, Eigen::VectorXd& qa);
