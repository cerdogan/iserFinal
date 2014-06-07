/**
 * @file kinematics.h
 * @author Can Erdogan
 * @date Oct 28, 2013
 * @brief This file contains the kinematic definitions and workspace control functions.
 */

#include "kinematics.h"
#include <amino.h>
#include <gtsam/base/timing.h>
#include <math/UtilsMath.h>

using namespace std;
using namespace Eigen;

/* ******************************************************************************************** */
int lower_dofs_ [] = {0, 1, 3, 5, 8}; 
vector <int> lower_dofs (lower_dofs_, lower_dofs_ + sizeof(lower_dofs_) / sizeof(int));
int lld [] = {0, 1, 3, 5, 8, 11, 13, 15, 17, 19, 21, 23}; 
vector <int> lowerLeft_dofs (lld, lld + sizeof(lld) / sizeof(int));
int lrd [] = {0, 1, 3, 5, 8, 12, 14, 16, 18, 20, 22, 24}; 
vector <int> lowerRight_dofs (lrd, lrd + sizeof(lrd) / sizeof(int));
int ld [] = {11, 13, 15, 17, 19, 21, 23}; 
vector <int> left_dofs (ld, ld + sizeof(ld) / sizeof(int));
int rd [] = {12, 14, 16, 18, 20, 22, 24}; 
vector <int> right_dofs (rd, rd + sizeof(rd) / sizeof(int));
int jx [] = {0, 1, 3, 5, 6, 8, 9, 10, 11, 12, 13, 14};
vector <int> jacobIndices (jx, jx + sizeof(jx) / sizeof(int));

typedef Transform <double, 3, Affine> TT;
typedef AngleAxis <double> AA;

#define pv(x) {if(debug) cout << #x << ": " << (x).transpose() << "\n"; }
#define pmr(x) {cout << #x << ": \n" << (x) << "\n"; }

/* ********************************************************************************************* */
void endEffectorJacobian(const Eigen::VectorXd& qb, const Eigen::VectorXd& qa, bool right, 
		Eigen::MatrixXd& J, Eigen::Matrix4d& wTee) {

	Vector3d xAxis (1.0, 0.0, 0.0), yAxis (0.0, 1.0, 0.0), zAxis (0.0, 0.0, 1.0);

	// ================================================================================
	// Get the fixed ones

	static Matrix4d baTs1 = (MatrixXd(4,4) << 1,0,0,0.0260,0,1,0,0.4990,-0,0,1,-0.0910,0,0,0,1).finished();
	static Matrix4d sTb1 =  (MatrixXd(4,4) << 0,1,-0,0.2553,1,-0,0,0.6670,-0,-0,-1,0.1088,0,0,0,1).finished();
	static Matrix4d a1Tb2 = (MatrixXd(4,4) << -1,0,0,0,-0,-1,0,0,0,0,1,0,0,0,0,1).finished();
	static Matrix4d a2Tb3 = (MatrixXd(4,4) << -1,0,-0,0,-0,-1,-0,-0.3280,-0,-0,1,0,0,0,0,1).finished();
	static Matrix4d a3Tb4 = (MatrixXd(4,4) << -1,0,0,0,-0,-1,0,0,0,0,1,0,0,0,0,1).finished();
	static Matrix4d a4Tb5 = (MatrixXd(4,4) << -1,0,-0,0,-0,-1,-0,-0.2765,-0,-0,1,0,0,0,0,1).finished();
	static Matrix4d a5Tb6 = (MatrixXd(4,4) << -1,0,0,0,-0,-1,0,0,0,0,1,0,0,0,0,1).finished();
	static Matrix4d a6Tb7 = (MatrixXd(4,4) << -1,-0,0,0,-0,-0,-1,-0.2000,0,-1,0,0,0,0,0,1).finished();

	// ================================================================================
	// Get the variable ones

	Matrix4d meye = Matrix4d::Identity();
	Matrix4d wTba1 = meye; wTba1.topRightCorner<3,1>() = Vector3d(qb(0), qb(1), 0.27);
	Matrix4d wTba2 = TT(AA(qb(2), zAxis)).matrix();
	Matrix4d wTba3 = TT(AA(qb(3), xAxis)).matrix();
	Matrix4d baTs2 = TT(AA(-qb(4), xAxis)).matrix();
	Matrix4d b1Ta1 = TT(AA(-qa(0), yAxis)).matrix();
	Matrix4d b2Ta2 = TT(AA(-qa(1), xAxis)).matrix();
	Matrix4d b3Ta3 = TT(AA(-qa(2), yAxis)).matrix();
	Matrix4d b4Ta4 = TT(AA(-qa(3), xAxis)).matrix();
	Matrix4d b5Ta5 = TT(AA(-qa(4), yAxis)).matrix();
	Matrix4d b6Ta6 = TT(AA(-qa(5), xAxis)).matrix();
	Matrix4d b7Ta7 = TT(AA(-qa(6), zAxis)).matrix();

	// ================================================================================
	// Get the derivatives
	
	Matrix4d z = Matrix4d::Zero();
	Matrix4d Tdphi = z, Tdpsi = z, Tdqw = z, Tdq1 = z, Tdq2 = z, Tdq3 = z, Tdq4 = z, Tdq5 = z, Tdq6 = z, Tdq7 = z;
	Tdphi(0,0) = Tdphi(1,1) = -sin(qb(2)); Tdphi(0,1) = Tdphi(1,0) = -cos(qb(2)); Tdphi(1,0) = -Tdphi(1,0);
	Tdpsi(1,1) = Tdpsi(2,2) = -sin(qb(3)); Tdpsi(1,2) = Tdpsi(2,1) = -cos(qb(3)); Tdpsi(2,1) = -Tdpsi(2,1);
	Tdqw(1,1) = Tdqw(2,2) = -sin(qb(4)); Tdqw(1,2) = Tdqw(2,1) = cos(qb(4)); Tdqw(2,1) = -Tdqw(2,1);
	Tdq1(0,0) = Tdq1(2,2) = -sin(qa(0)); Tdq1(0,2) = Tdq1(2,0) = -cos(qa(0)); Tdq1(2,0) = -Tdq1(2,0);
	Tdq2(1,1) = Tdq2(2,2) = -sin(qa(1)); Tdq2(1,2) = Tdq2(2,1) = cos(qa(1)); Tdq2(2,1) = -Tdq2(2,1);
	Tdq3(0,0) = Tdq3(2,2) = -sin(qa(2)); Tdq3(0,2) = Tdq3(2,0) = -cos(qa(2)); Tdq3(2,0) = -Tdq3(2,0);
	Tdq4(1,1) = Tdq4(2,2) = -sin(qa(3)); Tdq4(1,2) = Tdq4(2,1) = cos(qa(3)); Tdq4(2,1) = -Tdq4(2,1);
	Tdq5(0,0) = Tdq5(2,2) = -sin(qa(4)); Tdq5(0,2) = Tdq5(2,0) = -cos(qa(4)); Tdq5(2,0) = -Tdq5(2,0);
	Tdq6(1,1) = Tdq6(2,2) = -sin(qa(5)); Tdq6(1,2) = Tdq6(2,1) = cos(qa(5)); Tdq6(2,1) = -Tdq6(2,1);
	Tdq7(0,0) = Tdq7(1,1) = -sin(qa(6)); Tdq7(0,1) = Tdq7(1,0) = cos(qa(6)); Tdq7(1,0) = -Tdq7(1,0);

	// ================================================================================
	// Compute the cascades

	Matrix4d Tb1 = wTba1;
	Matrix4d Tb2 = Tb1 * wTba2;
	Matrix4d Tb3 = Tb2 * wTba3 * baTs1;
	Matrix4d Tb4 = Tb3 * baTs2 * sTb1;
	Matrix4d Tb5 = Tb4 * b1Ta1 * a1Tb2;
	Matrix4d Tb6 = Tb5 * b2Ta2 * a2Tb3;
	Matrix4d Tb7 = Tb6 * b3Ta3 * a3Tb4;
	Matrix4d Tb8 = Tb7 * b4Ta4 * a4Tb5;
	Matrix4d Tb9 = Tb8 * b5Ta5 * a5Tb6;
	Matrix4d Tb10= Tb9 * b6Ta6 * a6Tb7;

	Matrix4d Ta10 = meye;
	Matrix4d Ta9 = a6Tb7 * b7Ta7 * Ta10;
	Matrix4d Ta8 = a5Tb6 * b6Ta6 * Ta9;
	Matrix4d Ta7 = a4Tb5 * b5Ta5 * Ta8;
	Matrix4d Ta6 = a3Tb4 * b4Ta4 * Ta7;
	Matrix4d Ta5 = a2Tb3 * b3Ta3 * Ta6;
	Matrix4d Ta4 = a1Tb2 * b2Ta2 * Ta5;
	Matrix4d Ta3 = sTb1 * b1Ta1 * Ta4;
	Matrix4d Ta2 = baTs1 * baTs2 * Ta3;
	Matrix4d Ta1 = meye * wTba3 * Ta2;

	wTee = Tb1 * wTba2 * Ta1;
	Matrix3d wReeT = wTee.topLeftCorner<3,3>().transpose();

	// ================================================================================
	// Compute the derivatives

	J = MatrixXd(6,12); //J.block<3,12>(3,0) = MatrixXd::Zero(3,12);
/*
	J.block<3,1>(0,2) = (Tb1 * (Tdphi * (Ta1.topRightCorner<4,1>()))).topRightCorner<3,1>();
	J.block<3,1>(0,3) = (Tb2 * (Tdpsi* (Ta2.topRightCorner<4,1>()))).topRightCorner<3,1>();
	J.block<3,1>(0,4) = (Tb3 * (Tdqw* (Ta3.topRightCorner<4,1>()))).topRightCorner<3,1>();
	J.block<3,1>(0,5) = (Tb4 * (Tdq1* (Ta4.topRightCorner<4,1>()))).topRightCorner<3,1>();
	J.block<3,1>(0,6) = (Tb5 * (Tdq2 * (Ta5.topRightCorner<4,1>()))).topRightCorner<3,1>();
	J.block<3,1>(0,7) = (Tb6 * (Tdq3 * (Ta6.topRightCorner<4,1>()))).topRightCorner<3,1>();
	J.block<3,1>(0,8) = (Tb7 * (Tdq4 * (Ta7.topRightCorner<4,1>()))).topRightCorner<3,1>();
	J.block<3,1>(0,9) = (Tb8 * (Tdq5 * (Ta8.topRightCorner<4,1>()))).topRightCorner<3,1>();
	J.block<3,1>(0,10) = (Tb9 * (Tdq6 * (Ta9.topRightCorner<4,1>()))).topRightCorner<3,1>();
	J.block<3,1>(0,11) = (Tb10* (Tdq7 * (Ta10.topRightCorner<4,1>()))).topRightCorner<3,1>();
*/

	Matrix4d bla = (Tb5 * (Tdq2 * Ta5));
	Matrix4d wTdphi = (Tb1 * (Tdphi* Ta1));
	Matrix4d wTdpsi = (Tb2 * (Tdpsi* Ta2));
	Matrix4d wTdqw  = (Tb3 * (Tdqw * Ta3));
	Matrix4d wTdq1  = (Tb4 * (Tdq1 * Ta4));
	Matrix4d wTdq2  = (Tb5 * (Tdq2 * Ta5));
	Matrix4d wTdq3  = (Tb6 * (Tdq3 * Ta6));
	Matrix4d wTdq4  = (Tb7 * (Tdq4 * Ta7));
	Matrix4d wTdq5  = (Tb8 * (Tdq5 * Ta8));
	Matrix4d wTdq6  = (Tb9 * (Tdq6 * Ta9));
	Matrix4d wTdq7  = (Tb10* (Tdq7 * Ta10));

	// Get the linears
	J.block<3,1>(0,0) = Vector3d(1,0,0); 
	J.block<3,1>(0,1) = Vector3d(0,1,0);
	J.block<3,1>(0,2) = wTdphi.topRightCorner<3,1>();
	J.block<3,1>(0,3) = wTdpsi.topRightCorner<3,1>();
	J.block<3,1>(0,4) = wTdqw.topRightCorner<3,1>();
	J.block<3,1>(0,5) = wTdq1.topRightCorner<3,1>();
	J.block<3,1>(0,6) = wTdq2.topRightCorner<3,1>();
	J.block<3,1>(0,7) = wTdq3.topRightCorner<3,1>();
	J.block<3,1>(0,8) = wTdq4.topRightCorner<3,1>();
	J.block<3,1>(0,9) = wTdq5.topRightCorner<3,1>();
	J.block<3,1>(0,10) = wTdq6.topRightCorner<3,1>();
	J.block<3,1>(0,11) = wTdq7.topRightCorner<3,1>();

	// Get the angulars
	J.block<3,1>(3,0) = Vector3d(0,0,0); 
	J.block<3,1>(3,1) = Vector3d(0,0,0);
	J.block<3,1>(3,2) = math::fromSkewSymmetric(wTdphi.topLeftCorner<3,3>() * wReeT);
	J.block<3,1>(3,3) = math::fromSkewSymmetric(wTdpsi.topLeftCorner<3,3>() * wReeT);
	J.block<3,1>(3,4) = math::fromSkewSymmetric(wTdqw.topLeftCorner<3,3>() * wReeT);
	J.block<3,1>(3,5) = math::fromSkewSymmetric(wTdq1.topLeftCorner<3,3>() * wReeT);
	J.block<3,1>(3,6) = math::fromSkewSymmetric(wTdq2.topLeftCorner<3,3>() * wReeT);
	J.block<3,1>(3,7) = math::fromSkewSymmetric(wTdq3.topLeftCorner<3,3>() * wReeT);
	J.block<3,1>(3,8) = math::fromSkewSymmetric(wTdq4.topLeftCorner<3,3>() * wReeT);
	J.block<3,1>(3,9) = math::fromSkewSymmetric(wTdq5.topLeftCorner<3,3>() * wReeT);
	J.block<3,1>(3,10) = math::fromSkewSymmetric(wTdq6.topLeftCorner<3,3>() * wReeT);
	J.block<3,1>(3,11) = math::fromSkewSymmetric(wTdq7.topLeftCorner<3,3>() * wReeT);

/*
	pmr(wTdphi);
	pmr(wTdpsi);
	pmr(wTdqw);
	pmr(wTdq1);
	pmr(wTdq2);
	pmr(wTdq3);
	pmr(wTdq4);
	pmr(wTdq5);
	pmr(wTdq6);
	pmr(wTdq7);
*/
}

/* ********************************************************************************************* */
void computeCOM (dynamics::SkeletonDynamics* krang, double imu, double waist, const Eigen::VectorXd& larm,
		const Eigen::VectorXd& rarm, Eigen::Vector3d& com) {

	// Set the masses
	static const double mlower = 102.328;
	static const double mleft = 15.143;
	static const double mright = 15.143;
	static const double totalm = mlower + mleft + mright;

	// Compute the lower com
	Vector3d lowerCOM;
	Eigen::Matrix4d wTb; 
	computeLowerCOM(krang, imu, waist, lowerCOM, wTb);
	// com = lowerCOM;
	// return;

	// Compute the arm coms
	Vector3d leftCOM, rightCOM;
	computeArmCOM(krang, wTb, larm, false, leftCOM);
	computeArmCOM(krang, wTb, rarm, true, rightCOM);

	// Compute the total com
	com = (mlower * lowerCOM + mleft * leftCOM + mright * rightCOM) / totalm;
}

/* ********************************************************************************************* */
void computeLowerCOM (dynamics::SkeletonDynamics* krang, double imu, double waist, 
		Eigen::Vector3d& com, Eigen::Matrix4d& T) {
	
	// Get the local coms
	static const bool debug = 0;
	static const Vector3d bal = krang->getNode("Base")->getLocalCOM();
	static const Vector3d sl = krang->getNode("Spine")->getLocalCOM();
	static const Vector3d bl = krang->getNode("Bracket")->getLocalCOM();
	static const Vector3d kl = krang->getNode("Kinect")->getLocalCOM();
	static const double bam = krang->getNode("Base")->getMass();
	static const double sm = krang->getNode("Spine")->getMass();
	static const double bm = krang->getNode("Bracket")->getMass();
	static const double km = krang->getNode("Kinect")->getMass();
	static const double tm = bam + sm + bm + km;

	// Base COM
	Transform <double, 3, Affine> wTba (
		AngleAxis <double> (0.0, Vector3d(0.0, 0.0, 1.0)) *
		AngleAxis <double> (imu, Vector3d(1.0, 0.0, 0.0)));
	wTba.translation() = Vector3d(0.0, 0.0, 0.27);
	Vector4d baseCOM = wTba.matrix() * Vector4d(bal(0), bal(1), bal(2), 1.0);
	if(debug) cout << "baseCOM: " << baseCOM.transpose() << endl;

	// Spine COM
	Transform <double, 3, Affine> baTs (AngleAxis <double> (waist, Vector3d(-1.0, 0.0, 0.0)));
	baTs.translation() = Vector3d(0.026, 0.499, -0.091);
	Matrix4d wTs = (wTba * baTs).matrix();
	Vector4d spineCOM = wTs.matrix() * Vector4d(sl(0), sl(1), sl(2), 1.0);
	if(debug) cout << "spineCOM: " << spineCOM.transpose() << endl;

	// Bracket COM
	Transform <double, 3, Affine> sTb (AngleAxis <double> (M_PI, Vector3d(0.0, 1.0, 0.0)));
	sTb.translation() = Vector3d(-0.027, 0.667, 0.1088);
	Matrix4d wTb = wTs * sTb.matrix();
	Vector4d bracketCOM = wTb.matrix() * Vector4d(bl(0), bl(1), bl(2), 1.0);
	if(debug) cout << "bracketCOM: " << bracketCOM.transpose() << endl;

	// Compute the Kinect com
	Transform <double, 3, Affine> bTk (AngleAxis <double> (2.5416, Vector3d(1.0, 0.0, 0.0)));
	sTb.translation() = Vector3d(0.0, 0.0975, -0.1120);
	Matrix4d wTk = wTb * bTk.matrix();
	Vector4d kinectCOM = wTk.matrix() * Vector4d(kl(0), kl(1), kl(2), 1.0);
	if(debug) cout << "kinectCOM: " << kinectCOM.transpose() << endl;

	// Compute the total com with masses
	com = ((baseCOM * bam + spineCOM * sm + bracketCOM * bm + kinectCOM * km) / tm).topLeftCorner<3,1>();
	T = wTb;
}

/* ********************************************************************************************* */
void computeArmCOM (dynamics::SkeletonDynamics* krang, const Eigen::Matrix4d& wTb, 
		const Eigen::VectorXd& qa, bool right, Eigen::Vector3d& com) {

	// Get the local coms
	static const Vector3d n1l = krang->getNode("L1")->getLocalCOM();
	static const Vector3d n2l = krang->getNode("L2")->getLocalCOM();
	static const Vector3d n3l = krang->getNode("L3")->getLocalCOM();
	static const Vector3d n4l = krang->getNode("L4")->getLocalCOM();
	static const Vector3d n5l = krang->getNode("L5")->getLocalCOM();
	static const Vector3d n6l = krang->getNode("L6")->getLocalCOM();
	static const Vector3d n7l = krang->getNode("lGripper")->getLocalCOM();
	static const double m1 = krang->getNode("L1")->getMass();
	static const double m2 = krang->getNode("L2")->getMass();
	static const double m3 = krang->getNode("L3")->getMass();
	static const double m4 = krang->getNode("L4")->getMass();
	static const double m5 = krang->getNode("L5")->getMass();
	static const double m6 = krang->getNode("L6")->getMass();
	static const double m7 = krang->getNode("lGripper")->getMass();
	static const double tm = m1 + m2 + m3 + m4 + m5 + m6 + m7;
	// cout << "tm: " << tm << endl;

	// Node 1 COM
	Transform <double, 3, Affine> bT1;
	if(!right) {
		Transform <double, 3, Affine> temp;
		temp = Transform <double, 3, Affine> (AngleAxis <double> (-1.5708, Vector3d(0.0, 0.0, 1.0)) * 
			AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)));
		temp.translation() = Vector3d(0.2823, 0.0, 0.0);
		bT1 = temp * Transform <double, 3, Affine> (AngleAxis <double> (-qa(0), Vector3d(0.0, 1.0, 0.0)));
	}
	else {
		bT1 = Transform <double, 3, Affine> (AngleAxis <double> (1.5708, Vector3d(0.0, 0.0, 1.0)) * 
			AngleAxis <double> (-qa(0), Vector3d(0.0, 1.0, 0.0)));
		bT1.translation() = Vector3d(-0.2823, 0.0, 0.0);
	}
	Matrix4d wT1 = wTb * bT1.matrix();
	Vector4d n1com = wT1.matrix() * Vector4d(n1l(0), n1l(1), n1l(2), 1.0);
	// cout << "n1COM: " << n1com.transpose() << endl;

	Transform <double, 3, Affine> _1T2t ( AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (3.1416, Vector3d(1.0, 0.0, 0.0)));
	_1T2t.translation() = Vector3d(0.0, 0.0, 0.0);
	Transform <double, 3, Affine> _1T2 = _1T2t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(1), Vector3d(1.0, 0.0, 0.0)));
	Matrix4d wT2 = wT1 * _1T2.matrix();
	Vector4d n2com = wT2.matrix() * Vector4d(n2l(0), n2l(1), n2l(2), 1.0);
	// cout << "n2COM: " << n2com.transpose() << endl;

	Transform <double, 3, Affine> _2T3t (AngleAxis <double> (-3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (-3.1416, Vector3d(1.0, 0.0, 0.0)));
	_2T3t.translation() = Vector3d(0.0, -0.328, 0.0);
	Transform <double, 3, Affine> _2T3 = _2T3t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(2), Vector3d(0.0, 1.0, 0.0)));
	Matrix4d wT3 = wT2 * _2T3.matrix();
	Vector4d n3com = wT3.matrix() * Vector4d(n3l(0), n3l(1), n3l(2), 1.0);
	// cout << "n3COM: " << n3com.transpose() << endl;

	Transform <double, 3, Affine> _3T4t ( AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (3.1416, Vector3d(1.0, 0.0, 0.0)));
	_3T4t.translation() = Vector3d(0.0, 0.0, 0.0);
	Transform <double, 3, Affine> _3T4 = _3T4t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(3), Vector3d(1.0, 0.0, 0.0)));
	Matrix4d wT4 = wT3 * _3T4.matrix();
	Vector4d n4com = wT4.matrix() * Vector4d(n4l(0), n4l(1), n4l(2), 1.0);
	// cout << "n4COM: " << n4com.transpose() << endl;

	Transform <double, 3, Affine> _4T5t (AngleAxis <double> (-3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (-3.1416, Vector3d(1.0, 0.0, 0.0)));
	_4T5t.translation() = Vector3d(0.0, -0.2765, 0.0);
	Transform <double, 3, Affine> _4T5 = _4T5t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(4), Vector3d(0.0, 1.0, 0.0)));
	Matrix4d wT5 = wT4 * _4T5.matrix();
	Vector4d n5com = wT5.matrix() * Vector4d(n5l(0), n5l(1), n5l(2), 1.0);
	// cout << "n5COM: " << n5com.transpose() << endl;

	Transform <double, 3, Affine> _5T6t ( AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (3.1416, Vector3d(1.0, 0.0, 0.0)));
	_5T6t.translation() = Vector3d(0.0, 0.0, 0.0);
	Transform <double, 3, Affine> _5T6 = _5T6t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(5), Vector3d(1.0, 0.0, 0.0)));
	Matrix4d wT6 = wT5 * _5T6.matrix();
	Vector4d n6com = wT6.matrix() * Vector4d(n6l(0), n6l(1), n6l(2), 1.0);
	// cout << "n6COM: " << n6com.transpose() << endl;

	Transform <double, 3, Affine> _6T7t (AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (1.5708, Vector3d(1.0, 0.0, 0.0)));
	_6T7t.translation() = Vector3d(0.0, -0.2, 0.0);
	Transform <double, 3, Affine> _6T7 = _6T7t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(6), Vector3d(0.0, 0.0, 1.0)));
	Matrix4d wT7 = wT6 * _6T7.matrix();
	Vector4d n7com = wT7.matrix() * Vector4d(n7l(0), n7l(1), n7l(2), 1.0);
	// cout << "n7COM: " << n7com.transpose() << endl;

	com = ((n1com * m1 + n2com * m2 + n3com * m3 + n4com * m4 + n5com * m5 + n6com * m6 + n7com * m7) / tm).topLeftCorner<3,1>();
}

/* ********************************************************************************************* */
void endEffectorJacobianBase (const Eigen::VectorXd& qb, const Eigen::VectorXd& qa, bool right, 
		Eigen::MatrixXd& J) {

	// Get the variables
	double x = qb(0), y = qb(1), phi = qb(2), psi = qb(3), qw = qb(4);
	
	// Compute the transform due to the arm configurations (we just need to translation)
	Matrix4d T;
	endEffectorInBracket(qa, right, T);
	double bTee3_4 = T(2,3), bTee1_4 = T(0,3), bTee2_4 = T(1,3);
	
	// Compute the jacobian (3x2)
	Vector3d diffpsi ((499*sin(phi)*sin(psi))/1000 - bTee3_4*((4503599627248967.0*sin(phi)*sin(psi)*sin(qw))/4503599627370496.0 + (4503599627248967.0*cos(psi)*cos(qw)*sin(phi))/4503599627370496.0) - bTee2_4*(cos(psi)*sin(phi)*sin(qw) - cos(qw)*sin(phi)*sin(psi)) - (91*cos(psi)*sin(phi))/1000 + bTee1_4*((8673110332316285.0*sin(phi)*sin(psi)*sin(qw))/1180591620717411303424.0 + (8673110332316285.0*cos(psi)*cos(qw)*sin(phi))/1180591620717411303424.0) + (68*sin(phi)*sin(psi)*sin(qw))/625 + (68*cos(psi)*cos(qw)*sin(phi))/625 - (667*cos(psi)*sin(phi)*sin(qw))/1000 + (667*cos(qw)*sin(phi)*sin(psi))/1000, (91*cos(phi)*cos(psi))/1000 - (499*cos(phi)*sin(psi))/1000 - bTee1_4*((8673110332316285.0*cos(phi)*cos(psi)*cos(qw))/1180591620717411303424.0 + (8673110332316285.0*cos(phi)*sin(psi)*sin(qw))/1180591620717411303424.0) + bTee3_4*((4503599627248967.0*cos(phi)*cos(psi)*cos(qw))/4503599627370496.0 + (4503599627248967.0*cos(phi)*sin(psi)*sin(qw))/4503599627370496.0) + bTee2_4*(cos(phi)*cos(psi)*sin(qw) - cos(phi)*cos(qw)*sin(psi)) - (68*cos(phi)*cos(psi)*cos(qw))/625 + (667*cos(phi)*cos(psi)*sin(qw))/1000 - (667*cos(phi)*cos(qw)*sin(psi))/1000 - (68*cos(phi)*sin(psi)*sin(qw))/625, (499*cos(psi))/1000 + (91*sin(psi))/1000 + (667*cos(psi)*cos(qw))/1000 + bTee1_4*((8673110332316285.0*cos(psi)*sin(qw))/1180591620717411303424.0 - (8673110332316285.0*cos(qw)*sin(psi))/1180591620717411303424.0) + (68*cos(psi)*sin(qw))/625 - (68*cos(qw)*sin(psi))/625 + (667*sin(psi)*sin(qw))/1000 - bTee3_4*((4503599627248967.0*cos(psi)*sin(qw))/4503599627370496.0 - (4503599627248967.0*cos(qw)*sin(psi))/4503599627370496.0) + bTee2_4*(cos(psi)*cos(qw) + sin(psi)*sin(qw)));
	Vector3d diffqw (bTee3_4*((4503599627248967.0*sin(phi)*sin(psi)*sin(qw))/4503599627370496.0 + (4503599627248967.0*cos(psi)*cos(qw)*sin(phi))/4503599627370496.0) + bTee2_4*(cos(psi)*sin(phi)*sin(qw) - cos(qw)*sin(phi)*sin(psi)) - bTee1_4*((8673110332316285.0*sin(phi)*sin(psi)*sin(qw))/1180591620717411303424.0 + (8673110332316285.0*cos(psi)*cos(qw)*sin(phi))/1180591620717411303424.0) - (68*sin(phi)*sin(psi)*sin(qw))/625 - (68*cos(psi)*cos(qw)*sin(phi))/625 + (667*cos(psi)*sin(phi)*sin(qw))/1000 - (667*cos(qw)*sin(phi)*sin(psi))/1000, bTee1_4*((8673110332316285.0*cos(phi)*cos(psi)*cos(qw))/1180591620717411303424.0 + (8673110332316285.0*cos(phi)*sin(psi)*sin(qw))/1180591620717411303424.0) - bTee3_4*((4503599627248967.0*cos(phi)*cos(psi)*cos(qw))/4503599627370496.0 + (4503599627248967.0*cos(phi)*sin(psi)*sin(qw))/4503599627370496.0) - bTee2_4*(cos(phi)*cos(psi)*sin(qw) - cos(phi)*cos(qw)*sin(psi)) + (68*cos(phi)*cos(psi)*cos(qw))/625 - (667*cos(phi)*cos(psi)*sin(qw))/1000 + (667*cos(phi)*cos(qw)*sin(psi))/1000 + (68*cos(phi)*sin(psi)*sin(qw))/625, (68*cos(qw)*sin(psi))/625 - bTee1_4*((8673110332316285.0*cos(psi)*sin(qw))/1180591620717411303424.0 - (8673110332316285.0*cos(qw)*sin(psi))/1180591620717411303424.0) - (68*cos(psi)*sin(qw))/625 - (667*cos(psi)*cos(qw))/1000 - (667*sin(psi)*sin(qw))/1000 + bTee3_4*((4503599627248967.0*cos(psi)*sin(qw))/4503599627370496.0 - (4503599627248967.0*cos(qw)*sin(psi))/4503599627370496.0) - bTee2_4*(cos(psi)*cos(qw) + sin(psi)*sin(qw)));
 
	J = MatrixXd(3,2);
	J.block<3,1>(0,0) = diffpsi;
	J.block<3,1>(0,1) = diffqw;
}

/* ********************************************************************************************* */
void endEffectorInBracket (const Eigen::VectorXd& qa, bool right, 
		Eigen::Matrix4d& T) {

	Transform <double, 3, Affine> bT1;
	if(!right) {
		Transform <double, 3, Affine> temp;
		temp = Transform <double, 3, Affine> (AngleAxis <double> (-1.5708, Vector3d(0.0, 0.0, 1.0)) * 
			AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)));
		temp.translation() = Vector3d(0.2823, 0.0, 0.0);
		bT1 = temp * Transform <double, 3, Affine> (AngleAxis <double> (-qa(0), Vector3d(0.0, 1.0, 0.0)));
	}
	else {
		bT1 = Transform <double, 3, Affine> (AngleAxis <double> (1.5708, Vector3d(0.0, 0.0, 1.0)) * 
			AngleAxis <double> (-qa(0), Vector3d(0.0, 1.0, 0.0)));
		bT1.translation() = Vector3d(-0.2823, 0.0, 0.0);
	}

	Transform <double, 3, Affine> _1T2t ( AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (3.1416, Vector3d(1.0, 0.0, 0.0)));
	_1T2t.translation() = Vector3d(0.0, 0.0, 0.0);
	Transform <double, 3, Affine> _1T2 = _1T2t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(1), Vector3d(1.0, 0.0, 0.0)));

	Transform <double, 3, Affine> _2T3t (AngleAxis <double> (-3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (-3.1416, Vector3d(1.0, 0.0, 0.0)));
	_2T3t.translation() = Vector3d(0.0, -0.328, 0.0);
	Transform <double, 3, Affine> _2T3 = _2T3t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(2), Vector3d(0.0, 1.0, 0.0)));

	Transform <double, 3, Affine> _3T4t ( AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (3.1416, Vector3d(1.0, 0.0, 0.0)));
	_3T4t.translation() = Vector3d(0.0, 0.0, 0.0);
	Transform <double, 3, Affine> _3T4 = _3T4t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(3), Vector3d(1.0, 0.0, 0.0)));

	Transform <double, 3, Affine> _4T5t (AngleAxis <double> (-3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (-3.1416, Vector3d(1.0, 0.0, 0.0)));
	_4T5t.translation() = Vector3d(0.0, -0.2765, 0.0);
	Transform <double, 3, Affine> _4T5 = _4T5t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(4), Vector3d(0.0, 1.0, 0.0)));

	Transform <double, 3, Affine> _5T6t ( AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (3.1416, Vector3d(1.0, 0.0, 0.0)));
	_5T6t.translation() = Vector3d(0.0, 0.0, 0.0);
	Transform <double, 3, Affine> _5T6 = _5T6t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(5), Vector3d(1.0, 0.0, 0.0)));

	Transform <double, 3, Affine> _6T7t (AngleAxis <double> (-3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (-3.1416, Vector3d(1.0, 0.0, 0.0)));
	_6T7t.translation() = Vector3d(0.0, -0.2, 0.0);
	Transform <double, 3, Affine> _6T7 = _6T7t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(6), Vector3d(0.0, 1.0, 0.0)));

	T = (bT1 * _1T2 * _2T3 * _3T4 * _4T5 * _5T6 * _6T7).matrix();
}

/* ********************************************************************************************* */
void endEffectorTransform (const Eigen::VectorXd& qb, const Eigen::VectorXd& qa, bool right, 
		Eigen::Matrix4d& T) {

	Matrix4d wTb; 
	bracketTransform(qb, wTb);
	//cout << "wTb: \n" << wTb << endl;

	Transform <double, 3, Affine> bT1;
	if(!right) {
		Transform <double, 3, Affine> temp;
		temp = Transform <double, 3, Affine> (AngleAxis <double> (-1.5708, Vector3d(0.0, 0.0, 1.0)) * 
			AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)));
		temp.translation() = Vector3d(0.2823, 0.0, 0.0);
		bT1 = temp * Transform <double, 3, Affine> (AngleAxis <double> (-qa(0), Vector3d(0.0, 1.0, 0.0)));
	}
	else {
		bT1 = Transform <double, 3, Affine> (AngleAxis <double> (1.5708, Vector3d(0.0, 0.0, 1.0)) * 
			AngleAxis <double> (-qa(0), Vector3d(0.0, 1.0, 0.0)));
		bT1.translation() = Vector3d(-0.2823, 0.0, 0.0);
	}

	Transform <double, 3, Affine> _1T2t ( AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (3.1416, Vector3d(1.0, 0.0, 0.0)));
	_1T2t.translation() = Vector3d(0.0, 0.0, 0.0);
	Transform <double, 3, Affine> _1T2 = _1T2t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(1), Vector3d(1.0, 0.0, 0.0)));

	Transform <double, 3, Affine> _2T3t (AngleAxis <double> (-3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (-3.1416, Vector3d(1.0, 0.0, 0.0)));
	_2T3t.translation() = Vector3d(0.0, -0.328, 0.0);
	Transform <double, 3, Affine> _2T3 = _2T3t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(2), Vector3d(0.0, 1.0, 0.0)));

	Transform <double, 3, Affine> _3T4t ( AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (3.1416, Vector3d(1.0, 0.0, 0.0)));
	_3T4t.translation() = Vector3d(0.0, 0.0, 0.0);
	Transform <double, 3, Affine> _3T4 = _3T4t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(3), Vector3d(1.0, 0.0, 0.0)));

	Transform <double, 3, Affine> _4T5t (AngleAxis <double> (-3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (-3.1416, Vector3d(1.0, 0.0, 0.0)));
	_4T5t.translation() = Vector3d(0.0, -0.2765, 0.0);
	Transform <double, 3, Affine> _4T5 = _4T5t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(4), Vector3d(0.0, 1.0, 0.0)));

	Transform <double, 3, Affine> _5T6t ( AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (3.1416, Vector3d(1.0, 0.0, 0.0)));
	_5T6t.translation() = Vector3d(0.0, 0.0, 0.0);
	Transform <double, 3, Affine> _5T6 = _5T6t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(5), Vector3d(1.0, 0.0, 0.0)));

	Transform <double, 3, Affine> _6T7t (AngleAxis <double> (3.1416, Vector3d(0.0, 1.0, 0.0)) * 
		AngleAxis <double> (1.5708, Vector3d(1.0, 0.0, 0.0)));
	_6T7t.translation() = Vector3d(0.0, -0.2, 0.0);
	Transform <double, 3, Affine> _6T7 = _6T7t * 
		Transform <double, 3, Affine> (AngleAxis <double> (-qa(6), Vector3d(0.0, 0.0, 1.0)));

	T = wTb * (bT1 * _1T2 * _2T3 * _3T4 * _4T5 * _5T6 * _6T7).matrix();
}

/* ********************************************************************************************* */
void bracketJacobian (const Eigen::VectorXd& conf, Eigen::MatrixXd& J) {

	double x = conf(0), y = conf(1), phi = conf(2), psi = conf(3), qw = conf(4);
	Eigen::Vector3d diffx = Eigen::Vector3d( 1, 0, 0 );
	Eigen::Vector3d diffy = Eigen::Vector3d( 0, 1, 0);
	Eigen::Vector3d diffphi = Eigen::Vector3d( sin(phi)/1000 - (499*cos(phi)*cos(psi))/1000 - (91*cos(phi)*sin(psi))/1000 - (667*cos(phi)*cos(psi)*cos(qw))/1000 - (68*cos(phi)*cos(psi)*sin(qw))/625 + (68*cos(phi)*cos(qw)*sin(psi))/625 - (667*cos(phi)*sin(psi)*sin(qw))/1000, (68*cos(qw)*sin(phi)*sin(psi))/625 - (499*cos(psi)*sin(phi))/1000 - (91*sin(phi)*sin(psi))/1000 - (667*sin(phi)*sin(psi)*sin(qw))/1000 - (667*cos(psi)*cos(qw)*sin(phi))/1000 - (68*cos(psi)*sin(phi)*sin(qw))/625 - cos(phi)/1000, 0);
	Eigen::Vector3d diffpsi = Eigen::Vector3d( (499*sin(phi)*sin(psi))/1000 - (91*cos(psi)*sin(phi))/1000 + (68*sin(phi)*sin(psi)*sin(qw))/625 + (68*cos(psi)*cos(qw)*sin(phi))/625 - (667*cos(psi)*sin(phi)*sin(qw))/1000 + (667*cos(qw)*sin(phi)*sin(psi))/1000, (91*cos(phi)*cos(psi))/1000 - (499*cos(phi)*sin(psi))/1000 - (68*cos(phi)*cos(psi)*cos(qw))/625 + (667*cos(phi)*cos(psi)*sin(qw))/1000 - (667*cos(phi)*cos(qw)*sin(psi))/1000 - (68*cos(phi)*sin(psi)*sin(qw))/625, (499*cos(psi))/1000 + (91*sin(psi))/1000 + (667*cos(psi)*cos(qw))/1000 + (68*cos(psi)*sin(qw))/625 - (68*cos(qw)*sin(psi))/625 + (667*sin(psi)*sin(qw))/1000);
  
	Eigen::Vector3d diffqw = Eigen::Vector3d( (667*cos(psi)*sin(phi)*sin(qw))/1000 - (68*cos(psi)*cos(qw)*sin(phi))/625 - (68*sin(phi)*sin(psi)*sin(qw))/625 - (667*cos(qw)*sin(phi)*sin(psi))/1000, (68*cos(phi)*cos(psi)*cos(qw))/625 - (667*cos(phi)*cos(psi)*sin(qw))/1000 + (667*cos(phi)*cos(qw)*sin(psi))/1000 + (68*cos(phi)*sin(psi)*sin(qw))/625, (68*cos(qw)*sin(psi))/625 - (68*cos(psi)*sin(qw))/625 - (667*cos(psi)*cos(qw))/1000 - (667*sin(psi)*sin(qw))/1000);
	J = Eigen::MatrixXd(3,5);
	J.block<3,1>(0,0) = diffx;	
	J.block<3,1>(0,1) = diffy;	
	J.block<3,1>(0,2) = diffphi;
	J.block<3,1>(0,3) = diffpsi;
	J.block<3,1>(0,4) = diffqw;
}

/* ********************************************************************************************* */
void bracketTransform (const VectorXd& conf, Matrix4d& T) {
	Transform <double, 3, Affine> wTba (
		AngleAxis <double> (conf(2), Vector3d(0.0, 0.0, 1.0)) *
		AngleAxis <double> (conf(3), Vector3d(1.0, 0.0, 0.0)));
	wTba.translation() = Vector3d(conf(0), conf(1), 0.27);
	Transform <double, 3, Affine> baTs (AngleAxis <double> (conf(4), Vector3d(-1.0, 0.0, 0.0)));
	baTs.translation() = Vector3d(0.026, 0.499, -0.091);
	Transform <double, 3, Affine> sTb (AngleAxis <double> (M_PI, Vector3d(0.0, 1.0, 0.0)));
	sTb.translation() = Vector3d(-0.027, 0.667, 0.1088);
	T = (wTba * baTs * sTb).matrix();
}

/* ********************************************************************************************* */
void approachGoal (dynamics::SkeletonDynamics* robot,
		const Eigen::VectorXd& goal, size_t kNumIters, bool right, Eigen::VectorXd& error, Eigen::VectorXd&
 		finalq) {

	static const bool debug = false;
	

	// Keep making incremental steps towards the goal
	VectorXd xdot;
	static VectorXd q = VectorXd::Random(12);
/*
	q(0) = q(1) = q(2) = 0.0;
	q(3) = 0.0;
	q(4) = 2.0;
	q(5) = q(7) = q(9) = q(11) = 0.0;
	q(6) = M_PI / 3.0;
	q(8) = -2*M_PI / 3.0;
	q(10) = M_PI / 3.0;
*/
	for(size_t i = 0; i < kNumIters; i++) {

		// cout << "q: " << q.transpose() << endl;

		// Get the current pose and the jacobian of the end-effector
		Matrix4d T;
		MatrixXd J;
		endEffectorJacobian(q.topLeftCorner<5,1>(), q.bottomLeftCorner<7,1>(), true, J, T);
		
		// Get the workspace velocity
		bool reached = workVelocity(T, goal, xdot, error);
		if(reached) {
			error = VectorXd::Zero(6);
			finalq = q;
			// cout << "reached goal " << i << endl;
			return;
		}
		xdot(3) = xdot(4) = xdot(5) = 0.0;

		// Get the joint-space velocities by multiplying inverse Jacobian with x.
		Eigen::MatrixXd Jt = J.transpose();
		Eigen::MatrixXd JJt = J * Jt;
		Eigen::MatrixXd JJtinv = JJt.inverse();
		Eigen::MatrixXd Jinv = Jt * JJtinv;
		VectorXd qdot = Jinv * xdot;

		// Apply the joint space velocities to the robot
		q = q + qdot;
	}
	// cout << "failed: " << error.transpose() << endl;
	// cout << "goal: " << goal.transpose() << endl;
}

/* ********************************************************************************************* */
/// Returns the workspace velocity to get the end-effector to the desired goal transformation
/// If already there, returns true.
bool workVelocity (const Matrix4d& eeTransform_, const Eigen::VectorXd& goal, 
		Eigen::VectorXd& xdot, Eigen::VectorXd& error, bool print) {

	// Get the current goal location and orientation (as a quaternion);
	Vector3d temp = goal.bottomLeftCorner<3,1>();
	Matrix3d goalRotM = math::eulerToMatrix(temp, math::XYZ);
	VectorXd goalPos = goal.topLeftCorner<3,1>();
	Quaternion <double> goalOri (goalRotM);

	// Get the current end-effector location and orientation (as a quaternion);
	Matrix4d axisChange;
	axisChange << 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
	MatrixXd eeTransform = eeTransform_ * axisChange;
	VectorXd eePos = eeTransform.topRightCorner<3,1>();
	Quaternion <double> eeOri (eeTransform.topLeftCorner<3,3>());

	// Find the position error
	VectorXd errPos = goalPos - eePos;
	
	// Find the orientation error and express it in RPY representation
	Quaternion <double> errOriQ = goalOri * eeOri.inverse();
	Matrix3d errOriM = errOriM = errOriQ.matrix();
	Vector3d errOri = math::matrixToEuler(errOriM, math::XYZ);
	error = VectorXd(6);
	error << errPos(0), errPos(1), errPos(2), errOri(0), errOri(1), errOri(2);
	if(print) cout << "eePos: " << eePos.transpose() << endl;

	// Check if the goal is reached
	static const double posLimit = 0.01;
	static const double oriLimit = 0.01;
	if((errPos.norm() < posLimit)) return true; // && (errOri.norm() < oriLimit)) return true;

	// Get the workspace velocity
	static const double kSpeed = 0.1;
	xdot = VectorXd (6);
	xdot << errPos, errOri;
	xdot = kSpeed * xdot.normalized();
	return false;
}

/* ********************************************************************************************* */
void approachGoal2 (dynamics::SkeletonDynamics* robot,
		const Eigen::VectorXd& goal, size_t kNumIters, bool right, Eigen::VectorXd& error, Eigen::VectorXd&
 		finalq) {

	static const bool debug = false;

	// Keep making incremental steps towards the goal
	VectorXd xdot;
	kinematics::BodyNode* eeNode = robot->getNode(right ? "rGripper" : "lGripper");
	for(size_t i = 0; i < kNumIters; i++) {

		// Get the workspace velocity
		gttic_(workVel);
		bool reached = workVelocity(eeNode, goal, xdot, error);
		gttoc_(workVel);
		if(reached) {
			error = VectorXd::Zero(6);
			vector <int>& dofs = right ? lowerRight_dofs : lowerLeft_dofs;
			finalq = robot->getConfig(dofs);
			if(debug) cout << "bye: " << i << endl;
			return;
		}
		xdot(3) = xdot(4) = xdot(5) = 0.0;

		// Get the joint-space velocities by multiplying inverse Jacobian with x.
		gttic_(J);
		VectorXd qdot = workToJointVelocity(eeNode, xdot, right);
		gttoc_(J);

		// Apply the joint space velocities to the robot
		gttic_(conf);
		vector <int>& dofs = right ? lowerRight_dofs : lowerLeft_dofs;
		VectorXd q = robot->getConfig(dofs);
		robot->setConfig(dofs, q + qdot, true, false);	
		gttoc_(conf);
		gtsam::tictoc_finishedIteration_();
	}
	if(debug) cout << "bye: " << kNumIters << endl;
	if(debug) cout << "goal: " << goal.transpose() << endl;
//	workVelocity(mWorld, eeNode, goal, xdot, error, true);
}

/* ********************************************************************************************* */
/// Given a workspace velocity, returns the joint space velocity
VectorXd workToJointVelocity (kinematics::BodyNode* eeNode, const VectorXd& xdot, bool right) {

	// Get the Jacobian towards computing joint-space velocities
	vector <int>& dofs = right ? lowerRight_dofs : lowerLeft_dofs;
	gttic_(getJ);
	MatrixXd JlinFull = eeNode->getJacobianLinear();
	MatrixXd JangFull = eeNode->getJacobianAngular();
	gttoc_(getJ);
	MatrixXd J (6, 12);
	gttic_(getSubset);
	for(size_t i = 0; i < 12; i++) {
		J.block<3,1>(0,i) = JlinFull.block<3,1>(0,jacobIndices[i]);
		J.block<3,1>(3,i) = JangFull.block<3,1>(0,jacobIndices[i]);
	}
	gttoc_(getSubset);
	// cout << "J =[ \n" << J << "];" << endl;

	// Compute the inverse of the Jacobian
	Eigen::MatrixXd Jt = J.transpose();
	Eigen::MatrixXd JJt = J * Jt;
	Eigen::MatrixXd JJtinv = JJt.inverse();
	// aa_la_inv(12, JJtinv.data());
	gttic_(inv);
	Eigen::MatrixXd Jinv = Jt * JJtinv;
	gttoc_(inv);
	// cout << "Jinv =[ \n" << Jinv << "];" << endl;

	// Get the joint-space velocities by multiplying inverse Jacobian with x.
	VectorXd qdot = Jinv * xdot;
	return qdot;
}

/* ********************************************************************************************* */
/// Returns the workspace velocity to get the end-effector to the desired goal transformation
/// If already there, returns true.
bool workVelocity (kinematics::BodyNode* eeNode, const Eigen::VectorXd& goal, 
		Eigen::VectorXd& xdot, Eigen::VectorXd& error, bool print) {

	// Get the current goal location and orientation (as a quaternion);
	Vector3d temp = goal.bottomLeftCorner<3,1>();
	Matrix3d goalRotM = math::eulerToMatrix(temp, math::XYZ);
	VectorXd goalPos = goal.topLeftCorner<3,1>();
	Quaternion <double> goalOri (goalRotM);

	// Get the current end-effector location and orientation (as a quaternion);
	Matrix4d axisChange;
	axisChange << 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
	MatrixXd eeTransform = eeNode->getWorldTransform() * axisChange;
	VectorXd eePos = eeTransform.topRightCorner<3,1>();
	Quaternion <double> eeOri (eeTransform.topLeftCorner<3,3>());

	// Find the position error
	VectorXd errPos = goalPos - eePos;
	
	// Find the orientation error and express it in RPY representation
	Quaternion <double> errOriQ = goalOri * eeOri.inverse();
	Matrix3d errOriM = errOriM = errOriQ.matrix();
	Vector3d errOri = math::matrixToEuler(errOriM, math::XYZ);
	error = VectorXd(6);
	error << errPos(0), errPos(1), errPos(2), errOri(0), errOri(1), errOri(2);
	if(print) cout << "eePos: " << eePos.transpose() << endl;

	// Check if the goal is reached
	static const double posLimit = 0.01;
	static const double oriLimit = 0.01;
	if((errPos.norm() < posLimit)) return true; // && (errOri.norm() < oriLimit)) return true;

	// Get the workspace velocity
	static const double kSpeed = 0.01;
	xdot = VectorXd (6);
	xdot << errPos, errOri;
	xdot = kSpeed * xdot.normalized();
	return false;
}

