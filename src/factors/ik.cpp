/**
 * @file ik.cpp
 * @author Can Erdogan
 * @date May 03, 2013
 * @brief Handles inverse-kinematics operation for a 7-dof arm with Krang's dimensions.
 */

#include "ik.h"
#include "kinematics.h"

#define transform(x,T) ((T * Eigen::Vector4d(x(0), x(1), x(2), 1.0)).topLeftCorner<3,1>())

bool ignoreIKDist = false;

using namespace Eigen;
using namespace std;

/* ********************************************************************************************* */
bool getArmConfFull (Part* part, size_t faceId, const Eigen::VectorXd& tL, const Eigen::VectorXd& c, 
		const Eigen::VectorXd& qb, bool right, Eigen::VectorXd& qa) {

	// Compute the location of the point
	Eigen::Matrix4d Tl; 
	createTransform(tL, Tl);
	cout << "c: " << c.transpose() << endl;

	// Get the normal of the face
	// Eigen::Vector3d normal2 = Tl.topLeftCorner<3,3>() * part->getPlane(faceId).topLeftCorner<3,1>();
	// Eigen::Vector3d normal = part->getVertexW(faceId, 0) - part->getVertexW(faceId, 1);
	Eigen::Vector3d normal = part->getPlane(faceId).topLeftCorner<3,1>();
	normal = (Tl.topLeftCorner<3,3>() * normal).normalized();
	Eigen::Matrix3d rotM;
	rotM.block<3,1>(0,2) = -normal;
 	Eigen::Vector3d col2 = Eigen::Vector3d::Random(3);
	rotM.block<3,1>(0,1) = (col2 - col2.dot(normal) * normal).normalized();
	// rotM.block<3,1>(0,1) = normal2;
	rotM.block<3,1>(0,0) = rotM.block<3,1>(0,1).cross(rotM.block<3,1>(0,2));

	// Create the goal matrix
	Matrix4d Twee = Matrix4d::Identity();
	Twee.topRightCorner<3,1>() = c;// - normal * 0.05;
	Twee.topLeftCorner<3,3>() = rotM;

	// Make the call
	Vector7d bla;
	bool result = singleArmIK(qb, Twee, right, bla, true);
	qa = Vector7d(bla);
	if(!result) cout << "can not reach: " << qa(0) << endl;
	return result;
}

/* ********************************************************************************************* */
bool getArmConf (Part* part, size_t faceId, const Eigen::VectorXd& tL, const Eigen::VectorXd& c, 
		const Eigen::VectorXd& qb, bool right, Eigen::VectorXd& qa) {

	// Compute the location of the point
	Eigen::Matrix4d Tl; 
	createTransform(tL, Tl);
	Eigen::Vector3d v1 = part->getVertexW(faceId, 0);
	Eigen::Vector3d v2 = part->getVertexW(faceId, 1);
	Eigen::Vector3d v3 = part->getVertexW(faceId, 2);
	Eigen::Vector3d v3local = c(0) * v1 + c(1) * v2 + (1 - c(0) - c(1)) * v3;
	Eigen::Vector3d vc = transform(v3local, Tl);
	// cout << "vc: " << vc.transpose() << ", right: " << right << endl;

	// Get the normal of the face
	Eigen::Vector3d normal = Tl.topLeftCorner<3,3>() * part->getPlane(faceId).topLeftCorner<3,1>();
	Eigen::Matrix3d rotM;
	rotM.block<3,1>(0,2) = normal;
 	Eigen::Vector3d col2 = Eigen::Vector3d::Random(3);
	rotM.block<3,1>(0,1) = (col2 - col2.dot(normal) * normal).normalized();
	rotM.block<3,1>(0,0) = rotM.block<3,1>(0,1).cross(rotM.block<3,1>(0,2));

	// Create the goal matrix
	Matrix4d Twee = Matrix4d::Identity();
	Twee.topRightCorner<3,1>() = vc;
	Twee.topLeftCorner<3,3>() = rotM;

	// Make the call
	Vector7d bla;
	bool result = singleArmIK(qb, Twee, right, bla);
	qa = Vector7d(bla);
	if(!result) cout << "can not reach: " << qa(0) << endl;
	return result;
}

/* ********************************************************************************************* */
/// Can avoid collisions and joint limits
bool singleArmIK (const VectorXd& base_conf, const Matrix4d& Twee, bool rightArm, Vector7d& theta,
		bool minimizeColl) {

	static const bool debug = 0;

	// Get the relative goal
	Transform <double, 3, Affine> relGoal;
	Matrix4d Twb;
	bracketTransform(base_conf, Twb);
	// cout << "Twb: \n" << Twb<< "\n";
	getWristInShoulder(Twb, Twee, rightArm, relGoal.matrix());

	// Compute the inverse kinematics if you don't want to minimize collisions
	if(!minimizeColl) {
		// double phi = 0.0;
		double phi = theta(0);
		theta = VectorXd (7);
		bool result = ik(relGoal, phi, theta);
		return result;
	}

	// Otherwise check multiple phi values
	Vector7d bestTheta;
	double maxMinDistSq = -16.0;
	bool result = false;
	for(double phi = 0.0; phi <= M_PI; phi += M_PI/16.0) {

		// Check if there is an IK
		Vector7d someTheta = VectorXd(7);
		bool someResult = ik(relGoal, phi, someTheta);
		if(debug) cout << "phi: " << phi << ", result: " << someResult << ", theta: " << someTheta.transpose() << endl;
		
		// If there is a successful result, see if it is the best one so far
		if(someResult) {

			// Set the config to the left arm and get the coms of bracket, 7 links and end-effector
			krang->setConfig(left_dofs, someTheta);
			const char* names [7] = {"Bracket", "L2", "L3", "L4", "L5", "L6", "lFingerA"}; 
			Eigen::Vector3d locs [7];
			for(size_t i = 0; i < 7; i++) {
				locs[i] = krang->getNode(names[i])->getWorldCOM();
				if(debug) cout << "\t\t locs[" << i << "] (" << names[i] << "): " << locs[i].transpose() <<
					", localCOM: " << krang->getNode(names[i])->getLocalCOM().transpose() << endl;
			}

			// Find the distance to both joint limits for each dof and determine the one that is
			// closest to the limit
			double minDistSq = 16.0;
			for(size_t i = 0; i < 7; i++) {
				for(size_t j = i+1; j < 7; j++) {
					double distSq = (locs[i] - locs[j]).squaredNorm();
					if(distSq < minDistSq) minDistSq = distSq;
				}
			}

			if(debug) cout << "\tminDistSq: " << minDistSq << ", maxMinDistSq : " << maxMinDistSq << endl;

			// Check if it is more than the current maximum minAngle
			if(minDistSq > maxMinDistSq) {
				maxMinDistSq = minDistSq;	
				bestTheta = someTheta;
			}
			result = true;			
		}
	}

	// Set the result of sampling the space
	theta = bestTheta;
	return result;
}

/* ********************************************************************************************* */
/// Returns the goal frame of the wrist in the shoulder frame
void getWristInShoulder (const Matrix4d& Twb_, const Matrix4d& Twee, bool rightArm, 
    Matrix4d& TswM) {

	// =======================================================================
	// Get the bracket frame in the world frame and then ee in bracket frame

	// Need to rotate and move the frame 8.5 cm up in the z frame
	Matrix4d smallChange;	
	smallChange << 
		0.0, 0.0, -1.0, -0.0025, 
		0.0, 1.0, 0.0, 0.00525, 
		1.0, 0.0, 0.0, 0.0008, 
		.0, .0, .0, 1.0;
  MatrixXd Twb = Twb_ * smallChange;
	
	// Flip the x and z axis if working on the right arm
	if(rightArm) {
		Matrix4d flip = Matrix4d::Identity();
		flip(0,0) = flip(2,2) = -1.0;
		Twb = Twb * flip;
	}

	// Get the end-effector in the bracket frame
	Transform <double, 3, Affine> Tbee;
	Tbee.matrix() = Twb.inverse() * Twee;
	
	// =======================================================================
	// Now, we want the waist frame in the shoulder frame. This requires
	// going backwards from the ee towards the waist and forward from the
	// bracket to the shoulder

	// First of all, we rotate from the lgPlate1, the finger, to the wrist
	Transform <double, 3, Affine> Tsw;
	Matrix3d fingerToWrist;
	fingerToWrist << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
	Tsw.linear() = Tbee.linear() * fingerToWrist;

	// Next, get the wrist location by going backwards from the end-effector towards the wrist
	// I think + () because z3 and z10 are opposite.
	Vector3d posGoal = Tbee.translation() + Tsw.linear() * Vector3d(0.0, 0.0, L7 + L8_Schunk);
	Tsw.translation() = posGoal;

	// Lastly, transform the matrix to the shoulder frame
	Tsw.translation() += Vector3d(0.0, 0.0, L4);
	TswM = Tsw.matrix();
}

/* ********************************************************************************************** */
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) {
	Eigen::MatrixXd mat2 (mat);
	for(size_t i = 0; i < mat2.rows(); i++)
		for(size_t j = 0; j < mat2.cols(); j++)
			if(fabs(mat2(i,j)) < 1e-5) mat2(i,j) = 0.0;
	return mat2;
}

/* ********************************************************************************************** */
void getAB (Transform<double, 3, Affine>& A, Transform<double, 3, Affine>& B) {

	// The A matrix is the 4x4 transformation that represents the proximal elbow frame in the
	// distal shoulder frame. In Krang, this represents the 6th frame in the 5th frame.
	Matrix4d& matA = A.matrix();
	matA << 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, -1.0, 0.0, -L5,
         0.0, 0.0, 0.0, 1.0;

	// The B matrix is the 4x4 transformation that represents the proximal wrist frame in the
	// distal elbow frame. In Krang, this represents the 7th frame in the 'rotated' 6th frame.
	Matrix4d& matB = B.matrix();
	matB << 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, -1.0, L6,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
}

/* ********************************************************************************************** */
void anglesFromRotationMatrix(double &theta1, double &theta2, double &theta3, const Vector3d &n1, 
		const Vector3d &n2, const Vector3d &n3, const Matrix3d &A) {
	
	double lambda = std::atan2(n3.cross(n2).dot(n1), n3.dot(n1));
	double temp = n1.transpose() * A * n3;
	theta2 = -lambda - acos(temp);
	theta3 = -std::atan2(n1.transpose() * A * n2, -n1.transpose() * A * n3.cross(n2));
	theta1 = -std::atan2(n2.transpose() * A * n3, -n2.cross(n1).transpose() * A * n3);

	if(theta2 < -M_PI) {
		theta2 += 2.0 * M_PI;
	}
}

/* ********************************************************************************************** */
/// Computes a transformation given the DH parameters
Eigen::Matrix4d dh (double a, double alpha, double d, double theta) {

	// Compute the cos and sin functions of the input degrees
	double cth = cos(theta), sth = sin(theta);
	double calp = cos(alpha), salp = sin(alpha);

	// Set the matrix
	Eigen::Matrix4d T;
	T << 
		cth, -sth * calp,  sth * salp, a * cth,
		sth,  cth * calp, -cth * salp, a * sth,
		0.0,        salp,        calp,       d,
		0.0,         0.0,         0.0,     1.0;  

	return T;
}


/* ********************************************************************************************** */
/// Implements Sections 3.2.1 and 3.2.2
void getElbowPosAngle (const Vector3d& relGoalLoc, double Lsw, double phi, Vector3d& elbow, 
		double& theta4) {

	const bool debug = 0;

	// Get the elbow joint angle
	theta4 = M_PI - acos((Lsw*Lsw - L6*L6 - L5*L5)/(-2*L5*L6));
	
	// Compute the direction from shoulder to goal
	const Vector3d n = relGoalLoc.normalized();

	// Compute the center and radius of the circle
	const double cosAlpha = (Lsw*Lsw + L5*L5 - L6*L6) / (2*Lsw*L5);
	const Vector3d c = cosAlpha * L5 * n;
	const double R = sqrt(1 - cosAlpha*cosAlpha) * L5;

	// Create the frame in the circle plane by choosing a random vector a
	const Vector3d a = Vector3d(1.0, 0.0, 0.0);
	const Vector3d u = (a - a.dot(n) * n).normalized();
	const Vector3d v = n.cross(u);

	// Compute the location of the elbow
	elbow = c + R*cos(phi)*u + R*sin(phi)*v;

	if(0 && debug) {
		printf("L5: %lf, L6: %lf, Lsw: %lf\n", L5, L6, Lsw);
		cout << "R: " << R<< endl;
		cout << "u: " << u.transpose() << endl;
		cout << "center: " << c.transpose() << endl;
	}

	if(debug) {
		printf("theta4: %lf (%lf)\n", theta4, RAD2DEG(theta4));
		cout << "elbow pos: " << elbow.transpose() << endl;
	}
}

/* ********************************************************************************************** */
/// Returns the transformation between the proximal and distal shoulder frames (3.2.3)
void getT1 (const Transform<double, 3, Affine>& relGoal, 
		const Transform<double, 3, Affine>& Ty, const Vector3d& elbow, 
		Transform<double, 3, Affine>& T1) {

	const bool debug = 0;

	// First, compute the transformation between distal shoulder and 
	// proximal wrist frames, and then, its translation is the position of
	// the wrist in the distal shoulder (world) frame
	Transform<double, 3, Affine> temp (A * Ty.matrix() * B);
	Vector3d w = temp.translation();
	
	// Define the axes for the distal shoulder frame assuming T_1, the 
	// transformation between proximal and distal shoulder frames is 
	// identity (the rest configuration). 
	Vector3d x = A.topRightCorner<3,1>().normalized();
	Vector3d y = (w - w.dot(x) * x);
	
	y = y.normalized();
	Vector3d z = x.cross(y);

	// First, compute the wrist location wrt to the distal shoulder
	// in the goal configuration
	Vector3d wg = relGoal.translation();

	// Then, define the axes for the distal shoulder frame in the
	// goal configuration (T_1 is not I anymore). 
	Vector3d xg = elbow.normalized();
	Vector3d yg = (wg - wg.dot(xg) * xg).normalized();
	Vector3d zg = xg.cross(yg);

	// Create the two rotation matrices from the frames above and
	// compute T1.
	Matrix3d R1r, R1g;
	R1r.col(0) = x; R1r.col(1) = y; R1r.col(2) = z;
	R1g.col(0) = xg; R1g.col(1) = yg; R1g.col(2) = zg;
	T1 = Transform<double, 3, Affine>(R1g * R1r.transpose());

	if(0 && debug) {
		cout << "Ty:\n" << Ty.matrix() << endl;
		cout << "A * Ty * B: \n" << temp.matrix() << endl << endl;
		cout << "w: " << w.transpose() << endl;
		cout << "y: " << y.transpose() << "\n" << endl;
		cout << "x: " << x.transpose() << endl;
		cout << "y: " << y.transpose() << endl;
		cout << "z: " << z.transpose() << endl;
		cout << "R1r: " << R1r << endl;
		cout << "R1g: " << R1g << endl;
	}
}

/* ********************************************************************************************** */
/// Computes the inverse-kinematics
bool ik (const Transform<double, 3, Affine>& relGoal, double phi, Matrix <double, 7, 1>& theta) {
	
	// Sanity check for triangle inequality computing the length between shoulder and wrist
	double Lsw = relGoal.translation().norm();
	// if(!ignoreIKDist && (Lsw > (L5 + L6))){
	if((Lsw > (L5 + L6))){
		theta(0) = (Lsw - (L5 + L6));
		return false;
	}

	// Get the elbow position and the angle
	Vector3d elbow;
	getElbowPosAngle(relGoal.translation(), Lsw, phi, elbow, theta(3));

	// Create the elbow transformation matrix (only rotation)
	Transform<double, 3, Affine> Te;
	Te.matrix() = dh(0.0, 0.0, 0.0, theta(3));

	// Get the T1 matrix
	Transform <double, 3, Affine> T1;
	getT1(relGoal, Te, elbow, T1);

	if(0) {
		pm(A * Te * B);
		pm(T1.inverse() * relGoal);
		pm(A.inverse() * T1.inverse() * relGoal);
		pm(Te.inverse() * A.inverse() * T1.inverse() * relGoal);
		pm(B.inverse() * Te.inverse() * A.inverse() * T1.inverse() * relGoal);
	}

	// Comptute the transformation between the proximal and distal wrist frames, T2
	Transform<double, 3, Affine> T2 ((T1.matrix() * A * Te.matrix() * B).inverse() * relGoal.matrix());

	// Define the two types of motor axes 
	Vector3d type1 = Vector3d(0.0, 0.0, 1.0);
	Vector3d type2 = Vector3d(0.0, 1.0, 0.0);

	// Get the first and last three angles
	anglesFromRotationMatrix(theta(0), theta(1), theta(2), type1, type2, type1, T1.rotation());
	anglesFromRotationMatrix(theta(4), theta(5), theta(6), type1, type2, type1, T2.rotation());

	return true;
}
