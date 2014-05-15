/**
 * @file vision.cpp
 * @author Can Erdogan
 * @date Feb 18, 2014
 * @brief Contains the vision related helper functions.
 */

#include <Eigen/Dense>
#include <vector>
#include "controllers.h"
#include <fstream>

using namespace std;

/* ******************************************************************************************** */
/// Determines the turn angles and the distance the robot should go from its current state
Eigen::Vector3d loadToPose (const Eigen::VectorXd& load) {

	static const bool debug = 1;

	// Determine the three values
	Eigen::Vector3d planeCenter = load.block<3,1>(0,0);
	Eigen::Vector3d dir = load.block<3,1>(3,0);
	if(debug) cout << "planeCenter: " << planeCenter.transpose() << endl;
	if(debug) cout << "dir: " << dir.transpose() << endl;
	Eigen::Vector3d perp (-dir(1), dir(0), 0.0);
	Eigen::Vector3d goalPos = planeCenter - 1.2 * dir + 0.38 * perp; 
	Eigen::Vector3d normal = perp;
	if(debug) cout << "goalPos: " << goalPos.transpose() << endl;
	if(debug) cout << "normal: " << normal.transpose() << endl;
	double th2 = M_PI_2 + atan2(-goalPos(0), goalPos(1));
	double th = (M_PI_2 + atan2(normal(0), -normal(1)) + 0.2) - th2;
	if(debug) cout << "th: " << th << ", th2: " << th2 << endl;

	// Change angles to wheel encoders using the wheel radius and the rotation radius
	static const double wheelRadius = 0.264;	// r, cm
	static const double rotationRadius = 0.350837; // R, cm
	double v1 = (th2 * rotationRadius) / wheelRadius;
	double v3 = (th * rotationRadius) / wheelRadius;

	// Change the distance to the goal position to encoder values
	double dist = sqrt(goalPos(0) * goalPos(0) + goalPos(1) * goalPos(1));
	if(debug) cout << "dist: " << dist << endl;
	double v2 = dist / wheelRadius;
	Eigen::Vector3d params (v1,v2,v3);
	if(debug) cout << "params: " << params.transpose() << endl;
	return params;
}


/* ******************************************************************************************** */
/// Determines the turn angles and the distance the robot should go from its current state
Eigen::Vector3d cinderToPose (const Eigen::VectorXd& cinder) {

	static const bool debug = 0;

	// Determine the three values
	Eigen::Vector3d hole = cinder.block<3,1>(0,0), normal = cinder.block<3,1>(3,0);
	Eigen::Vector3d perp (-normal(1), normal(0), 0.0);
	Eigen::Vector3d goalPos = hole + normal * 0.65 + perp * 0.6;
	if(debug) cout << "hole: " << hole.transpose() << endl;
	if(debug) cout << "normal: " << normal.transpose() << endl;
	// double th = -(M_PI_2 + atan2(normal(0), -normal(1)) + 0.2);
	double th2 = M_PI_2 + atan2(-goalPos(0), goalPos(1));
	double th = (M_PI_2 + atan2(normal(0), -normal(1)) + 0.2) - th2;
	if(debug) cout << "th: " << th << ", th2: " << th2 << endl;

	// Change angles to wheel encoders using the wheel radius and the rotation radius
	static const double wheelRadius = 0.264;	// r, cm
	static const double rotationRadius = 0.350837; // R, cm
	double v1 = (th2 * rotationRadius) / wheelRadius;
	double v3 = (th * rotationRadius) / wheelRadius;

	// Change the distance to the goal position to encoder values
	double dist = sqrt(goalPos(0) * goalPos(0) + goalPos(1) * goalPos(1));
	if(debug) cout << "dist: " << dist << endl;
	double v2 = dist / wheelRadius;
	Eigen::Vector3d params (v1,v2,v3);
	if(debug) cout << "params: " << params.transpose() << endl;
	return params;
}

/* ******************************************************************************************** */
/// Finds the consensus in the data and interprets it to get the motion parameters. Returns the
/// consensus data
Eigen::VectorXd analyzeKinectData (const std::vector <Eigen::VectorXd>& data, bool extra) {

	static const bool debug = 0;

	// Get data ~50 times and make a histogram of results
	vector <pair<Vector6d, size_t> > meanValues;
	for(int data_idx = 0; data_idx < data.size(); data_idx++) {

		// Find the data point most closed to if one exists
		Eigen::VectorXd dataPoint = data[data_idx];
		bool foundOne = false;
		for(size_t mean_idx = 0; mean_idx < meanValues.size(); mean_idx++) {
			pair <Vector6d, size_t>& meanPair = meanValues[mean_idx];
			double distSQ = (meanPair.first - dataPoint).squaredNorm();
			if(distSQ < 0.1) {
				foundOne = true;
				meanPair.first = (meanPair.first * meanPair.second + dataPoint) / (meanPair.second + 1);
				meanPair.second++;
			}
		}

		// If could not find any data points close by, add it to the list
		if(!foundOne) meanValues.push_back(make_pair(dataPoint, 1));
	}
		
	// Find the most popular cluster
	size_t maxSize = 0, bestMeanIdx = 0;
	if(debug) cout << "Clusters: \n";  
	for(size_t mean_idx = 0; mean_idx < meanValues.size(); mean_idx++) {
		if(debug) cout << "\tcluster: " << mean_idx << ", size: " << meanValues[mean_idx].second 
			<< ", mean: " << meanValues[mean_idx].first.transpose() << endl;
		if(meanValues[mean_idx].second > maxSize) {
			maxSize = meanValues[mean_idx].second;
			bestMeanIdx = mean_idx;
		}
	}
	
	// Set the hole location and the normal
	Eigen::Vector3d hole = meanValues[bestMeanIdx].first.topLeftCorner<3,1>();
	Eigen::Vector3d normal = meanValues[bestMeanIdx].first.bottomLeftCorner<3,1>().normalized();
	if(extra) {
		hole(2) = 0.455;
		Eigen::Vector3d perp (-normal(1), normal(0), 0.0);
		hole -= perp * 0.06;
	}

	// Set the output
	Eigen::VectorXd output (6);
	output.block<3,1>(0,0) = hole;
	output.block<3,1>(3,0) = normal;
	return output;
}

/* ******************************************************************************************** */
bool getVecData (ach_channel_t& chan, Eigen::VectorXd& data) {

	// Get data
	struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );
	int result;
	size_t numBytes = 0;
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &chan, &numBytes, 
		&abstimeout, ACH_O_WAIT, &result);
	assert(false && "Channel name wrong?");
	if(numBytes == 0) return false;

	// Read the message
	Somatic__Vector* msg = somatic__vector__unpack(&(daemon_cx.pballoc), numBytes, 
		buffer);
	for(size_t i = 0; i < 3; i++) data(i) = msg->data[i];
	for(size_t i = 0; i < 3; i++) data(i+3) = msg->data[i];
	return true;
}

/* ******************************************************************************************** */
/// Reads the cinder block data coming from the vision computer and returns it in the robot frame
/// (e.g. the initial robot frame at time 0).
bool getLoad (Eigen::VectorXd& data) { 

	// Get data
	struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );
	int result;
	size_t numBytes = 0;
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &cinder_chan, &numBytes, 
		&abstimeout, ACH_O_WAIT, &result);
	if(numBytes == 0) return false;

	// Read the message
	Somatic__Vector* cinder_msg = somatic__vector__unpack(&(daemon_cx.pballoc), numBytes, 
		buffer);
	for(size_t i = 0; i < 6; i++) data(i) = cinder_msg->data[i];
	return true;
}

/* ******************************************************************************************** */
/// Reads the cinder block data coming from the vision computer and returns it in the robot frame
/// (e.g. the initial robot frame at time 0).
bool getCinder (Eigen::VectorXd& data) { 

	// Get data
	struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );
	int result;
	size_t numBytes = 0;
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &cinder_chan, &numBytes, 
		&abstimeout, ACH_O_WAIT, &result);
	if(numBytes == 0) return false;

	// Read the message
	Somatic__Cinder* cinder_msg = somatic__cinder__unpack(&(daemon_cx.pballoc), numBytes, 
		buffer);
	for(size_t i = 0; i < 3; i++) data(i) = cinder_msg->hole->data[i];
	for(size_t i = 0; i < 3; i++) data(i+3) = cinder_msg->normal->data[i];
	return true;
}

/* ******************************************************************************************** */
/// Read data from the localization executable
Eigen::VectorXd readCinderData () {
	Eigen::VectorXd data (6);
	fstream file ("/home/cerdogan/.cinderLocated");
	char line [1024];
	file.getline(line, 1024);
	file.getline(line, 1024);
	std::stringstream stream(line, std::stringstream::in);
	size_t index = 0;
	for(size_t i = 0; i < 3; i++) { stream >> data(index++); }
	file.getline(line, 1024);
	file.getline(line, 1024);
	std::stringstream stream2(line, std::stringstream::in);
	for(size_t i = 0; i < 3; i++) { stream2 >> data(index++); }
	return data;
}

