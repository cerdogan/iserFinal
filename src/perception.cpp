/**
 * @file perception.cpp
 * @author Can Erdogan
 * @date Jun 02, 2014
 * @brief Talks with the vision computer and handles frames.
 */

#include "perception.h"

using namespace std;

bool pinitialized = false;
bool pdbg = false;
Eigen::Vector3d cinder1loc (0.0, 0.0, 0.0);
Eigen::Vector3d cinder2loc (0.0, 0.0, 0.0);
Eigen::Vector3d plate1loc (0.0, 0.0, 0.0);
Eigen::Vector3d plate2loc (0.0, 0.0, 0.0);
ach_channel_t vision_chan;

/* ******************************************************************************************** */
/// Finds the consensus in the data and interprets it to get the motion parameters. Returns the
/// consensus data
Eigen::VectorXd analyzeData (const std::vector <Eigen::VectorXd>& data) {

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
	
	// Return the best value
	return meanValues[bestMeanIdx].first;
}

/* ********************************************************************************************* */
void cleanUp (Mode mode) {
	if(mode == A2) {
		system("ssh 192.168.10.10 \"/home/cerdogan/Documents/Software/project/vision/build/"
			"stop-11\" > bla &");
	}
}

/* ********************************************************************************************* */
bool perception (Mode mode) {

	// Start the program on the vision computer based on the mode
	static int c_ = 0;
	static vector <Eigen::VectorXd> data;
	if(!pinitialized) {

		// Start the program on the vision computer
		if(mode == A2) {
			system("ssh 192.168.10.10 \"/home/cerdogan/Documents/Software/project/vision/build/"
				"11-detectSmallCinder\" > bla &");
			somatic_d_channel_open(&daemon_cx, &vision_chan, "smallCinder", NULL); 
		}
		else assert(false && "unknown perception goal");

		// Reset the flags
		pinitialized = true;
		data.clear();
	}

	// Get data
	struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );
	int result;
	size_t numBytes = 0;
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &vision_chan, &numBytes, 
		&abstimeout, ACH_O_WAIT, &result);
	if(numBytes == 0) return false;

	// Read the message
	Somatic__Cinder* cinder_msg = somatic__cinder__unpack(&(daemon_cx.pballoc), numBytes, 
		buffer);
	Eigen::VectorXd value (6);
	for(size_t i = 0; i < 3; i++) value(i) = cinder_msg->hole->data[i];
	for(size_t i = 0; i < 3; i++) value(i+3) = cinder_msg->normal->data[i];
	data.push_back(value);

	// Compute the mean if enough data is accumulated
	if(data.size() >= 50) {
		
		// Analyze the data
		Eigen::VectorXd mean = analyzeData(data);
		Eigen::Vector3d rpy = mean.block<3,1>(3,0);
		Eigen::Matrix4d rTo = Eigen::Matrix4d::Identity();
		rTo.topLeftCorner<3,3>() = math::eulerToMatrix(rpy, math::XYZ);
		rTo.topRightCorner<3,1>() = mean.block<3,1>(0,0);

		// Integrate the data to the robot pose
		Eigen::Matrix4d wTr = Eigen::Matrix4d::Identity();
		wTr.topRightCorner<3,1>() = Eigen::Vector3d(state(0), state(2), 0.27);
		rpy = Eigen::Vector3d(0, 0, state(4));
		wTr.topLeftCorner<3,3>() = math::eulerToMatrix(rpy, math::XYZ);
	}

	// Keep accumulating data
	return false;
}
/* ********************************************************************************************* */
