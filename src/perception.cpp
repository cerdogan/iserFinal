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
ach_channel_t vision_chan;

/* ******************************************************************************************** */
void detectSmallCinder (Mode mode, const Eigen::VectorXd& mean) {

	static const bool dbg = 1;

	if(dbg) cout << "mean: " << mean.transpose() << endl;
	Eigen::Matrix4d rTo = Eigen::Matrix4d::Identity();
	double th = atan2(mean(4), mean(3)) + M_PI_2;
	if(dbg) cout << "th: " << th << endl;
	rTo.topLeftCorner<3,3>() = 
		Eigen::AngleAxis<double>(th, Eigen::Vector3d(0.0, 0.0, 1.0)).matrix();
	rTo.topRightCorner<3,1>() = mean.block<3,1>(0,0);
	if(dbg) cout << "rTo: \n" << rTo << endl;

	// Integrate the data to the robot pose
	Eigen::Matrix4d wTr = Eigen::Matrix4d::Identity();
	wTr.topRightCorner<3,1>() = Eigen::Vector3d(state(0), state(2), 0.27);
	Eigen::Vector3d rpy = Eigen::Vector3d(0, 0, state(4));
	wTr.topLeftCorner<3,3>() = math::eulerToMatrix(rpy, math::XYZ);
	if(dbg) cout << "wTr: \n" << wTr << endl;
	Eigen::Matrix4d wTo = wTr * rTo;
	if(dbg) cout << "wTo: \n" << wTo << endl;

	// Set the pose for the object 
	Eigen::VectorXd conf (6);
	conf.topLeftCorner<3,1>() = wTo.topRightCorner<3,1>();
	Eigen::Matrix3d R = wTo.topLeftCorner<3,3>();
	conf.bottomLeftCorner<3,1>() = math::matrixToEuler(R, math::XYZ);
	conf.block<2,1>(0,0) += Eigen::Vector2d(-sin(conf(5)), cos(conf(5))).normalized() * 0.15;
	double temp = conf(3); conf(3) = conf(5); conf(5) = temp;
	if(dbg) cout << "set conf: " << conf.transpose() << endl;
	conf(2) = 0.0;
	conf(4) = 0.0;
	conf(5) = M_PI_2;
	if(dbg) cout << "set conf 2: " << conf.transpose() << endl;
	world->getSkeleton("Cinder2")->setPose(conf);

	// Update the goal location for the first cinder if in part 2
	if(mode == B5) {
		Eigen::Matrix4d wTc2 = world->getSkeleton("Cinder2")->getNode("root")->getWorldTransform();
		cout << "wTc2: \n" << wTc2 << endl;
		Eigen::Matrix4d wTo = wTc2 * design.oTc2.inverse();
		Eigen::Matrix4d wTc1 = wTo * design.oTc1;
		cout << "wTc1: \n" << wTc1 << endl;
		Eigen::VectorXd c1 = matToVec(wTc1);
		cout << "c1: " << c1 << endl;
		world->getSkeleton("Cinder1G")->setPose(c1);
	}
}

/* ******************************************************************************************** */
void detectObstacle (Mode mode, const Eigen::VectorXd& mean) {

	static const bool dbg = 1;

	if(dbg) cout << "mean: " << mean.transpose() << endl;
	Eigen::Matrix4d rTo = Eigen::Matrix4d::Identity();
	double th = atan2(mean(4), mean(3)) + M_PI_2;
	if(dbg) cout << "th: " << th << endl;
	rTo.topLeftCorner<3,3>() = 
		Eigen::AngleAxis<double>(th, Eigen::Vector3d(0.0, 0.0, 1.0)).matrix();
	rTo.topRightCorner<3,1>() = mean.block<3,1>(0,0);
	if(dbg) cout << "rTo: \n" << rTo << endl;

	// Integrate the data to the robot pose
	Eigen::Matrix4d wTr = Eigen::Matrix4d::Identity();
	wTr.topRightCorner<3,1>() = Eigen::Vector3d(state(0), state(2), 0.27);
	Eigen::Vector3d rpy = Eigen::Vector3d(0, 0, state(4));
	wTr.topLeftCorner<3,3>() = math::eulerToMatrix(rpy, math::XYZ);
	if(dbg) cout << "wTr: \n" << wTr << endl;
	Eigen::Matrix4d wTo = wTr * rTo;
	if(dbg) cout << "wTo: \n" << wTo << endl;

	// Set the pose for the object 
	Eigen::VectorXd conf (6);
	conf.topLeftCorner<3,1>() = wTo.topRightCorner<3,1>();
	Eigen::Matrix3d R = wTo.topLeftCorner<3,3>();
	conf.bottomLeftCorner<3,1>() = math::matrixToEuler(R, math::XYZ);
	conf.block<2,1>(0,0) += Eigen::Vector2d(-sin(conf(5)), cos(conf(5))).normalized() * 0.15;
	double temp = conf(3); conf(3) = conf(5); conf(5) = temp;
	if(dbg) cout << "set conf: " << conf.transpose() << endl;
	conf(2) = 0.1;
	conf(4) = 0.0;
	conf(5) = 0.0;
	if(dbg) cout << "set conf 2: " << conf.transpose() << endl;
	world->getSkeleton("Obstacle")->setPose(conf);
}

/* ******************************************************************************************** */
void detectCinder (Mode mode, const Eigen::VectorXd& mean) {

	static const bool dbg = 0;

	if(dbg) cout << "mean: " << mean.transpose() << endl;
	Eigen::Matrix4d rTo = Eigen::Matrix4d::Identity();
	double th = atan2(mean(4), mean(3)) + M_PI_2;
	if(dbg) cout << "th: " << th << endl;
	rTo.topLeftCorner<3,3>() = 
		Eigen::AngleAxis<double>(th, Eigen::Vector3d(0.0, 0.0, 1.0)).matrix();
	rTo.topRightCorner<3,1>() = mean.block<3,1>(0,0);
	if(dbg) cout << "rTo: \n" << rTo << endl;

	// Integrate the data to the robot pose
	Eigen::Matrix4d wTr = Eigen::Matrix4d::Identity();
	wTr.topRightCorner<3,1>() = Eigen::Vector3d(state(0), state(2), 0.27);
	Eigen::Vector3d rpy = Eigen::Vector3d(0, 0, state(4));
	wTr.topLeftCorner<3,3>() = math::eulerToMatrix(rpy, math::XYZ);
	if(dbg) cout << "wTr: \n" << wTr << endl;
	Eigen::Matrix4d wTo = wTr * rTo;
	if(dbg) cout << "wTo: \n" << wTo << endl;

	// Set the pose for the object 
	Eigen::VectorXd conf (6);
	conf.topLeftCorner<3,1>() = wTo.topRightCorner<3,1>();
	Eigen::Matrix3d R = wTo.topLeftCorner<3,3>();
	conf.bottomLeftCorner<3,1>() = math::matrixToEuler(R, math::XYZ);
	conf.block<2,1>(0,0) += Eigen::Vector2d(-sin(conf(5)), cos(conf(5))).normalized() * 0.15;
	double temp = conf(3); conf(3) = conf(5); conf(5) = temp;
	if(dbg) cout << "set conf: " << conf.transpose() << endl;
	conf(2) = 0.0;
	conf(4) = 0.0;
	conf(5) = M_PI_2;
	if(dbg) cout << "set conf 2: " << conf.transpose() << endl;
	world->getSkeleton("Cinder1")->setPose(conf);
}

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
	if(mode == B1) 
		system("ssh 192.168.10.10 \"/home/cerdogan/Documents/Software/project/vision/build/"
			"stop-06\" > bla &");
	else 
		system("ssh 192.168.10.10 \"/home/cerdogan/Documents/Software/project/vision/build/"
			"stop-11\" > bla &");
}

/* ********************************************************************************************* */
bool perception (Mode mode) {

	// Start the program on the vision computer based on the mode
	static int c_ = 0;
	static vector <Eigen::VectorXd> data;
	if(!pinitialized) {

		cout << "\n\nInitializing perception for new mode" << endl;

		// Start the program on the vision computer
		if(mode == A2) {
			system("echo 260 450 | somatic_motor_cmd dynamixel pos");
			system("ssh 192.168.10.10 \"/home/cerdogan/Documents/Software/project/vision/build/"
				"11-detectSmallCinder\" > bla &");
			somatic_d_channel_open(&daemon_cx, &vision_chan, "smallCinder", NULL); 
		}
		else if(mode == A6) {
			system("echo 350 450 | somatic_motor_cmd dynamixel pos");
			system("ssh 192.168.10.10 \"/home/cerdogan/Documents/Software/project/vision/build/"
				"12-detectObstacle\" > bla &");
			somatic_d_channel_open(&daemon_cx, &vision_chan, "smallCinder", NULL); 
		}
		else if(mode == B1) {
			system("echo 300 450 | somatic_motor_cmd dynamixel pos");
			system("ssh 192.168.10.10 \"/home/cerdogan/Documents/Software/project/vision/build/"
				"06-detectCinder\" > bla &");
			somatic_d_channel_open(&daemon_cx, &vision_chan, "cinder", NULL); 
		}
		else if(mode == B5) {
			system("echo 260 450 | somatic_motor_cmd dynamixel pos");
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
	struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1.0/30.0) );
	int result;
	size_t numBytes = 0;
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &vision_chan, &numBytes, 
		&abstimeout, ACH_O_LAST, &result);
	if(numBytes == 0) return false;
 
	// Read the message
	Somatic__Cinder* cinder_msg = somatic__cinder__unpack(&(daemon_cx.pballoc), numBytes, 
		buffer);
	Eigen::VectorXd value (6);
	for(size_t i = 0; i < 3; i++) value(i) = cinder_msg->hole->data[i];
	for(size_t i = 0; i < 3; i++) value(i+3) = cinder_msg->normal->data[i];
	data.push_back(value);
	cout << "data count: " << data.size() << endl;

	// Compute the mean if enough data is accumulated
	if(data.size() >= 50) {
		
		// Analyze the data
		Eigen::VectorXd mean = analyzeData(data);
		
		// Use it according to the mode 
		if(mode == A2) detectSmallCinder(mode, mean);
		else if(mode == A6) detectObstacle(mode, mean);
		else if(mode == B1) detectCinder(mode, mean);
		else if(mode == B5) detectSmallCinder(mode, mean);
		else assert("unknown perception interpretation");

		// Stop the program called on the vision computer
		cleanUp(mode);
		pinitialized = false;
		return true;
	}

	// Keep accumulating data
	return false;
}
/* ********************************************************************************************* */
