/**
 * @file 03-approachCinder-shorten.cpp
 * @author Can Erdogan
 * @date Feb 14, 2014
 * @brief Meant to run on the vision computer by the main computer after a path is found.
 */

#include <kore.hpp>
#include <kore/ik.hpp>
#include <kore/safety.hpp>
#include <kore/util.hpp>
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>

#include <iostream>
#include <fstream>
#include <iomanip>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <somatic/msg.h>
#include <list>

#define SQ(x) ((x) * (x))

using namespace std;
using namespace Krang;

list <Eigen::VectorXd> path;

/* ********************************************************************************************* */
// State variables

somatic_d_t daemon_cx;    
ach_channel_t cinder_chan;
simulation::World* world;
dynamics::SkeletonDynamics* krang;
Hardware* hw;     
Eigen::Vector3d normal, hole;

bool dbg = false;
bool resetInitPos = false;
bool sending_commands = false;
bool halt = false;

/* ******************************************************************************************** */
/// Clean up
void destroy() {
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
	delete hw;
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
/// Get the mode input
void *kbhit(void *) {
	char input;
	while(true){ 
		input=cin.get(); 
		if(input=='s') sending_commands = !sending_commands;
		if(input=='h') {
			halt = !halt;
			if(halt) somatic_motor_halt(&daemon_cx, hw->arms[LEFT]);
			else somatic_motor_reset(&daemon_cx, hw->arms[LEFT]);
		}
	}
}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// start some timers
	double time_last_display = aa_tm_timespec2sec(aa_tm_now());
	double time_last = aa_tm_timespec2sec(aa_tm_now());

	static int c_ = 0;
	Eigen::VectorXd curr;
	curr = *(path.begin());
	path.pop_front();	
	size_t index = 0;
	while(!somatic_sig_received) {

		dbg = (c_ % 20 == 0);
		c_++;
		if(dbg) cout << "\nsending commands: " << sending_commands << endl;
		if(dbg) cout << "path index: " << index << endl;
		
		// Update times
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;

		// Update the robot
		hw->updateSensors(time_delta);

		// Determine the next goal position. If close to current one, jump to next available.
		double dist = (eig7(hw->arms[LEFT]->pos) - curr).norm();
		if(dbg) cout << "dist to current goal: " << dist << endl;
		if((dist < 0.1) && !path.empty()) {
			index++;
			curr = *(path.begin());
			path.pop_front();	
		}

		// Go towards the goal configuration
		if(sending_commands) somatic_motor_setpos(&daemon_cx, hw->arms[LEFT], curr.data(), 7);

		// Sleep to fill out the loop period so we don't eat the entire CPU
		usleep(1e5);
	}
}

/* ******************************************************************************************** */
void getCinder () {

	// Open the ach channel
	somatic_d_channel_open(&daemon_cx, &cinder_chan, "cinder", NULL); 

	// Get data 50 times and make a histogram of results
	vector <pair<Vector6d, size_t> > meanValues;
	for(int data_idx = 0; data_idx < 250; data_idx++) {

		// Get data
		struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );
		int result;
		size_t numBytes = 0;
		uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &cinder_chan, &numBytes, 
			&abstimeout, ACH_O_WAIT | ACH_O_LAST, &result);
  	if(numBytes == 0) {
			data_idx--;
			continue;
		}

		// Read the message
		Somatic__Cinder* cinder_msg = somatic__cinder__unpack(&(daemon_cx.pballoc), numBytes, 
			buffer);
		Vector6d data;
		for(size_t i = 0; i < 3; i++) data(i) = cinder_msg->hole->data[i];
		for(size_t i = 0; i < 3; i++) data(i+3) = cinder_msg->normal->data[i];
		cout << "data " << data_idx << ": " << data.transpose() << endl;

		// Find the data point most closed to if one exists
		bool foundOne = false;
		for(size_t mean_idx = 0; mean_idx < meanValues.size(); mean_idx++) {
			pair <Vector6d, size_t>& meanPair = meanValues[mean_idx];
			double distSQ = (meanPair.first - data).squaredNorm();
			if(distSQ < 0.1) {
				foundOne = true;
				meanPair.first = (meanPair.first * meanPair.second + data) / (meanPair.second + 1);
				meanPair.second++;
			}
		}

		// If could not find any data points close by, add it to the list
		if(!foundOne) meanValues.push_back(make_pair(data, 1));
	}
		
	// Find the most popular cluster
	size_t maxSize = 0, bestMeanIdx = 0;
	cout << "Clusters: \n";  
	for(size_t mean_idx = 0; mean_idx < meanValues.size(); mean_idx++) {
		cout << "\tcluster: " << mean_idx << ", size: " << meanValues[mean_idx].second 
			<< ", mean: " << meanValues[mean_idx].first.transpose() << endl;
		if(meanValues[mean_idx].second > maxSize) {
			maxSize = meanValues[mean_idx].second;
			bestMeanIdx = mean_idx;
		}
	}
	
	// Set the hole location and the normal
	hole = meanValues[bestMeanIdx].first.topLeftCorner<3,1>();
	hole(2) = 0.453;
	normal = meanValues[bestMeanIdx].first.bottomLeftCorner<3,1>().normalized();
	Eigen::Vector3d perp (-normal(1), normal(0), 0.0);
	hole -= perp * 0.06;
	cout << "hole: " << hole.transpose() << endl;
	cout << "normal: " << normal.transpose() << endl;

	// Set the cinder block pose in the world representation
	Eigen::VectorXd vals (6);
	Eigen::Vector3d center = hole - 0.0945 * normal;
	cout << "center: " << center.transpose() << endl;
	vals(0) = center(0), vals(1) = center(1);
	vals(2) = vals(4) = 0.0;
	vals(5) = atan2(-normal(0), normal(1));
	cout << "theta: " << vals(5) / M_PI * 180.0 << endl;
	vals(3) = M_PI_2;
	world->getSkeleton("Cinder")->setConfig(dart_root_dof_ids, vals);

	// Write the configuration of the cinder block 
	std::fstream file (".cinder", std::fstream::out);
	file << "normal: \n" << normal.transpose() << "\nhole: \n" << hole.transpose() << 
		"\nconfig: \n" << vals.transpose() << endl;
	file.close();
	
	// Give it to the vision computer as well
	system("scp .cinder 192.168.10.10:~/");

	getchar();
}

/* ******************************************************************************************** */
void performIK () {

	// Compute the goal position
	Eigen::Matrix4d goalT = Eigen::Matrix4d::Identity();
	goalT.block<3,1>(0,3) = hole + 0.07 * normal;
	goalT.block<3,1>(0,2) = normal; 
	goalT.block<3,1>(0,0) = -Eigen::Vector3d(0,0,1);
	goalT.block<3,1>(0,1) = goalT.block<3,1>(0,2).cross(goalT.block<3,1>(0,0));
	cout << "goalT: \n" << goalT << endl;
 	
	// Perform I.K. with different arm angles until no collisions
	Vector7d theta;
	Eigen::VectorXd ql = krang->getConfig(left_arm_ids);
	bool result = singleArmIKLimitsAndCollsBestWheel(world, krang, goalT, false, 0.1, theta);
	if(!result) {
		cout << "Not reachable!" << endl;
		destroy(); exit(1);
	}
	cout << "Theta: " << theta.transpose() << endl;
	getchar();

	// Perform RRT from the current configuration to the found IK contact configuration
	planning::PathPlanner <planning::RRT> planner (*world);
	list <Eigen::VectorXd> longPath;
	bool success = planner.planPath(krang, left_arm_ids, ql, theta, longPath);
	if(success) {
		cout << "Found path: " << longPath.size() << endl;
		fstream file ("/home/cerdogan/path", fstream::out);
		file << krang->getPose().transpose() << endl;
		file << world->getSkeleton("Cinder")->getPose().transpose() << endl;
		size_t numPoints = longPath.size();
		for(size_t i = 0; i < numPoints; i++) {
			file << (*longPath.begin()).transpose() << endl;
			longPath.pop_front();
		}
		file.close();
	}
	else {
		cout << "Planner failed." << endl;
		destroy(); exit(1);
	}

	// Call the path shortener on the vision machine
	system("scp /home/cerdogan/path 192.168.10.10:"
		"/home/cerdogan/Documents/Software/project/krang/iser/build/path");
	system("ssh 192.168.10.10 \"cd /home/cerdogan/Documents/Software/project/krang/iser/build/;"
		"rm -f short; ./03-approachCinder-shorten; \"");
	system("scp 192.168.10.10:/home/cerdogan/Documents/Software/project/krang/iser/build/short /home/cerdogan/short");
	
	// Read the short file	
	fstream file ("/home/cerdogan/short", fstream::in);
	char line [1024];
	size_t i = 0;
	double newDouble;
	while(file.getline(line, 1024)) {
		i = 0;
		Eigen::VectorXd config (7);
		std::stringstream stream(line, std::stringstream::in);
		while (stream >> newDouble) config(i++) = newDouble;
		path.push_back(config);
	}
}

/* ******************************************************************************************** */
/// Initialization
void init() {

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon 
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/04-World-Grasp.urdf");
	assert((world != NULL) && "Could not find the world");
	krang = world->getSkeleton("Krang");
	Krang::setupKrangCollisionModel(world, krang);
	cout << "Loaded the Krang model." << endl;

	// Initialize the daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "03-approachCinder";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware
	Hardware::Mode mode = (Hardware::Mode)(Hardware::MODE_ALL_GRIPSCH);
	hw = new Hardware(mode, &daemon_cx, krang);
	cout << "Loaded the hardware daemons." << endl;
	hw->printState();

	// Check if the waist is within the region we want (to pick it needs to be high)
	assert(fabs(hw->waist->pos[0] - 2.73) < 0.05 && "Waist angle is too different?");
	
	// Send the arm to its initial pose
	if(resetInitPos) {
		cout << "Moving arm to the initial location: " << endl;
		double q0 [] = {0.500,  0.400, -1.571, -1.571, -1.571,  1.810, -1.571};
		somatic_motor_setpos(&daemon_cx, hw->arms[LEFT], q0, 7);
		while(true) {
			somatic_motor_update(&daemon_cx, hw->arms[LEFT]);
			usleep(1e6);
			double distSQ = 0.0;
			for(size_t i = 0; i < 7; i++) distSQ += SQ(q0[i] - hw->arms[LEFT]->pos[i]);
			if(distSQ < 0.1) break;
		}
		cout << "\treached... " << endl;
	}
	
	// Get the cinder block location
	getCinder();

	// Perform analytical inverse kinematics
	performIK();

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {
	if(argc > 1 && strcmp(argv[1], "-r") == 0) resetInitPos = true;
	init();
	run();
	destroy();
}
