/**
 * @file 01-locateCinder.cpp
 * @author Can Erdogan
 * @date Feb 17, 2014
 * @brief The robot locates the position of the cinder block wrt its own frame by moving its
 * neck around and collecting the cinder block positions. Then, chooses the one that is closest.
 */

#include <kore.hpp>
#include <kore/ik.hpp>
#include <kore/safety.hpp>
#include <kore/util.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <somatic/msg.h>

#define SQ(x) ((x) * (x))

using namespace std;
using namespace Krang;

Vector7d lgoal, rgoal;

/* ********************************************************************************************* */
// State variables

somatic_d_t daemon_cx;    
ach_channel_t cinder_chan;
somatic_motor_t* dynos;
simulation::World* world;
dynamics::SkeletonDynamics* krang;
Hardware* hw;     

bool dbg = false;
bool resetInitPos = false;
bool resetFinalArmPos = true;
bool sending_commands = false;
bool destroyAll = false;

/* ******************************************************************************************** */
void moveArmsOutOfPos (Vector7d& lgoal, Vector7d& rgoal) {

	// First move the top two joints
	Eigen::VectorXd lgoal2 = lgoal, rgoal2 = rgoal;
	lgoal2(0) = lgoal2(1) = 0.0;
	rgoal2(0) = rgoal2(1) = 0.0;
	somatic_motor_setpos(&daemon_cx, hw->arms[LEFT], lgoal2.data(), 7);
	somatic_motor_setpos(&daemon_cx, hw->arms[RIGHT], rgoal2.data(), 7);
	while(true) {
		somatic_motor_update(&daemon_cx, hw->arms[LEFT]);
		somatic_motor_update(&daemon_cx, hw->arms[RIGHT]);
		usleep(1e6);
		if(((eig7(hw->arms[LEFT]->pos)-lgoal2).squaredNorm() < 0.005) &&
			 ((eig7(hw->arms[RIGHT]->pos)-rgoal2).squaredNorm() < 0.005)) break;
	}

	// Then move the rest to zero
	double q0 [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	somatic_motor_setpos(&daemon_cx, hw->arms[LEFT], q0, 7);
	somatic_motor_setpos(&daemon_cx, hw->arms[RIGHT], q0, 7);
	while(true) {
		somatic_motor_update(&daemon_cx, hw->arms[LEFT]);
		somatic_motor_update(&daemon_cx, hw->arms[RIGHT]);
		usleep(1e6);
		if(((eig7(hw->arms[LEFT]->pos)-eig7(q0)).squaredNorm() < 0.005) &&
			 ((eig7(hw->arms[RIGHT]->pos)-eig7(q0)).squaredNorm() < 0.005)) break;
	}


}

/* ******************************************************************************************** */
void decreaseWaistAngle () {
	Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();		
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__REAL_CURRENT_MODE);
	while(true) {

		// Send a current value
		double current = 0.0;
		if(hw->waist->pos[0] < 2.3) current = 0.1;
		else if(hw->waist->pos[0] < 2.62) current = 0.0;
		else current = -0.01;
		somatic_vector_set_data(waistDaemonCmd->data, &current, 1);
		int r = SOMATIC_PACK_SEND(hw->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
		if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
			ach_result_to_string(static_cast<ach_status_t>(r)));

		// Update the state and see if we can stop
		hw->updateSensors(1e-2);
		cout << hw->waist->pos[0] << endl;
		if(hw->waist->pos[0] > 2.85) {
			somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
			int r = SOMATIC_PACK_SEND(hw->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
			if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
				ach_result_to_string(static_cast<ach_status_t>(r)));
			break;
		}
		
		usleep(1e4);
	}

	// Last move to balancing pose
	Eigen::VectorXd lfinal (7), rfinal (7);
	lfinal << 1.400, -1.000, -0.000, -0.800, -0.000, -0.800, -0.000;
	rfinal << -1.400,  1.000,  0.000,  0.800,  0.000,  0.800, -0.000;
	somatic_motor_setpos(&daemon_cx, hw->arms[LEFT], lfinal.data(), 7);
	somatic_motor_setpos(&daemon_cx, hw->arms[RIGHT], rfinal.data(), 7);
	while(true) {
		somatic_motor_update(&daemon_cx, hw->arms[LEFT]);
		somatic_motor_update(&daemon_cx, hw->arms[RIGHT]);
		usleep(1e6);
		if(((eig7(hw->arms[LEFT]->pos)-lfinal).squaredNorm() < 0.005) &&
			 ((eig7(hw->arms[RIGHT]->pos)-rfinal).squaredNorm() < 0.005)) break;
	}
}

/* ******************************************************************************************** */
/// Clean up
void destroy() {
	cout << "Moving arms out of position? Ready? " << endl;
	if(resetFinalArmPos) moveArmsOutOfPos(lgoal, rgoal);
	cout << "Moving the waist? Ready? " << endl;
	decreaseWaistAngle();
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
	somatic_motor_halt(&daemon_cx, dynos);
	somatic_motor_destroy(&daemon_cx, dynos);
	delete dynos;
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
	}
}

/* ******************************************************************************************** */
void getCinder (Eigen::Vector3d& normal, Eigen::Vector3d& hole, size_t numMessages) {

	// Open the ach channel
	somatic_d_channel_open(&daemon_cx, &cinder_chan, "cinder", NULL); 

	// Get data ~50 times and make a histogram of results
	vector <pair<Vector6d, size_t> > meanValues;
	for(int data_idx = 0; data_idx < numMessages; data_idx++) {

		// Get data
		struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );
		int result;
		size_t numBytes = 0;
		uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &cinder_chan, &numBytes, 
			&abstimeout, ACH_O_WAIT, &result);
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
	hole(2) = 0.455;
	normal = meanValues[bestMeanIdx].first.bottomLeftCorner<3,1>();
	Eigen::Vector3d perp (-normal(1), normal(0), 0.0);
	hole -= perp * 0.06;
}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Populate the set of goal states for the kinect
	static const size_t numGoals = 10;
	double goals [numGoals] = {950, 850, 750, 650, 550, 450, 350, 250, 150, 50};
	
	// start some timers
	struct timespec t_now, t_prev = aa_tm_now();
	int c_ = 0;
	size_t goalIdx = 0;
	double minDistSq = 1000000.0;
	Eigen::Vector3d bestHole, bestNormal;
	double bestGoal;
	while(!somatic_sig_received) {

		dbg = (c_++ % 20 == 0);

		// Update times
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Update the dynamixel motors
		somatic_motor_update(&daemon_cx, dynos);
		if(dbg) cout << "dynos: (" << dynos->pos[0] << ", " << dynos->pos[1] << ")" << endl;

		// Check if reached the current goal
		double diff = fabs(dynos->pos[1] - goals[goalIdx]);
		if(diff < 5.0) {

			// Get the normal and hole values
			cout << "========================================" << endl;
			cout << "Reached goal: " << goals[goalIdx] << endl;
			sleep(2);
			cout << "\tgetting data now..." << endl;
			Eigen::Vector3d normal, hole;
			getCinder(normal, hole, 50);

			// Find the distance between the cinder block and the robot
			double distSq = hole(0) * hole(0) + hole(1) * hole(1);
			if(distSq < minDistSq) {
				minDistSq = distSq;
				bestNormal = normal;
				bestHole = hole;
				bestGoal = goals[goalIdx];
			}

			// Increment the goal index or end the program
			if(goalIdx == (numGoals - 1)) {
				cout << "bestNormal: \n" << bestNormal.transpose() << endl;
				cout << "bestHole: \n" << bestHole.transpose() << endl;
				cout << "bestGoal: \n" << bestGoal << endl;
				bestGoal = 50;

				// Look ahead like you are done scanning
				double pos2 [] = {140, 450};
				cout << "Commanding: (140, " << 450 << ")" << endl;
				somatic_motor_setpos(&daemon_cx, dynos, pos2, 2);
				sleep(3);

				// Look back at the goal to get more data
				double pos [] = {140, bestGoal};
				cout << "Commanding: (140, " << bestGoal << ")" << endl;
				somatic_motor_setpos(&daemon_cx, dynos, pos, 2);
				sleep(3);
				getCinder(normal, hole, 200);
				cout << "\n\nfurther data: " << endl;
				cout << "normal:\n " << normal.transpose() << endl;
				cout << "hole:\n " << hole.transpose() << endl;

				// Write the results to a file
				fstream file ("/home/cerdogan/.cinderLocated", std::fstream::out);
				file << "hole:\n " << hole.transpose() << endl;
				file << "normal:\n " << normal.transpose() << endl;
				file.close();

				// Look ahead like you are done with the entire thing 
				cout << "Commanding: (140, " << 450 << ")" << endl;
				somatic_motor_setpos(&daemon_cx, dynos, pos2, 2);
				return;
			}
			else goalIdx++;
		}

		// Set the current goal position command	
		double pos [] = {140, goals[goalIdx]};
		if(dbg) cout << "Commanding: (140, " << goals[goalIdx] << ")" << endl;
		somatic_motor_setpos(&daemon_cx, dynos, pos, 2);
		usleep(1e5);
	}
}

/* ******************************************************************************************** */
void moveArmsToPos (Vector7d& lgoal, Vector7d& rgoal) {

	// First move the arms to the zero position
	double q0 [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	somatic_motor_setpos(&daemon_cx, hw->arms[LEFT], q0, 7);
	somatic_motor_setpos(&daemon_cx, hw->arms[RIGHT], q0, 7);
	while(true) {
		somatic_motor_update(&daemon_cx, hw->arms[LEFT]);
		somatic_motor_update(&daemon_cx, hw->arms[RIGHT]);
		usleep(1e6);
		if(((eig7(hw->arms[LEFT]->pos)-eig7(q0)).squaredNorm() < 0.005) &&
			 ((eig7(hw->arms[RIGHT]->pos)-eig7(q0)).squaredNorm() < 0.005)) break;
	}
	cout << "moved to the zero position" << endl;

	// Then move all the small joints
	Eigen::VectorXd lgoal2 = lgoal, rgoal2 = rgoal;
	lgoal2(0) = lgoal2(1) = 0.0;
	rgoal2(0) = rgoal2(1) = 0.0;
	somatic_motor_setpos(&daemon_cx, hw->arms[LEFT], lgoal2.data(), 7);
	somatic_motor_setpos(&daemon_cx, hw->arms[RIGHT], rgoal2.data(), 7);
	while(true) {
		somatic_motor_update(&daemon_cx, hw->arms[LEFT]);
		somatic_motor_update(&daemon_cx, hw->arms[RIGHT]);
		usleep(1e6);
		if(((eig7(hw->arms[LEFT]->pos)-lgoal2).squaredNorm() < 0.005) &&
			 ((eig7(hw->arms[RIGHT]->pos)-rgoal2).squaredNorm() < 0.005)) break;
	}

	// Last, move the top two joints
	somatic_motor_setpos(&daemon_cx, hw->arms[LEFT], lgoal.data(), 7);
	somatic_motor_setpos(&daemon_cx, hw->arms[RIGHT], rgoal.data(), 7);
	while(true) {
		somatic_motor_update(&daemon_cx, hw->arms[LEFT]);
		somatic_motor_update(&daemon_cx, hw->arms[RIGHT]);
		usleep(1e6);
		if(((eig7(hw->arms[LEFT]->pos)-lgoal).squaredNorm() < 0.005) &&
			 ((eig7(hw->arms[RIGHT]->pos)-rgoal).squaredNorm() < 0.005)) break;
	}
}

/* ******************************************************************************************** */
void increaseWaistAngle () {
	cout << "Moving the waist? Ready? " << endl;
	getchar();
	Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();		
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__REAL_CURRENT_MODE);
	while(true) {

		// Send a current value
		double current = -1.8;
		somatic_vector_set_data(waistDaemonCmd->data, &current, 1);
		int r = SOMATIC_PACK_SEND(hw->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
		if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
			ach_result_to_string(static_cast<ach_status_t>(r)));

		// Update the state and see if we can stop
		hw->updateSensors(1e-2);
		cout << hw->waist->pos[0] << endl;
		if(hw->waist->pos[0] < 2.151) {
			somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
			int r = SOMATIC_PACK_SEND(hw->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
			if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
				ach_result_to_string(static_cast<ach_status_t>(r)));
			break;
		}
		
		usleep(1e4);
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
	daemon_opt.ident = "01-locateCinder";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware
	Hardware::Mode mode = (Hardware::Mode)(Hardware::MODE_ALL_GRIPSCH);
	hw = new Hardware(mode, &daemon_cx, krang);
	cout << "Loaded the hardware daemons." << endl;
	hw->printState();

	// Check that the robot is in the right configuration
	if(((hw->waist->pos[0] - 2.151) > 0.1) && !destroyAll) increaseWaistAngle();
	assert((fabs(hw->waist->pos[0] - 2.151) < 0.05) && "Waist in wrong position.");
	if(resetInitPos && !destroyAll) {
		cout << "Moving arms to the zero position? Are you sure?" << endl;
		getchar();
		moveArmsToPos(lgoal, rgoal);
	}
	
	// Open a channel to the dynamixel motors
	Eigen::VectorXd lim2 = Eigen::VectorXd::Ones(2) * 2047;
	Hardware::initMotorGroup(&daemon_cx, dynos, "dynamixel-cmd", "dynamixel-state", -lim2, lim2, 
		-lim2, lim2);
	somatic_motor_update(&daemon_cx, dynos);

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {
	lgoal << 	1.379,  1.703, -1.441, -1.368, -0.583, -1.771, -0.076;
	rgoal << -1.506, -1.676,  1.645,  1.366,  0.553,  1.712,  0.083;
	if(argc > 1 && strcmp(argv[1], "-r") == 0) resetInitPos = true;
	if(argc > 1 && strcmp(argv[1], "-f") == 0) resetFinalArmPos = false;
	if(argc > 1 && strcmp(argv[1], "-d") == 0) { destroyAll = true; init(); destroy(); return 1; }
	init();
	run();
	destroy();
}
