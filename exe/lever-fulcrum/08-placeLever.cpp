/**
 * @file 08-placeLever.cpp
 * @author Can Erdogan
 * @date Feb 20, 2014
 * @brief Krang places the lever using only f/t data and the design parameters.
 */

#include <kore.hpp>
#include <kore/workspace.hpp>
#include <kore/display.hpp>
#include <kore/util.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <somatic/msg.h>

using namespace std;
using namespace Krang;

double bla;


// initializers for the workspace control constants
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.1;
const double DAMPING_GAIN = 0.005;
Krang::WorkspaceControl* workspace;
Krang::Vector7d nullspace_q_ref; ///< nullspace configurations for the arms
Krang::Vector7d nullspace_q_mask; ///< nullspace configurations for the arms

ach_channel_t cinder_chan;
Krang::Hardware* krang;                                   ///< connects to hardware
somatic_d_t daemon_cx;                          ///< daemon context
simulation::World* world;
dynamics::SkeletonDynamics* robot;
Eigen::Vector3d normal, hole;
bool dbg = false;

enum MODE {
	GoDown = 0,
	GoRight,
	GoLeft,
	OpenHand,
	GoLeft2,
	CloseHand,
	GoUp,
};

/* ******************************************************************************************** */
/// Clean up
void destroy() {
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
	delete workspace;
	delete krang;
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// start some timers
	double time_last = aa_tm_timespec2sec(aa_tm_now());
	MODE mode = GoDown;
	MODE lastMode = mode;
	size_t counter = 0;
	double tz = krang->fts[LEFT]->lastExternal(5);
	while(!somatic_sig_received) {

		// Update times
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;
		cout << "mode: " << mode << endl;
		if(lastMode != mode) getchar();
		lastMode = mode;

		// Update the robot
		krang->updateSensors(time_delta);

		// Behave based on the mode
		Vector6d xdot;
		bool jacob = false;
		switch(mode) {
			case GoDown: {
				Vector7d qFront;
//				qFront << 1.176, -1.556, -1.531,  -1.390,  -0.004,  1.4,  -1.594;
//				qFront << 1.356,  -1.523,  -1.531,  -1.876,  -0.004,   1.723,  -1.594;
				// qFront << 1.339, -1.237,  -1.832,  -1.702,  -0.320,   1.638,  -1.345;
			//	qFront << 1.337 , -1.231,  -1.834,  -1.722,  -0.325,   1.664,  -1.351;
 qFront << 1.299000,   -0.792000,  -2.200000,  -1.453000 , -0.670000  , 1.797000 , -1.151000;


				somatic_motor_setpos(&daemon_cx, krang->arms[Krang::LEFT], qFront.data(), 7);
				sleep(3);
				mode = GoRight;
			} break;
			case GoRight: {
				xdot << 0.0, -1.0, 0.0, 0.0, 0.0, 0.0;
				jacob = true;
				cout << "\nlft: " << krang->fts[Krang::LEFT]->lastExternal.topLeftCorner<3,1>().transpose() << endl;
				double magn = krang->fts[Krang::LEFT]->lastExternal.topLeftCorner<3,1>().norm();
				if(magn > 15) {
					Vector7d zero = Vector7d::Zero();
					somatic_motor_setvel(&daemon_cx, krang->arms[Krang::LEFT], zero.data(), 7);
					mode = GoLeft;
					continue;
				}
			} break;
			case GoLeft: {
				xdot << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
				jacob = true;
				counter++;
				if(counter > 50) {
					Vector7d zero = Vector7d::Zero();
					somatic_motor_setvel(&daemon_cx, krang->arms[Krang::LEFT], zero.data(), 7);
					mode = GoUp;
					counter = 0;
					continue;
				}
			} break;
			case OpenHand: {
				system("echo 0.9 | sudo somatic_motor_cmd lgripper pos");
				sleep(2);
				mode = GoLeft2;
			} break;
			case GoLeft2: {
				xdot << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
				jacob = true;
				counter++;
				if(counter > 150) {
					Vector7d zero = Vector7d::Zero();
					somatic_motor_setvel(&daemon_cx, krang->arms[Krang::LEFT], zero.data(), 7);
					mode = CloseHand;
					counter = 0;
					continue;
				}
			} break;
			case CloseHand: {
				system("echo 0.0 | sudo somatic_motor_cmd lgripper pos");
				sleep(2);
				mode = GoUp;
			} break;
			case GoUp: {
				double tz = krang->fts[LEFT]->lastExternal(3);
				xdot << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
				cout << "\ntz: " << tz << endl;
				cout << "xdot: " << xdot.transpose() << endl;
				jacob = true;
				counter++;
				if(counter > 2500) {
					Vector7d zero = Vector7d::Zero();
					somatic_motor_setvel(&daemon_cx, krang->arms[Krang::LEFT], zero.data(), 7);
					mode = CloseHand;
					counter = 0;
					continue;
				}
			} break;
		};

		if(jacob) {

			// Nullspace: construct a qdot that the jacobian will bias toward using the nullspace
			Eigen::VectorXd q = robot->getConfig(*workspace->arm_ids);
			Krang::Vector7d nullspace_qdot_ref = (nullspace_q_ref - q).cwiseProduct(nullspace_q_mask);

			// Jacobian: compute the desired jointspace velocity from the inputs and sensors
			Eigen::VectorXd qdot_jacobian;
			workspace->refJSVelocity(xdot, nullspace_qdot_ref, qdot_jacobian);
			cout << "qdot_jacobian: " << qdot_jacobian.transpose() << endl;

			// Avoid joint limits
			Eigen::VectorXd qdot_avoid(7);
			Krang::computeQdotAvoidLimits(robot, *workspace->arm_ids, q, qdot_avoid);

			// Add qdots together to get the overall movement
			Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;
			cout << "qdot_apply before norm: " << qdot_apply.transpose() << endl;
			qdot_apply = qdot_apply.normalized() * 0.05;
			cout << "qdot_apply: " << qdot_apply.transpose() << endl;

			// And apply that to the arm
			double tz = krang->fts[LEFT]->lastExternal(3);
			cout << "tz: " << tz << endl;
			if(mode == GoUp && fabs(tz) < 20.0) qdot_apply(6) = bla;
			somatic_motor_setvel(&daemon_cx, krang->arms[Krang::LEFT], qdot_apply.data(), 7);
		}


		// Sleep to fill out the loop period so we don't eat the entire CPU
		static const double LOOP_FREQUENCY = 15.0;
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loopend - time_now);
		int time_sleep_usec = std::max(0, (int)(time_sleep * 1e6));
		usleep(time_sleep_usec);
	}
}

/* ******************************************************************************************** *
void getCinder () {

	// Open the ach channel
	somatic_d_channel_open(&daemon_cx, &cinder_chan, "cinder", NULL); 

	// Get data 50 times and make a histogram of results
	vector <pair<Vector6d, size_t> > meanValues;
	for(int data_idx = 0; data_idx < 50; data_idx++) {

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
	hole(2) = 0.453;
	normal = meanValues[bestMeanIdx].first.bottomLeftCorner<3,1>().normalized();
	Eigen::Vector3d perp (-normal(1), normal(0), 0.0);
	hole -= perp * 0.06;
	cout << "hole: " << hole.transpose() << endl;
	cout << "normal: " << normal.transpose() << endl;
}

/* ******************************************************************************************** */
/// Initialization
void init() {

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon 
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton("Krang");

	// Initialize the daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "02-pushCinder";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware
	Krang::Hardware::Mode mode = (Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL_GRIPSCH);
	krang = new Krang::Hardware(mode, &daemon_cx, robot);

	// Set up the workspace stuff
	workspace = new Krang::WorkspaceControl(robot, Krang::LEFT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
																				  0.0, 0.0, 0.0, 0.0);	

	// Set up nullspace stuff
	nullspace_q_ref = (Krang::Vector7d()   << 0, -1.0, 0, -0.5, 0, -0.8, 0).finished();
	nullspace_q_mask = (Krang::Vector7d()  << 0,    0, 0,    1, 0,    0, 0).finished();

	// Home development...
	// getCinder();
	// cout << "hole: " << hole.transpose() << endl;
	// getchar();

	// Get the cinder block location (and assume it is parallel to the robot)
	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {
	assert(argc > 1);
	bla = atof(argv[1]);
	init();
	run();
	destroy();
}
