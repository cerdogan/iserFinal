/**
 * @file 02-refineApproach.cpp
 * @author Can Erdogan
 * @date Feb 16, 2014
 * @brief Performs Jacobian inverse kinematics to refine the gripper location by +-5 cm.
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

/* ********************************************************************************************* */
// Type declarations

/* ********************************************************************************************* */
// Constants

// initializers for the workspace control constants
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.1;
const double DAMPING_GAIN = 0.005;
bool negateDir = false;

// various rate limits - be nice to other programs

using namespace std;

/* ********************************************************************************************* */
// State variables

// hardware objects
Krang::Hardware* hw;                                   ///< connects to hardware

// process state
somatic_d_t daemon_cx;                          ///< daemon context
ach_channel_t cinder_chan;

// pointers to important DART objects
simulation::World* world;
dynamics::SkeletonDynamics* krang;
bool dbg = false;

// workspace stuff
Krang::WorkspaceControl* workspace;
Krang::Vector7d nullspace_q_ref; ///< nullspace configurations for the arms
Krang::Vector7d nullspace_q_mask; ///< nullspace configurations for the arms

// cinder block
Eigen::Vector3d normal;

/* ******************************************************************************************** */
/// Clean up
void destroy() {
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
	delete workspace;
	delete hw;
	somatic_d_destroy(&daemon_cx);

}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// start some timers
	double time_last_display = aa_tm_timespec2sec(aa_tm_now());
	double time_last = aa_tm_timespec2sec(aa_tm_now());
	bool reached = false || negateDir;
	size_t waited = 0;
	size_t limit = negateDir ? 200 : 20;
	while(!somatic_sig_received) {

		// Update times
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;

		// Update the krang
		hw->updateSensors(time_delta);

		// Stop if f/t sensors feel too much force
		cout << "\nlft: " << hw->fts[Krang::LEFT]->lastExternal.topLeftCorner<3,1>().transpose() << endl;
		double magn = hw->fts[Krang::LEFT]->lastExternal.topLeftCorner<3,1>().norm();
		if(!reached && (magn > 15)) reached = true;
		if(reached) waited++;
		cout << "reached: " << reached << endl;
		cout << "waited: " << waited << endl;

		if(waited > limit) return;
		

		// Nullspace: construct a qdot that the jacobian will bias toward using the nullspace
		Eigen::VectorXd q = krang->getConfig(*workspace->arm_ids);
		Krang::Vector7d nullspace_qdot_ref = (nullspace_q_ref - q).cwiseProduct(nullspace_q_mask);

		// Get a workspace velocity from the goal position TODO
		Eigen::VectorXd xdot_ws_goal = Eigen::VectorXd::Zero(6);
		for(size_t i = 0; i < 3; i++) xdot_ws_goal(i) = (reached ? 1 : -1) * normal(i);
		cout << "xdot: " << xdot_ws_goal.transpose() << endl;

		// Jacobian: compute the desired jointspace velocity from the inputs and sensors
		Eigen::VectorXd qdot_jacobian;
		workspace->refJSVelocity(xdot_ws_goal, nullspace_qdot_ref, qdot_jacobian);
		cout << "qdot_jacobian: " << qdot_jacobian.transpose() << endl;

		// Avoid joint limits
		Eigen::VectorXd qdot_avoid(7);
		Krang::computeQdotAvoidLimits(krang, *workspace->arm_ids, q, qdot_avoid);

		// Add qdots together to get the overall movement
		Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;
		cout << "qdot_apply before norm: " << qdot_apply.transpose() << endl;
		qdot_apply = qdot_apply.normalized() * 0.05;
		cout << "qdot_apply: " << qdot_apply.transpose() << endl;

		// And apply that to the arm
		somatic_motor_setvel(&daemon_cx, hw->arms[Krang::LEFT], qdot_apply.data(), 7);

		// Sleep to fill out the loop period so we don't eat the entire CPU
		static const double LOOP_FREQUENCY = 15.0;
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loopend - time_now);
		int time_sleep_usec = std::max(0, (int)(time_sleep * 1e6));
		usleep(time_sleep_usec);
	}
}

/* ******************************************************************************************** */
void readCinder () {
// 	fstream file ("/var/run/somatic/01-approachCinder/.cinder", fstream::in);
// 	assert(file.is_open());
// 	string line;
// 	getline(file, line);
// 	getline(file, line);
// 	stringstream stream (line, stringstream::in);
// 	double temp;
// 	for(size_t i = 0; i < 3; i++) {
// 		stream >> temp;
// 		normal(i) = temp;
// 	}
// 	cout << "normal: " << normal.transpose() << endl;
	normal = -Eigen::Vector3d(1.0, 0.05, 0.0).normalized();
	getchar();
}

/* ******************************************************************************************** */
/// Initialization
void init() {

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon 
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	krang = world->getSkeleton("Krang");

	// Initialize the daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "02-pushCinder";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware
	Krang::Hardware::Mode mode = (Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL_GRIPSCH);
	hw = new Krang::Hardware(mode, &daemon_cx, krang);

	// Set up the workspace stuff
	workspace = new Krang::WorkspaceControl(krang, Krang::LEFT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
																				  0.0, 0.0, 0.0, 0.0);	

	// Set up nullspace stuff
	nullspace_q_ref = (Krang::Vector7d()   << 0, -1.0, 0, -0.5, 0, -0.8, 0).finished();
	nullspace_q_mask = (Krang::Vector7d()  << 0,    0, 0,    1, 0,    0, 0).finished();

	// Read the cinder block normal
	readCinder();

	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {
	if(argc > 1 && strcmp(argv[1], "-n") == 0) negateDir = true;
	init();
	run();
	destroy();
}
