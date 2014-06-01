/**
 * @file 07-pickLever.cpp
 * @author Can Erdogan
 * @date Feb 20, 2014
 * @brief Drives towards a lever while sitting down on the ground and detects when both wheels
 * are against the lever by checking encoder values being the same (torque 25Nm seems good).
 * Then uses left arm to pick up the lever using a predefined configuration (after backing up
 * by 0.60 degrees on both wheels).
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
bool dbg = false;

enum MODE {
	MakeContact = 0,
	BackUp,
	PrepareArm1,
	PrepareArm2,
	ReachObject,
	GrabObject,
	PullUp,
	TakeFront
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

	// Goal positions for the arm
	Vector7d qKey, qReach, qGrab, qFront;
	qKey << 1.516, -1.556, -1.534, -0.800, 0.00, -0.800,  0.000;
	qReach << 1.516, -1.556, -1.534, -1.598, 0.00, -0.4, -1.584; 
	qGrab << 1.176, -1.556, -1.531,  -1.390,  -0.004,  -0.272,  -1.594;
	qFront << 1.176, -1.556, -1.531,  -1.390,  -0.004,  1.4,  -1.594;

	// start some timers
	double time_last = aa_tm_timespec2sec(aa_tm_now());
	bool reachedKeyPoint = false;
	MODE mode = MakeContact;
	size_t numSameIters = 0;
	Eigen::Vector2d lastVals;
	Eigen::Vector2d desiredBackVals;
	MODE lastMode = mode;
	double goalZ;
	bool goalZset = false;
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
		bool jacob = false;
		Vector6d xdot;
		switch(mode) {
			case MakeContact: {

				// Check if the encoder values changed in the last x iterations
				double d0 = lastVals(0) - krang->amc->pos[0], d1 = lastVals(1) - krang->amc->pos[1];
				if((fabs(d0) > 0.003) || (fabs(d1) > 0.003)) {
					numSameIters = 0;
					lastVals = Eigen::Vector2d(krang->amc->pos[0], krang->amc->pos[1]); 
				}
				else numSameIters++;
				cout << "\tnumSameIters: " << numSameIters << endl;
	
				// If the limit is reached, declare contact made
				if(numSameIters > 30) {
					mode = BackUp;
					desiredBackVals = lastVals - Eigen::Vector2d(0.70, 0.70);
				}
	
				// Move forward
				else {
					double u [] = {20.0, 20.0};
					somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, u, 2, NULL);
				}
			} break;

			case BackUp: {
				
				// Check if reached desired position
				Eigen::Vector2d curr (krang->amc->pos[0], krang->amc->pos[1]); 
				if((curr(0) < desiredBackVals(0)) && (curr(1) < desiredBackVals(1))) {
					double u [] = {0.0, 0.0};
					somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, u, 2, NULL);
					mode = PrepareArm1;
				}
				else {
					// Go backward slowly 
					double u [] = {-7.5, -7.5};
					somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, u, 2, NULL);
				}
			} break;

			case PrepareArm1: {
				somatic_motor_setpos(&daemon_cx, krang->arms[LEFT], qKey.data(), 7);
				sleep(5);
				mode = PrepareArm2;
			} break;
			case PrepareArm2: {
				somatic_motor_setpos(&daemon_cx, krang->arms[LEFT], qReach.data(), 7);
				sleep(5);
				mode = ReachObject;
			} break;
			case ReachObject: {
				jacob = true;
				xdot << 0.0, 0.0, -1.0, 0.0, 0.0, 0.0;
				Eigen::Vector3d temp = robot->getNode("lGripper")->getWorldTransform().topRightCorner<3,1>();
				if(!goalZset) {
					goalZ = temp(2);
					goalZset = true;
				}
			} break;
			case GrabObject: {
				system("echo 0 | sudo somatic_motor_cmd lgripper pos");
				sleep(2);
				mode = PullUp;
			} break;
			case PullUp: {
				jacob = true;
				xdot << 0.0, 0.0, 1.0, 0.0, 0.0, 0.1;
			} break;
			case TakeFront: {
				somatic_motor_setpos(&daemon_cx, krang->arms[LEFT], qFront.data(), 7);
				sleep(5);
				return;
			} break;
		};
	
		if(mode == ReachObject) {
			Eigen::Vector3d temp = robot->getNode("lGripper")->getWorldTransform().topRightCorner<3,1>();
			if(temp(2) < (goalZ - 0.12)) {
				Vector7d zero = Vector7d::Zero();
				somatic_motor_setvel(&daemon_cx, krang->arms[Krang::LEFT], zero.data(), 7);
				mode = GrabObject;
				jacob = false;
			}
		}

		if(mode == PullUp) {
			Eigen::Vector3d temp = robot->getNode("lGripper")->getWorldTransform().topRightCorner<3,1>();
			if(temp(2) > (goalZ + 0.1)) {
				Vector7d zero = Vector7d::Zero();
				somatic_motor_setvel(&daemon_cx, krang->arms[Krang::LEFT], zero.data(), 7);
				mode = TakeFront;
				jacob = false;
			}
		}

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

	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {
	init();
	run();
	destroy();
}
