/**
 * @file manipulation.cpp
 * @author Can Erdogan
 * @date Jun 04, 2014
 * @brief Handles manipulation, aka motion planning, hardcoded behaviors, etc.
 */

#include "manipulation.h"
#include <kore/workspace.hpp>

using namespace std;

bool minitialized = false;

/* ********************************************************************************************* */
void reachOut (const Eigen::Vector3d& normal, double tl1, double tl2) {

	// workspace stuff
	const double K_WORKERR_P = 1.00;
	const double NULLSPACE_GAIN = 0.0;
	const double DAMPING_GAIN = 0.005;
	Krang::WorkspaceControl* workspace = new Krang::WorkspaceControl(krang, Krang::RIGHT,K_WORKERR_P, 
		NULLSPACE_GAIN, DAMPING_GAIN, 0.0, 0.0, 0.0, 0.0);	
	Krang::Vector7d nullspace_q_ref; ///< nullspace configurations for the arms
	Krang::Vector7d nullspace_q_mask; ///< nullspace configurations for the arms
	nullspace_q_ref = (Krang::Vector7d()   << 0, -1.0, 0, -0.5, 0, -0.8, 0).finished();
	nullspace_q_mask = (Krang::Vector7d()  << 0,    0, 0,    1, 0,    0, 0).finished();

	size_t waited = 0;
	bool negateDir = false;
	double time_last = aa_tm_timespec2sec(aa_tm_now());
	bool reached = false;
	size_t counter = 0;
	while(true) {

		// Update times
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;
		counter++;

		// Update the krang
		hw->updateSensors(time_delta);

		// Stop if f/t sensors feel too much force
		cout << "\nlft: "<<hw->fts[Krang::RIGHT]->lastExternal.topLeftCorner<3,1>().transpose() << endl;
		double magn = hw->fts[Krang::RIGHT]->lastExternal.topLeftCorner<3,1>().norm();
		if(!reached && ((magn > 15) || (counter > tl1))) reached = true;
		if(reached) waited++;
		cout << "reached: " << reached << endl;
		cout << "waited: " << waited << endl;

		if(waited > tl2) {
			somatic_motor_halt(&daemon_cx, hw->arms[Krang::RIGHT]);
			return;
		}

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
		Eigen::VectorXd qdot_apply = qdot_jacobian;
		cout << "qdot_apply before norm: " << qdot_apply.transpose() << endl;
		qdot_apply = qdot_apply.normalized() * 0.05;
		cout << "qdot_apply: " << qdot_apply.transpose() << endl;

		// And apply that to the arm
		somatic_motor_setvel(&daemon_cx, hw->arms[Krang::RIGHT], qdot_apply.data(), 7);

		// Sleep to fill out the loop period so we don't eat the entire CPU
		static const double LOOP_FREQUENCY = 15.0;
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loopend - time_now);
		int time_sleep_usec = std::max(0, (int)(time_sleep * 1e6));
		usleep(time_sleep_usec);
	}
}

/* ********************************************************************************************* */
bool manipulation (Mode mode) {

	// Prepare the arm for visualization
	if(mode == A4) {

		// Move the arm to the grasp pose by first moving to a keypoint in the middle
//		Eigen::VectorXd smallKeyPoint (7);
//		smallKeyPoint << -0.211,   0.690,  -0.016,   0.879,  -1.445,   1.376,  -0.000;
//		somatic_motor_setpos(&daemon_cx,hw->arms[Krang::RIGHT], smallKeyPoint.data(), 7);
//		sleep(3);
//		somatic_motor_setpos(&daemon_cx,hw->arms[Krang::RIGHT],smallGraspPose.data(), 7);
//		sleep(6);

		// Move the camera
		double pos [] = {260, 510};
		somatic_motor_setpos(&daemon_cx, dynos, pos, 2);
		sleep(2);

		// Open the hand
		system("echo 0.9 | sudo somatic_motor_cmd rgripper pos");

		// Move the arm forward until contact
		Eigen::VectorXd conf = krang->getPose();
		double th = conf(3);
		Eigen::Vector3d normal (sin(th), -cos(th), 0.0);
		cout << normal.transpose() << endl;
		reachOut(normal, 200, 15);

		// Close the hand
		system("echo 0.0 | sudo somatic_motor_cmd rgripper pos");

		// Move the camera
		double pos2 [] = {260, 450};
		somatic_motor_setpos(&daemon_cx, dynos, pos2, 2);
		sleep(2);

		hw->updateSensors(0);

		// Move the small cinder to the middle
		Eigen::VectorXd carryKeyPoint (7);
		carryKeyPoint << 0.382000, 0.732000,  -1.458000, 0.973000, 0.000000, 1.396000, 0.0;
		somatic_motor_reset(&daemon_cx,hw->arms[Krang::RIGHT]);
		usleep(1e4);
		somatic_motor_setpos(&daemon_cx,hw->arms[Krang::RIGHT], carryKeyPoint.data(), 7);
		sleep(4);
		carryKeyPoint << -0.211,   0.664,  -0.016,   1.207,  -0.000,   1.299,  -0.000;
		somatic_motor_setpos(&daemon_cx,hw->arms[Krang::RIGHT], carryKeyPoint.data(), 7);
		sleep(4);


		return true;
	}

	// Prepare the arm for visualization
	if(mode == A8) {

		// Move the arm to the grasp pose by first moving to a keypoint in the middle
		Eigen::VectorXd smallKeyPoint2 (7);
		smallKeyPoint2 << 0.382000, 0.732000,  -1.458000, 0.973000, 0.000000, 1.396000, 0.0;
		somatic_motor_setpos(&daemon_cx,hw->arms[Krang::RIGHT], smallKeyPoint2.data(), 7);
		sleep(6);
		somatic_motor_setpos(&daemon_cx,hw->arms[Krang::RIGHT],smallGraspPose.data(), 7);
		sleep(6);

		// Open the hand
		system("echo 0.9 | sudo somatic_motor_cmd rgripper pos");

		// Move the arm out of the cinder
		Eigen::VectorXd conf = krang->getPose();
		double th = conf(3);
		Eigen::Vector3d normal (sin(th), -cos(th), 0.0);
		cout << normal.transpose() << endl;
		reachOut(-normal, 50, 0);

		// Move the small cinder to the middle
		Eigen::VectorXd carryKeyPoint (7);
		carryKeyPoint << -0.211,   0.664,  -0.016,   1.207,  -0.000,   1.299,  -0.000;
		somatic_motor_reset(&daemon_cx,hw->arms[Krang::RIGHT]);
		usleep(1e4);
		somatic_motor_setpos(&daemon_cx,hw->arms[Krang::RIGHT], carryKeyPoint.data(), 7);
		hw->updateSensors(0);
		sleep(4);

		return true;
	}


	else assert(false && "Unknown manipulation mode");
}
