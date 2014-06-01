/**
 * @file 03-standUp.cpp
 * @author Can Erdogan
 * @date Feb 16, 2014
 * @brief Krang brings the cinder block to its front and stands up.
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

Krang::Hardware* hw;                                   ///< connects to hardware
somatic_d_t daemon_cx;        
simulation::World* world;
dynamics::SkeletonDynamics* krang;
somatic_motor_t* larmTop;
ach_channel_t cinder_chan;
bool dbg = false;

/* ******************************************************************************************** */
void increaseWaistAngle () {
	cout << "Moving the waist? Ready? " << endl;
	getchar();
	Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();		
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__REAL_CURRENT_MODE);
	while(true) {

		// Send a current value
		double current = -2.5;
		somatic_vector_set_data(waistDaemonCmd->data, &current, 1);
		int r = SOMATIC_PACK_SEND(hw->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
		if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
			ach_result_to_string(static_cast<ach_status_t>(r)));

		// Update the state and see if we can stop
		hw->updateSensors(1e-2);
		cout << hw->waist->pos[0] << endl;
		if(hw->waist->pos[0] < 2.618) {
			somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
			int r = SOMATIC_PACK_SEND(hw->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
			if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
				ach_result_to_string(static_cast<ach_status_t>(r)));
			break;
		}
		
		usleep(1e4);
	}
}
/* ********************************************************************************************* */
int main () {

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon 
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	krang = world->getSkeleton("Krang");

	// Initialize the daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "05-prepareBrickStand";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Get the current robot configuration and then delete it because we will use a special pciod
	Krang::Hardware::Mode mode = (Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL_GRIPSCH);
	hw = new Krang::Hardware(mode, &daemon_cx, krang);
	Vector7d lq0 = eig7(hw->arms[LEFT]->pos);
	cout << "lq0: " << lq0.transpose() << endl;
	cout << "krang pose: " << krang->getPose().transpose() << endl;

	// Close the hand
	getchar();
	system("echo 0 | sudo somatic_motor_cmd lgripper pos");
	sleep(1);

	// Use the waist to move the block off the ground
	increaseWaistAngle();
	hw->updateSensors(0.0);
	cout << "Done with waist? " << endl;
	getchar();

	// ===========================================================================
	// Part 1: Use the top two joints on left arm to move cinder to front/middle

	// Delete the hardware because we only need left top arm 
	delete hw;

	// Delete the normal left arm pciod - we will restart it at the end
	system("sudo sns -k pciod-llwa-standup");

	// Start a second pciod daemon to control only the top arm so that the rest of the
	// modules are on breaks and don't yell to the heavy weight
	system("sudo sns -k pciod-llwa-standup");
	system("sudo pciod -d -I pciod-llwa-standup -c llwa-top-cmd -s llwa-top-state -b 0 -m 4 -m 5");
	Eigen::VectorXd lim2 = Eigen::VectorXd::Ones(2) * 1024.1;
	Hardware::initMotorGroup(&daemon_cx, larmTop, "llwa-top-cmd", "llwa-top-state", -lim2, lim2, 
		-lim2, lim2);
	usleep(1e4);
	somatic_motor_reset(&daemon_cx, larmTop);
	usleep(1e4);
	somatic_motor_update(&daemon_cx, larmTop);
	usleep(1e4);

	// Move the top joint 
	double inputPos [] = {lq0(0) + 0.3, lq0(1)};
	cout << "Raising the left arm. Ready?\n goalPos: " << inputPos[0] << ", " << inputPos[1] << endl;
	getchar();
	somatic_motor_setpos(&daemon_cx, larmTop, inputPos, 2);
	usleep(1e4);
	somatic_motor_update(&daemon_cx, larmTop);
	sleep(4);

	// Move the second joint until the end-effector is in the middle of the front
	cout << "moving the cinder block to the middle. ready? " << endl;
	getchar();
	double inputVel [] = {0.0, 0.1};
	somatic_motor_setvel(&daemon_cx, larmTop, inputVel, 2);
	vector <int> topIds;
	topIds.push_back(11);
	topIds.push_back(14);
	while(true) {

		// Update the state of the robot
		somatic_motor_update(&daemon_cx, larmTop);
		usleep(1e4);
		Eigen::Vector2d topVals (larmTop->pos[0], larmTop->pos[1]);
		// DISPLAY_VECTOR(topVals);
		krang->setConfig(topIds, topVals);

		// Get the state of the object
		Eigen::Matrix4d T = krang->getNode("lGripper")->getWorldTransform();
		Eigen::Vector3d objectCOM = T.topRightCorner<3,1>() + T.block<3,1>(0,2) * 0.20;
		DISPLAY_VECTOR(objectCOM);

		// Stop when we reach the correct position
		if(objectCOM(1) < 0.04) {
			somatic_motor_halt(&daemon_cx, larmTop);
			break;
		}
	}

	// Close the arm
	somatic_motor_destroy(&daemon_cx, larmTop);
	system("sudo sns -k pciod-llwa-standup");

	// Restart the original pciod
	system("pciod -d -I pciod-llwa -c llwa-cmd -s llwa-state -b 0 -m 4 -m 5 -m 6 -m 7 "
		"-b 1 -m 8 -m 9 -m 10 -v -v");

	// ===========================================================================
	// Part 2: Use the second arm to hold the cinder block stably

	// Reconnect to the hardware
	hw = new Krang::Hardware(mode, &daemon_cx, krang);
	hw->updateSensors(0.0);
	hw->printState();

	// Determine where the object is
	Eigen::Vector3d lGripperPos = 
		krang->getNode("lGripper")->getWorldTransform().topRightCorner<3,1>();
	Eigen::Vector3d objPos = lGripperPos + 	krang->getNode("lGripper")->getWorldTransform().block<3,1>(0,2) * 0.20;
	objPos -= Eigen::Vector3d(-0.05, 0.0, 0.18);
	cout << "lGripperPos: " << lGripperPos.transpose() << endl;
	cout << "objPos: " << objPos.transpose() << endl;
	
	// Open the right arm a bit
	cout << "Going to open the right arm" << endl;
	getchar();
	Vector7d qr = eig7(hw->arms[RIGHT]->pos);
	qr(1) -= 0.30;
	somatic_motor_setpos(&daemon_cx, hw->arms[RIGHT], qr.data(), 7);
	hw->updateSensors(0.0);
	sleep(2);
	cout << "Done opening" << endl;

	cout << "moving the arm down?" << endl;
	getchar();
	// Move down until level with the object
	double vel [] = {0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double vel0 [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	somatic_motor_setvel(&daemon_cx, hw->arms[RIGHT], vel, 7);
	struct timespec t_now, t_prev = aa_tm_now();
	while(true) {
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;
		hw->updateSensors(dt);
		Eigen::Vector3d rGripperPos = 
			krang->getNode("rGripper")->getWorldTransform().topRightCorner<3,1>();
		if(rGripperPos(2) < objPos(2)) {
			somatic_motor_setvel(&daemon_cx, hw->arms[RIGHT], vel0, 7);
			break;
		}
	}
	cout << "Done moving down\nClosing the arm?" << endl;
	getchar();

	// Move left until enough force is sensed
	double vel2 [] = {0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
	somatic_motor_setvel(&daemon_cx, hw->arms[RIGHT], vel2, 7);
	t_prev = aa_tm_now();
	while(true) {
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;
		hw->updateSensors(dt);
		Eigen::VectorXd ft = hw->fts[Krang::RIGHT]->lastExternal;
		double ftMagn = ft.topLeftCorner<3,1>().norm();
		if(ftMagn > 70) {
			somatic_motor_setvel(&daemon_cx, hw->arms[RIGHT], vel0, 7);
			break;;
		}
	}
	
	cout << "Done closing" << endl;
	getchar();
	
	delete hw;
}

