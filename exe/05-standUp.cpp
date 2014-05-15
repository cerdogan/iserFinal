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

Vector6d K_stand, K_bal;
Vector6d state0;	
bool dbg = false;
bool start = false;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int locoMode = 1;

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {

	// Get the gains
	Vector6d* kgains [] = {&K_stand, &K_bal};
	ifstream file ("/home/cerdogan/Documents/Software/project/krang/iser/data/gains-03.txt");
	assert(file.is_open());
	char line [1024];
	for(size_t k_idx = 0; k_idx < 2; k_idx++) {
		*kgains[k_idx] = Vector6d::Zero();
		file.getline(line, 1024);
		std::stringstream stream(line, std::stringstream::in);
		size_t i = 0;
		double newDouble;
		while ((i < 6) && (stream >> newDouble)) (*kgains[k_idx])(i++) = newDouble;
	}
	cout << "K_stand: " << K_stand.transpose() << endl;
	cout << "K_bal: " << K_bal.transpose() << endl;
	file.close();
}

/* ******************************************************************************************** */
/// Clean up
void destroy() {
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
	system("echo 0.9 | sudo somatic_motor_cmd lgripper pos");
	delete hw;
	somatic_d_destroy(&daemon_cx);

}

/* ********************************************************************************************* */
/// Get the mode input
void *kbhit(void *) {
	char input;
	while(true){ 
		input=cin.get(); 
		pthread_mutex_lock(&mutex);
		if(input=='s') start = !start;
		else if(input=='0') locoMode = 0;
		else if(input=='1') locoMode = 1;
		else if(input=='2') locoMode = 2;
		else if(input=='r') readGains();
		pthread_mutex_unlock(&mutex);
	}
}

/* ******************************************************************************************** */
void computeTorques (const Vector6d& state, double& ul, double& ur) {

	// Set reference based on the mode
	Vector6d refState;
	if(locoMode == 1 || locoMode == 2) refState << 0.0, 0.0, state0(2), 0.0, state0(4), 0.0;
	else {
		ul = ur = 0.0;
		return;
	}

	// Set the gains
	Vector6d K;
	if(locoMode == 1) K = K_stand;
	else if(locoMode == 2) K = K_bal;
	else assert(false);
	if(dbg) cout << "K: " << K.transpose() << endl;

	// Compute the error
	Vector6d error = state - refState;
	if(dbg) cout << "error: " << error.transpose() << endl;

	// Compute the forward and spin torques 
	double u_x = K(2)*error(2) + K(3)*error(3);
	double u_spin = K.bottomLeftCorner<2,1>().dot(error.bottomLeftCorner<2,1>());
	double u_theta = K.topLeftCorner<2,1>().dot(error.topLeftCorner<2,1>());

	// Limit the output torques
	if(dbg) printf("u_theta: %lf, u_x: %lf, u_spin: %lf\n", u_theta, u_x, u_spin);
	u_spin = max(-10.0, min(10.0, u_spin));
	ul = u_theta + u_x + u_spin;
	ur = u_theta + u_x - u_spin;
	ul = max(-50.0, min(50.0, ul));
	ur = max(-50.0, min(50.0, ur));
}


/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void getState(Vector6d& state, double dt) {

	// Read motor encoders, imu and ft and update dart skeleton
  hw->updateSensors(dt);

	// Calculate the COM of the body
	Eigen::Vector3d comRobot = krang->getWorldCOM();
	comRobot(2) -= 0.264;
	comRobot(0) += 0.00;  
	if(dbg) cout << "comRobot: " << comRobot.transpose() << endl;

	// Get the com of the object
	Eigen::Matrix4d T = krang->getNode("lGripper")->getWorldTransform();
	Eigen::Vector3d objectCOM = T.topRightCorner<3,1>() + T.block<3,1>(0,2) * 0.20;
	if(dbg) cout << "R: " << T.topLeftCorner<3,3>() << endl;

	// Combine the two coms
	Eigen::Vector3d com = (comRobot * 145.0 + objectCOM * 13.2) / (145 + 13.2);
	if(dbg) cout << "com: " << com.transpose() << endl;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	state(0) = atan2(com(0), com(2));
	state(1) = hw->imuSpeed;
	state(2) = (hw->amc->pos[0] + hw->amc->pos[1]) / 2.0 + hw->imu;
	state(3) = (hw->amc->vel[0] + hw->amc->vel[1]) / 2.0 + hw->imuSpeed;
	state(4) = (hw->amc->pos[1] - hw->amc->pos[0]) / 2.0;
	state(5) = (hw->amc->vel[1] - hw->amc->vel[0]) / 2.0;
}

/* ******************************************************************************************** */
void switchModes (const Vector6d& state) {
	static int balancedCounter = 0;
	if(locoMode == 1) {
		if(fabs(state(0)) < 0.064) balancedCounter++;
		if(balancedCounter > 100) {
			locoMode = 2;
			balancedCounter = 0;
			state0 = state;
		}	
	}
}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// start some timers
	Vector6d state;	
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	int lastLocoMode = locoMode;
	while(!somatic_sig_received) {

		pthread_mutex_lock(&mutex);
		dbg = (c_++ % 20 == 0);
		if(dbg) cout << "\nmode: " << locoMode << endl;
		
		// Update times
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;
		if(dbg) cout << "dt: " << dt << endl;

		// Get the locomotion state 
		getState(state, dt); 
		if(dbg) cout << "state: " << state.transpose() << endl;

		// Check if the arm has reached the goal position or if the body is moving
		bool armReached = (fabs(hw->arms[LEFT]->pos[0] - (-1.8)) < 0.05);
		bool baseMoved = (hw->imu > -1.8);
		if(dbg) cout << "armReached: " << armReached << ", baseMoved: " << baseMoved << endl;
		if((armReached || baseMoved) && (locoMode == 0)) {
			locoMode = 1;
			state0 = state;
		}
		if(dbg) cout << "state0: " << state0.transpose() << endl;

		// Switch the mode if necessary
		switchModes(state);

		// Compute the torques based on the state and the mode
		double ul, ur;
		computeTorques(state, ul, ur);
	
		// Apply the torque
		double input [2] = {ul, ur};
		if(dbg) cout << "start: " << start << "\nu: {" << ul << ", " << ur << "}" << endl;
		if(!start) input[0] = input[1] = 0.0;
//		somatic_motor_cmd(&daemon_cx, hw->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);

		// Update the locomotion mode
		lastLocoMode = locoMode;
		pthread_mutex_unlock(&mutex);
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
	krang = world->getSkeleton("Krang");

	// Initialize the daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "03-standUp";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware
	Krang::Hardware::Mode mode = (Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL_GRIPSCH);
	hw = new Krang::Hardware(mode, &daemon_cx, krang);
	Vector7d lq0 = eig7(hw->arms[LEFT]->pos);

	// Read control gains
	readGains();

	// Close the hands
	system("echo 0 | sudo somatic_motor_cmd lgripper pos");
	sleep(2);

	// Start a second pciod daemon to control only the top arm so that the rest of the
	// modules are on breaks and don't yell to the heavy weight
	system("sudo sns -k pciod-llwa-standup");
	system("sudo pciod -d -I pciod-llwa-standup -c llwa-top-cmd -s llwa-top-state -b 0 -m 4 -m 5");
	Eigen::VectorXd lim2 = Eigen::VectorXd::Ones(2) * 1024.1;
	Hardware::initMotorGroup(&daemon_cx, larmTop, "llwa-top-cmd", "llwa-top-state", -lim2, lim2, 
		-lim2, lim2);

	// Move the top joint 
	double inputPos [] = {lq0(0) + 0.6, lq0(1)};
	somatic_motor_setpos(&daemon_cx, larmTop, inputPos, 2);
	somatic_motor_update(&daemon_cx, larmTop);
	sleep(4);

	// Move the second joint until the end-effector is in the middle of the front
	double inputVel [] = {0.0, 0.1};
	somatic_motor_setvel(&daemon_cx, larmTop, inputVel, 2);
	struct timespec t_now, t_prev = aa_tm_now();
	while(true) {
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;
		hw->updateSensors(dt);
		cout << endl;
		DISPLAY_SCALAR(dt);
		DISPLAY_VECTOR(eig7(hw->arms[LEFT]->pos));
		Eigen::Matrix4d T = krang->getNode("lGripper")->getWorldTransform();
		Eigen::Vector3d objectCOM = T.topRightCorner<3,1>() + T.block<3,1>(0,2) * 0.20;
		DISPLAY_VECTOR(objectCOM);
		if(objectCOM(1) < 0.0) {
			somatic_motor_halt(&daemon_cx, larmTop);
			break;
		}
		usleep(1e4);
	}

	// Close the arm
	somatic_motor_destroy(&daemon_cx, larmTop);
	system("sudo sns -k pciod-llwa-standup");
	exit(0);

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);

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
