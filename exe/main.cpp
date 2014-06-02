/**
 * @file main.cpp
 * @author Can Erdogan
 * @date Jun 02, 2014
 * @brief The ramp example.
 */

#include "helpers.h"
#include "locomotion.h"
#include "perception.h"

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
Mode mode = A1;
Vector6d state;					//< current state (x,x.,y,y.,th,th.)

bool sending_commands = false;

/* ********************************************************************************************* */
/// Mapping from modes to the type of modules they need

typedef bool (*Module)(Mode);
map <Mode, Module> modeMapping;
void setupModeMapping () {
	modeMapping[A1] = locomotion;
	modeMapping[A2] = perception;
}

/* ********************************************************************************************* */
/// Clean up
void destroy() {
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

/* ********************************************************************************************* */
/// The main loop
void run() {

	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// start some timers
	struct timespec t_now, t_prev = aa_tm_now();
	int c_ = 0;
	while(!somatic_sig_received) {
		Module M = modeMapping.at(mode);
		bool result = M(mode);
		if(result) mode = (Mode) (mode + 1);
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
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	krang = world->getSkeleton("Krang");
	Krang::setupKrangCollisionModel(world, krang);
	cout << "Loaded the Krang model." << endl;

	// Initialize the daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "main";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware
	Hardware::Mode mode = (Hardware::Mode)(Hardware::MODE_ALL_GRIPSCH);
	hw = new Hardware(mode, &daemon_cx, krang);
	cout << "Loaded the hardware daemons." << endl;
	hw->printState();

	// Open a channel to the dynamixel motors
	// Eigen::VectorXd lim2 = Eigen::VectorXd::Ones(2) * 2047;
	// Hardware::initMotorGroup(&daemon_cx, dynos, "dynamixel-cmd", "dynamixel-state", -lim2, lim2, 
	// 	-lim2, lim2);
	// somatic_motor_update(&daemon_cx, dynos);

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);

	// Set up the modules
	setupModeMapping();
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {
	init();
	run();
	destroy();
}
