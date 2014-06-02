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
