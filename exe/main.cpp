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
somatic_motor_t* dynos = NULL;
simulation::World* world;
dynamics::SkeletonDynamics* krang;
Hardware* hw;     
Mode mode = A2;
Vector6d state;					//< current state (x,x.,y,y.,th,th.)

bool sending_commands = false;

/* ********************************************************************************************* */
/// Mapping from modes to the type of modules they need

bool nullFunc (Mode mode) { return false; }
typedef bool (*Module)(Mode);
map <Mode, Module> modeMapping;
void setupModeMapping () {
	modeMapping[A1] = locomotion;
	modeMapping[A2] = perception;
	modeMapping[A3] = locomotion;
	modeMapping[A4] = perception;
	modeMapping[A5] = nullFunc;
}

/* ********************************************************************************************* */
/// Clean up
void destroy() {
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
	if(dynos != NULL) {
		somatic_motor_halt(&daemon_cx, dynos);
		somatic_motor_destroy(&daemon_cx, dynos);
		delete dynos;
	}
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
	Module M = modeMapping.at(mode);
	bool result = M(mode);
	if(result) mode = (Mode) (mode + 1);
}

/* ******************************************************************************************** */
/// Initialization
void init() {

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon 
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/13-World-Plates.urdf");
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

	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
#ifdef GRIP_ON

	// ==========================================================================================
	void Timer::Notify() {

		// Do the execution
		run();

		// Visualize the scene
		viewer->DrawGLScene();
		Start(0.005 * 1e4);
	}

	// ==========================================================================================
	SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
					 long style) : GRIPTab(parent, id, pos, size, style) {

		// Create user interface
		wxSizer* sizerFull= new wxBoxSizer(wxHORIZONTAL);
		viewer->camRadius = 3.0;
		viewer->worldV += Vector3d(-0.3, 0.0, -0.8);
		viewer->UpdateCamera();
		SetSizer(sizerFull);

		// Create the timer to notify the function that draws the robot at multiple configurations
		timer = new Timer();
		timer->Start(1);

		// Initialize the threads 
		init();

		// Set the world
		mWorld = world;
		viewer->backColor = Vector3d(0.95,0.95,0.95);
		viewer->gridColor = Vector3d(.8,.8,1);
		viewer->setClearColor();
		frame->DoLoadHelp("bla.urdf", false);
	}

	// ==========================================================================================
	SimTab::~SimTab() {
		destroy();
	}

	// ==========================================================================================
	BEGIN_EVENT_TABLE(SimTab, wxPanel)
	END_EVENT_TABLE()
	IMPLEMENT_DYNAMIC_CLASS(SimTab, GRIPTab)
	extern wxNotebook* tabView;
	class mainApp : public GRIPApp {
		virtual void AddTabs() {
		tabView->AddPage(new SimTab(tabView), wxT("Liberty"));
		}
	};
	IMPLEMENT_APP(mainApp)

#else

	int main(int argc, char* argv[]) {
		init();
		while(!somatic_sig_received) run();
		destroy();
	}

#endif 
