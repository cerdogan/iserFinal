/**
 * @file main.cpp
 * @author Can Erdogan
 * @date Jun 02, 2014
 * @brief The ramp example.
 */

#include "helpers.h"
#include "locomotion.h"
#include "perception.h"
#include "manipulation.h"

#define GRIP_ON 1

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
Vector6d state;					//< current state (x,x.,y,y.,th,th.)
Mode mode = B5;

bool sending_commands = false;

Design design;

/* ********************************************************************************************* */
/// Mapping from modes to the type of modules they need

bool nullFunc (Mode mode) { return false; }
typedef bool (*Module)(Mode);
map <Mode, Module> modeMapping;
void setupModeMapping () {
	modeMapping[A1] = locomotion;
	modeMapping[A2] = perception;
	modeMapping[A3] = locomotion;
	modeMapping[A4] = manipulation;
	modeMapping[A5] = locomotion;
	modeMapping[A6] = perception;
	modeMapping[A7] = locomotion;
	modeMapping[A8] = manipulation;
	modeMapping[A9] = nullFunc;

	modeMapping[B1] = perception;
	modeMapping[B2] = locomotion;
	modeMapping[B3] = manipulation;
	modeMapping[B4] = locomotion;
	modeMapping[B5] = perception;
	modeMapping[B6] = locomotion;
	modeMapping[B7] = manipulation;
	modeMapping[B7] = nullFunc;
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
		pthread_mutex_lock(&mutex);
		if(input=='s') sending_commands = !sending_commands;
		if(input=='r') readGains();
		pthread_mutex_unlock(&mutex);
	}
}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// Get the mode
	Module M = modeMapping.at(mode);

	// Check that the waist is at the right value
	if(mode <= A8) {
		Eigen::VectorXd conf = krang->getPose();
		assert(fabs(conf(8) - 2.809) < 0.01 && "Waist in wrong position for part A");
	}

	// Run the mode
	bool result = M(mode);

	// Change mode if successful
	if(result) mode = (Mode) (mode + 1);
}

/* ******************************************************************************************** */
void readDesign () {

	// Read the data in to the configuration vectors
	fstream file ("/home/cerdogan/result");
	assert(file.is_open() && "Could not open the design file");
	Eigen::VectorXd obstacle (6), cinder1 (6), cinder2 (6), plate1 (6), plate2 (6);
	char line [1024];
	file.getline(line, 1024);
	std::stringstream stream1(line, std::stringstream::in);
	size_t i = 0;
	double newDouble;
	while ((i < 6) && (stream1 >> newDouble)) obstacle(i++) = newDouble;
	file.getline(line, 1024);
	std::stringstream stream2(line, std::stringstream::in);
	i = 0;
	while ((i < 6) && (stream2 >> newDouble)) cinder1(i++) = newDouble;
	file.getline(line, 1024);
	std::stringstream stream3(line, std::stringstream::in);
	i = 0;
	while ((i < 6) && (stream3 >> newDouble)) cinder2(i++) = newDouble;
	file.getline(line, 1024);
	std::stringstream stream4(line, std::stringstream::in);
	i = 0;
	while ((i < 6) && (stream4 >> newDouble)) plate1(i++) = newDouble;
	file.getline(line, 1024);
	std::stringstream stream5(line, std::stringstream::in);
	i = 0;
	while ((i < 6) && (stream5 >> newDouble)) plate2(i++) = newDouble;
	obstacle(2) += 5.0;
	cinder1(2) += 5.0;
	cinder2(2) += 5.0;
	plate1(2) += 5.0;
	plate2(2) += 5.0;

	// Set the configuration of the objects
	world->getSkeleton("Obstacle")->setPose(obstacle);
	world->getSkeleton("Cinder2G")->setPose(cinder1);
	world->getSkeleton("Cinder1G")->setPose(cinder2);
	world->getSkeleton("Plate1G")->setPose(plate1);
	world->getSkeleton("Plate2G")->setPose(plate2);

	// Get the relative transformations
	Eigen::Matrix4d oTw = world->getSkeleton("Obstacle")->getNode("root")->getWorldTransform().inverse();
	design.oTc1 = oTw * world->getSkeleton("Cinder1G")->getNode("root")->getWorldTransform();
	design.oTc2 = oTw * world->getSkeleton("Cinder2G")->getNode("root")->getWorldTransform();
	design.oTp1 = oTw * world->getSkeleton("Plate1G")->getNode("root")->getWorldTransform();
	design.oTp2 = oTw * world->getSkeleton("Plate2G")->getNode("root")->getWorldTransform();
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
	Eigen::VectorXd lim2 = Eigen::VectorXd::Ones(2) * 2047;
	Hardware::initMotorGroup(&daemon_cx, dynos, "dynamixel-cmd", "dynamixel-state", -lim2, lim2, 
		-lim2, lim2);
	somatic_motor_update(&daemon_cx, dynos);

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);

	// Set up the modules
	setupModeMapping();

	// Read the design
	readDesign();

	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
#ifdef GRIP_ON

	// ==========================================================================================
	void Timer::Notify() {

		// Update the design configurations
		if(mode < B1) {
			Eigen::Matrix4d wTo = world->getSkeleton("Obstacle")->getNode("root")->getWorldTransform();
			Eigen::Matrix4d wTc1 = wTo * design.oTc1; 
			Eigen::Matrix4d wTc2 = wTo * design.oTc2;
			Eigen::Matrix4d wTp1 = wTo * design.oTp1;
			Eigen::Matrix4d wTp2 = wTo * design.oTp2;
			Eigen::VectorXd c1 = matToVec(wTc1);
			Eigen::VectorXd c2 = matToVec(wTc2);
			Eigen::VectorXd p1 = matToVec(wTp1);
			Eigen::VectorXd p2 = matToVec(wTp2);
			world->getSkeleton("Cinder1G")->setPose(c1);
			world->getSkeleton("Cinder2G")->setPose(c2);
			world->getSkeleton("Plate1G")->setPose(p1);
			world->getSkeleton("Plate2G")->setPose(p2);
		}

		// Do the execution
		run();

		// Visualize the scene
		viewer->DrawGLScene();
		Start(50);
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
