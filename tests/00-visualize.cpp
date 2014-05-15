/**
 * @author Can Erdogan
 * @file 00-visualize.cpp
 * @date Feb 19, 2014
 * @brief Visualizes the scene using the ach channels from Krang.
 */

#include <collision/CollisionDetector.h>
#include <collision/fcl_mesh/FCLMESHCollisionDetector.h>
#include <dynamics/ConstraintDynamics.h>
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>

#include <kore/ik.hpp>
#include <kore/safety.hpp>
#include <kore/simTab.hpp>
#include <kore/util.hpp>
#include <kore.hpp>

#include "vision.h"

using namespace Krang;
using namespace std;

bool dbg = false;
Hardware* krang;
somatic_d_t daemon_cx;    
ach_channel_t cinder_chan;
dynamics::SkeletonDynamics* robot;
somatic_motor_t* dynos;
list <Eigen::VectorXd> path;
Eigen::VectorXd ql0;
Vector7d theta;
bool armSet = false;

/* ********************************************************************************************* */
/// Application class
extern wxNotebook* tabView;
class mainApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new SimTab(tabView), wxT("-"));
	}
};

IMPLEMENT_DYNAMIC_CLASS(SimTab, GRIPTab)
DECLARE_APP(mainApp)
IMPLEMENT_APP(mainApp)

/* ********************************************************************************************* */
/// Detects collisions with the environment and within robot
struct timespec t_now, t_prev;	///< for timing each iteration
void Timer::Notify() {

	static vector <Eigen::VectorXd> data;

	// Get the current time and compute the difference
	t_now = aa_tm_now();
	double dt = (double) aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));
	t_prev = t_now;

	// Get data from the ach channels
	krang->updateSensors(dt);

	// Do the collision checking
	bool collision = mWorld->checkCollision(false);

	// Set the background
	if(collision) {
		glClearColor(1.0, 0.0, 0.0 ,1.0f);
		float fogCol[3] = {1.0, 0.0, 0.0};
		glFogfv(GL_FOG_COLOR,fogCol);
	}
	else viewer->setClearColor();

	// Update the dynamixel motors
	somatic_motor_update(&daemon_cx, dynos);
  double tilt = (dynos->pos[0] - 240.0) * (300.0 / 1024.0) * (M_PI / 180.0);
  double pan = (dynos->pos[1] - 450.0) * (300.0 / 1024.0) * (M_PI / 180.0);
	Vector2d kinect_dofs (tilt, pan);
	robot->setConfig(kinect_ids, kinect_dofs);

	// Place the cinder if data is available
	Eigen::VectorXd dataPoint (6);
	if(getCinder(dataPoint)) {
		if(dataPoint(0) < 9.99) 
			data.push_back(dataPoint);
	}
	if(data.size() > 50) {
		Eigen::VectorXd visionData = analyzeKinectData(data);
		Eigen::VectorXd hole = visionData.block<3,1>(0,0);
		Eigen::VectorXd normal = visionData.block<3,1>(3,0);
		Eigen::Vector3d center = hole - 0.0945 * normal;
		cout << "center: " << center.transpose() << endl;
		Eigen::VectorXd vals (6);
		vals(0) = center(0), vals(1) = center(1);
		vals(2) = vals(4) = 0.0;
		vals(5) = atan2(-normal(0), normal(1));
		cout << "theta: " << vals(5) / M_PI * 180.0 << endl;
		vals(3) = M_PI_2;
		mWorld->getSkeleton("Cinder")->setConfig(dart_root_dof_ids, vals);
		data.clear();
	}

	// Clean up the daemon memory
	aa_mem_region_release(&daemon_cx.memreg);

	// If planning is done, show it
	if(armSet) robot->setConfig(left_arm_ids, theta);
	
	// Update the scene
	viewer->DrawGLScene();
	Start(0.01 * 1e3);	
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
		long style) : GRIPTab(parent, id, pos, size, style) {

	// Setup environment
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* sizerFull = new wxBoxSizer (wxHORIZONTAL);
	viewer->camRadius = 10.0;
	viewer->UpdateCamera();
	SetSizer(sizerFull);
	frame->DoLoad("/etc/kore/scenes/04-World-Grasp.urdf");
	wxGetApp().setConfiguration("/etc/kore/scenes/configs/01-standUp.config");
	robot = mWorld->getSkeleton("Krang");
	Krang::setupKrangCollisionModel(mWorld, robot);
	ql0 = robot->getConfig(left_arm_ids);

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "00-visualize";
	somatic_d_init(&daemon_cx, &dopt );

	// Create the hardware object to read sensors from the piped ach channels
	Hardware::Mode mode = (Hardware::Mode)(Hardware::MODE_ALL_GRIPSCH);
	cout << "creating..." << endl;
	krang = new Hardware(mode, &daemon_cx, robot);
	cout << "created..." << endl;
	krang->printState();

	// Initialize the dynamixel motors
	dynos = new somatic_motor_t();
	somatic_motor_init(&daemon_cx, dynos, 2, "dynamixel-cmd", "dynamixel-state");

	// Open the ach channel
	somatic_d_channel_open(&daemon_cx, &cinder_chan, "cinder", NULL); 

	// Set the min/max values for the pos/vel fields' valid and limit values
	Eigen::Vector2d mins (-2000, -2000), maxs(2000, 2000);
	for(int i = 0; i < 2; i++) {
		dynos->pos_valid_min[i] = mins[i];
		dynos->pos_valid_max[i] = maxs[i];
		dynos->pos_limit_min[i] = mins[i];
		dynos->pos_limit_max[i] = maxs[i];

		dynos->vel_valid_min[i] = mins[i];
		dynos->vel_valid_max[i] = maxs[i];
		dynos->vel_limit_min[i] = mins[i];
		dynos->vel_limit_max[i] = maxs[i];
	}

	// Update and reset them
	somatic_motor_reset(&daemon_cx, dynos);
	usleep(1e5);
	somatic_motor_update(&daemon_cx, dynos);
	usleep(1e5);

	// Create the timer to notify the function that draws the robot at multiple configurations
	timer = new Timer();
	timer->Start(1);	

	// Add a button
	sizerFull->Add(new wxButton(this, 1233, wxT("Perform!")), 0, wxALL, 1); 
	sizerFull->Add(new wxButton(this, 1234, wxT("Reset!")), 0, wxALL, 1); 

	// Do the experiment with the initial configuration
	wxCommandEvent evt;
	// OnButton(evt);
}

/* ********************************************************************************************* */
/// Determines the goal configuration of the left arm based on the cinder block pose, calls
/// analytical I.K. to find a suitable goal arm configuration and calls RRT routine to get the
/// appropriate trajectory. Finally, using the Timer routine, displays the trajectory.
void SimTab::OnButton(wxCommandEvent &evt) {

	// Setup the initial configuration
	if(evt.GetId() == 1234) {
		robot->setConfig(left_arm_ids, ql0);
		armSet = false;
		return;
	}

	// Get the normal and the location of the cinder block
	Eigen::Matrix4d cinderT = mWorld->getSkeleton("Cinder")->getNode("root")->getWorldTransform();
	Eigen::Vector3d normal = -cinderT.block<3,1>(0,2);
	Eigen::Vector3d hole (cinderT(0,3), cinderT(1,3), 0.44);
	hole += normal * 0.0945;
	cout << "hole: " << hole.transpose() << endl;
	cout << "normal: " << normal.transpose() << endl;

	// Compute the goal position
	Eigen::Matrix4d goalT = Eigen::Matrix4d::Identity();
	goalT.block<3,1>(0,3) = hole + 0.20 * normal;
	goalT.block<3,1>(0,2) = normal; 
	goalT.block<3,1>(0,0) = -Eigen::Vector3d(0,0,1);
	goalT.block<3,1>(0,1) = goalT.block<3,1>(0,2).cross(goalT.block<3,1>(0,0));
	cout << "goalT: \n" << goalT << endl;
	
	// Perform I.K. with different arm angles until no collisions
	Eigen::VectorXd ql = robot->getConfig(left_arm_ids);
	bool result = Krang::singleArmIKLimitsAndCollsBestWheel(mWorld, robot, goalT, false, 0.1, theta);
	if(!result) {
		cout << "Not reachable!" << endl;
		return;
	}
	cout << "Reachable, theta: " << theta.transpose() << endl;
	robot->setConfig(left_arm_ids, theta);
	armSet = true;
	return;

	// Perform RRT from the current configuration to the found IK contact configuration
	planning::PathPlanner <planning::RRT> planner (*mWorld);
	bool success = planner.planPath(robot, left_arm_ids, ql, theta, path);
	if(success) {
		cout << "Found path: " << path.size() << endl;
		planning::PathShortener shortener (mWorld, robot, left_arm_ids);
		shortener.shortenPath(path);
		cout << "Shortened path: " << path.size() << endl;
	}
	else cout << "Planner failed." << endl;
}

/* ********************************************************************************************* */
void SimTab::GRIPEventRender() {}

/* ********************************************************************************************* */
void SimTab::GRIPEventSimulationBeforeTimestep() {}

/* ********************************************************************************************* */
// Handler for events

BEGIN_EVENT_TABLE(SimTab, wxPanel)
	EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, SimTab::OnButton)
	EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, SimTab::OnSlider)
END_EVENT_TABLE()
/* ********************************************************************************************* */
