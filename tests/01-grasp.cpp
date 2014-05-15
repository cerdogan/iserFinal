/**
 * @file 01-grasp.cpp
 * @author Can Erdogan
 * @date Feb 15, 2014
 * @brief Perform analytical I.K. to get a joint configuration and then use RRT to find a
 * collision-free, shortened trajectory.
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

using namespace Krang;
using namespace std;

dynamics::SkeletonDynamics* krang;
list <Eigen::VectorXd> path;
Eigen::VectorXd ql0;

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
void Timer::Notify() {

	// Do the collision checking
	bool collision = mWorld->checkCollision(false);

	// Set the background
	if(collision) {
		glClearColor(1.0, 0.0, 0.0 ,1.0f);
		float fogCol[3] = {1.0, 0.0, 0.0};
		glFogfv(GL_FOG_COLOR,fogCol);
	}
	else viewer->setClearColor();
	viewer->DrawGLScene();
	
	// If there is no path, just wait
	if(path.empty()) {
		Start(0.01 * 1e3);	
		return;
	}

	// Traverse the path
	static size_t pathIdx = 0;
	krang->setConfig(left_arm_ids, *(path.begin()));
	path.pop_front();
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
	krang = mWorld->getSkeleton("Krang");
	Krang::setupKrangCollisionModel(mWorld, krang);
	ql0 = krang->getConfig(left_arm_ids);

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
		krang->setConfig(left_arm_ids, ql0);
		return;
	}

	// Get the normal and the location of the cinder block
	Eigen::Matrix4d cinderT = mWorld->getSkeleton("Cinder")->getNode("root")->getWorldTransform();
	Eigen::Vector3d normal = -cinderT.block<3,1>(0,2);
	Eigen::Vector3d hole (cinderT(0,3), cinderT(1,3), 0.44);
	hole += normal * 0.0945;
	cout << "hole: " << hole.transpose() << endl;
	cout << "normal: " << normal.transpose() << endl;
	exit(0);

	// Compute the goal position
	Eigen::Matrix4d goalT = Eigen::Matrix4d::Identity();
	goalT.block<3,1>(0,3) = hole + 0.07 * normal;
	goalT.block<3,1>(0,2) = normal; 
	goalT.block<3,1>(0,0) = -Eigen::Vector3d(0,0,1);
	goalT.block<3,1>(0,1) = goalT.block<3,1>(0,2).cross(goalT.block<3,1>(0,0));
	
	// Perform I.K. with different arm angles until no collisions
	Vector7d theta;
	Eigen::VectorXd ql = krang->getConfig(left_arm_ids);
	bool result = Krang::singleArmIKLimitsAndCollsBestWheel(mWorld, krang, goalT, false, 0.1, theta);
	if(!result) {
		cout << "Not reachable!" << endl;
		return;
	}
	cout << "Reachable, theta: " << theta.transpose() << endl;
	krang->setConfig(left_arm_ids, theta);
	return;

	// Perform RRT from the current configuration to the found IK contact configuration
	planning::PathPlanner <planning::RRT> planner (*mWorld);
	bool success = planner.planPath(krang, left_arm_ids, ql, theta, path);
	if(success) {
		cout << "Found path: " << path.size() << endl;
		planning::PathShortener shortener (mWorld, krang, left_arm_ids);
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
