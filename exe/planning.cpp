/**
 * @file 33-ramp.cpp
 * @author Can Erdogan
 * @date Mar 11, 2014
 * @brief Demonstrates the construction of a recursive ramp structure. 
 */

#define private public
#define protected public

#include "ramp.h"
#include "basic.h"
#include "lever.h"
#include "display.h"
#include "ik.h"
#include "motion.h"

#include <collision/fcl_mesh/FCLMESHCollisionDetector.h>

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#define SQ(x) ((x) * (x))

typedef gtsam::NonlinearFactorGraph Graph;  
collision::FCLMESHCollisionDetector* detector;

using namespace std;
using namespace gtsam;

int _ld [] = {11, 14, 16, 18, 20, 22, 24}; 
vector <int> left_arm_ids (_ld, _ld + sizeof(_ld) / sizeof(int));
int _rd [] = {12, 15, 17, 19, 21, 23, 25}; 
vector <int> right_arm_ids (_rd, _rd + sizeof(_rd) / sizeof(int));
Eigen::VectorXd defaultConf (7);		//< for the left arm while holding the plate in front
list <Eigen::VectorXd> path;
list <Eigen::VectorXd> path2;
bool movedPlate2 = false;
bool side = false;
bool offsetLeftArm = false, offsetRightArm = false;

/* ******************************************************************************************** */
/// Application class
extern wxNotebook* tabView;
class mainApp : public GRIPApp {
	virtual bool OnInit() {
		bool res = GRIPApp::OnInit();
		return res;
	}
	virtual void AddTabs() {
		tabView->AddPage(new SimTab(tabView), wxT("Inverse Kinematics"));
	}
};

IMPLEMENT_DYNAMIC_CLASS(SimTab, GRIPTab)
IMPLEMENT_APP(mainApp)

/* ******************************************************************************************** */
Values result, lastResult;
static SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector_(1, 1e-3));
static SharedNoiseModel model2 = noiseModel::Diagonal::Sigmas(Vector_(2, 1e-3, 1e-3));
static const double plateLength = 1.2192;
static const double plate2Length = 0.889;

enum Keys {
	A0 = 0,
	A1,
	B0,
	B1,
	C0,
	C1,
	box0,
	box1
}; 

/* ******************************************************************************************** */
/// Creates a factor graph that represents a set of ramps where the left tip of each ramp either 
/// lies on the middle of the previous ramp or on the given blocks top-right corner. 
Graph* createGraph() {

	// The tip of the first plate needs to reach the tower
	Graph* graph = new Graph;
	graph->add(PriorFactor < LieVector >(A0,LieVector(Eigen::Vector2d(-1.3, 1.05)), model2));

	// Add the distance constraints
	graph->add(Factors::Distance(A0, A1, plateLength, model));
	graph->add(Factors::Distance(B0, B1, plate2Length, model));

	// Place the ground constraints
	graph->add(Factors::Prior1D(A1, false, 0.39, model));
	graph->add(Factors::Prior1D(B1, false, 0.03, model));

	// Place the A1 on a block
	graph->add(Factors::OnSurface(A1, box1, model));
	graph->add(Factors::OnSurface(box1, box0, model));
	graph->add(Factors::Prior1D(box1, false, 0.2850, model));
	graph->add(Factors::Prior1D(box0, false, 0.0975, model));

	// Make sure the top end-points are above ground
	graph->add(Factors::Bounding2D(A0, false, 0.03, true));
	graph->add(Factors::Bounding2D(B0, false, 0.03, true));

	// Add an alignment between consecutive plates
	graph->add(Factors::Alignment(B0, B1, box1, Eigen::Vector2d(0.0975, 0.0975), model2));
	
	// Make sure the structure is from left to right
	graph->add(Factors::Ordering(A0, A1, true, .15, true));
	graph->add(Factors::Ordering(A0, B0, true, .15, true));
	graph->add(Factors::Ordering(B0, B1, true, .15, true));

	// Avoid collisions
//	graph->add(Factors::Side(A1, B0, B1, false, model));
//	graph->add(Factors::MinDistance(A1, B0, 0.04, true));

	// Set angle limits
	graph->add(Factors::LimitAngle(A0, A1, model));
	graph->add(Factors::LimitAngle2(B0, B1, model));
	graph->add(Factors::LimitAngle(C0, C1, model));
	
	// Add the bounding constraint for the water for the last stick
	graph->add(Factors::Bounding2D(C0, true, 0.0, model));
	return graph;
}

/* ******************************************************************************************** */
/// Creates a random vector within the given limits
static Vector randomVector(const Vector& minLimits, const Vector& maxLimits) {
	size_t numDims = dim(minLimits);
	Vector vector = zero(numDims);
	for(size_t i = 0; i < numDims; i++) {
		double range = maxLimits(i) - minLimits(i);
		vector(i) = (((double) rand()) / RAND_MAX) * range + minLimits(i);
	}
	return vector;
}

/* ******************************************************************************************** */
/// Optimizes for the given graph with random restarts where the initial values are sampled
/// from the given bounding box limit.
static Values solveGraph(Graph& graph, double maxValue, size_t numIterations) {

	// Try different instantiations
	size_t attempt;
	size_t numKeys = graph.keys().size();
	for(attempt = 0; attempt < numIterations; attempt++) {

		// Set random values for initialization
		Values values;
		Vector minLimits = zero(numKeys * 2), maxLimits = maxValue * ones(numKeys * 2);
		Vector random = randomVector(minLimits, maxLimits);
		for(size_t i = 0; i < numKeys; i++)
			values.insert(i, LieVector(maxValue * Eigen::Vector2d::Random()));

		// Optimize
		LevenbergMarquardtParams params;
		params.absoluteErrorTol = 1e-15;
		params.relativeErrorTol = 1e-15;
		Values result;
		double error;
		LevenbergMarquardtOptimizer optimizer(graph, values, params);
		result = optimizer.optimize();
		error = graph.error(result);
			
		// Stop if a zero error is attained.
		if(!result.empty() && error < 1e-2) return result;
	}

	return Values();
}

/* ********************************************************************************************* */
void Timer::Notify() {

	collision::printVS = true;

  // Do the collision checking
	static bool started = false;
  bool collision = mWorld->checkCollision(false);

  // Set the background
  if(collision) {
    glClearColor(1.0, 0.0, 0.0 ,1.0f);
    float fogCol[3] = {1.0, 0.0, 0.0};
    glFogfv(GL_FOG_COLOR,fogCol);
  }
  else viewer->setClearColor();
  viewer->DrawGLScene();
  Start(0.01 * 1e3);  
}

/* ********************************************************************************************* */
vector <int> plate_ids;
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
		long style) : GRIPTab(parent, id, pos, size, style) {

	// Setup environment
	int seed = time(NULL);
	cout << "seed: " << seed << endl;
	srand(seed);
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* sizerFull = new wxBoxSizer (wxHORIZONTAL);
	SetSizer(sizerFull);
	frame->DoLoad("/etc/kore/scenes/13-World-Plates.urdf");
	krang = mWorld->getSkeleton("Krang");
	wxGetApp().setConfiguration("/etc/kore/scenes/configs/01-standUp.config");

	// Add button
	sizerFull->Add(new wxButton(this, 1233, wxT("Plan!")), 0, wxALL, 1); 
	sizerFull->Add(new wxButton(this, 1234, wxT("Plate2!")), 0, wxALL, 1); 
	sizerFull->Add(new wxButton(this, 1235, wxT("Plate1!")), 0, wxALL, 1); 
	sizerFull->Add(new wxButton(this, 1236, wxT("Cinder2!")), 0, wxALL, 1); 

	// Set background to white
  viewer->backColor = Vector3d(1,1,1);
  viewer->gridColor = Vector3d(.8,.8,1);
  viewer->setClearColor();

	// Set the camera angle
 	viewer->camRadius = 5.00;
	viewer->UpdateCamera();
	viewer->DrawGLScene();	

	// Setup dofs of plates
	plate_ids.push_back(0);
	plate_ids.push_back(1);
	plate_ids.push_back(2);
	plate_ids.push_back(4);

	// Set the timer
	timer = new Timer();
	timer->Start(1);

	// Disable some of the object collision pairs
  krang->setSelfCollidable(true);
  dynamics::ContactDynamics* constraints = mWorld->mCollisionHandle;
	dynamics::SkeletonDynamics* plate1 = mWorld->getSkeleton("Plate1"), 
		*plate2 = mWorld->getSkeleton("Plate2"), *cinder1 = mWorld->getSkeleton("Cinder1"),  
		*cinder2 = mWorld->getSkeleton("Cinder2"), *ground = mWorld->getSkeleton("ground");
  detector = (collision::FCLMESHCollisionDetector*) (constraints->mCollisionChecker);
  detector->deactivatePair(cinder1->getNode("root"), cinder2->getNode("root"));
  detector->deactivatePair(cinder1->getNode("root"), ground->getNode("ground"));
  detector->deactivatePair(cinder2->getNode("root"), ground->getNode("ground"));
  detector->deactivatePair(krang->getNode("Base"), ground->getNode("ground"));
  detector->deactivatePair(krang->getNode("LWheel"), ground->getNode("ground"));
  detector->deactivatePair(krang->getNode("RWheel"), ground->getNode("ground"));
  detector->deactivatePair(krang->getNode("R4"), krang->getNode("rGripper"));
  detector->deactivatePair(krang->getNode("R5"), krang->getNode("rGripper"));
  detector->deactivatePair(krang->getNode("L4"), krang->getNode("lGripper"));
  detector->deactivatePair(krang->getNode("L5"), krang->getNode("lGripper"));

	// Set the arm conf while holding the plate in front
	defaultConf << 0.7845, -0.34907, 0.0, -1.0472, 0.0, -1.7453, M_PI_2;
	krang->setConfig(left_arm_ids, defaultConf);
	krang->setConfig(right_arm_ids, -defaultConf);

	// Set the obstacle configuration
	Eigen::VectorXd fuckTard = mWorld->getSkeleton("Obstacle")->getPose();
	fuckTard(2) = 0.1;
	fuckTard(0) = 1.55;
	fuckTard(1) = -1.29;
	mWorld->getSkeleton("Obstacle")->setPose(fuckTard);
}

/* ********************************************************************************************* */
void createAndVisualizeDesign() {

	path.clear();
	path2.clear();

	// Reset the plate confs
	Eigen::VectorXd reset = Eigen::VectorXd::Zero(6);
	reset(3) = -M_PI_2;
	mWorld->getSkeleton("Plate1")->setPose(reset);
	mWorld->getSkeleton("Plate2")->setPose(reset);

	// Optimize the result with random initializations
	Graph* graph = createGraph();
	result = solveGraph(*graph, 6.0, 10000);

	// Visualize the result for plates
	for(size_t i = 0; i < 2; i++) {
		char buf [16];
		sprintf(buf, "Plate%lu", i+1);
		SkeletonDynamics* plate = mWorld->getSkeleton((const char*) buf);
		double x1 = result.at <LieVector>(2*i)(0), y1 = result.at <LieVector>(2*i)(1);
		double x2 = result.at <LieVector>(2*i+1)(0), y2 = result.at <LieVector>(2*i+1)(1);
		Eigen::VectorXd conf (4);
		conf(0) = 1.9;
		conf(1) = (x1 + x2) / 2.0;
		conf(2) = (y1 + y2) / 2.0;
		conf(3) = atan2((y2 - y1),(x2 - x1));
		cout << "Setting plate" << i << ": " << conf.transpose() << endl;
		plate->setConfig(plate_ids, conf);
	}

	vector <int> tempIds;
	tempIds.push_back(0);
	tempIds.push_back(1);
	tempIds.push_back(2);

	// Visualize the cinder blocks
	double x = result.at <LieVector>(box1)(0), y = result.at <LieVector>(box1)(1);
	Eigen::Vector3d conf(1.75, x, y);
	mWorld->getSkeleton("Cinder1")->setConfig(tempIds, conf);

	tempIds.push_back(3);
	double th = ((double) rand()) / RAND_MAX;
	x = result.at <LieVector>(box0)(0), y = result.at <LieVector>(box0)(1);
	Eigen::Vector4d conf2 = Eigen::Vector4d (1.95, x, y-0.0975, th);
	mWorld->getSkeleton("Cinder2")->setConfig(tempIds, conf2);
	viewer->DrawGLScene();	
}

/* ********************************************************************************************* */
void SimTab::GRIPEventSimulationBeforeTimestep() { }
void SimTab::GRIPEventRender() { }
void SimTab::OnButton(wxCommandEvent &evt) {

	// Get the initial time
  struct timeval start, end;
	long mtime, seconds, useconds;    
	gettimeofday(&start, NULL);

	// Make the design
	if(evt.GetId() == 1233) createAndVisualizeDesign();

	// Get the end time
	gettimeofday(&end, NULL);
	seconds  = end.tv_sec  - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	printf("Elapsed time for %s: %ld milliseconds\n", 
		evt.GetId() == 1233 ? "'making desing'" : "'motion planning'", mtime);
}

/* ********************************************************************************************* */
// Handler for events

BEGIN_EVENT_TABLE(SimTab, wxPanel)
	EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, SimTab::OnButton)
	EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, SimTab::OnSlider)
END_EVENT_TABLE()


