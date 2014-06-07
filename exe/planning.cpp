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
	graph->add(PriorFactor < LieVector >(A0,LieVector(Eigen::Vector2d(-1.3, 1.00)), model2));

	// Add the distance constraints
	graph->add(Factors::Distance(A0, A1, plateLength, model));
	graph->add(Factors::Distance(B0, B1, plateLength, model));

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
	graph->add(Factors::Side(A1, B0, B1, false, model));
	graph->add(Factors::MinDistance(A1, B0, 0.04, true));

	// Set angle limits
	graph->add(Factors::LimitAngle(A0, A1, model));
	graph->add(Factors::LimitAngle(B0, B1, model));
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
/// Given a workspace velocity, returns the joint space velocity
VectorXd workToJointVelocity (kinematics::BodyNode* eeNode, const VectorXd& xdot) {

  // Get the Jacobian towards computing joint-space velocities
  MatrixXd Jlin = eeNode->getJacobianLinear().topRightCorner<3,7>();
  MatrixXd Jang = eeNode->getJacobianAngular().topRightCorner<3,7>();
  MatrixXd J (6,7);
  J << Jlin, Jang;

  // Compute the inverse of the Jacobian
  Eigen::MatrixXd Jt = J.transpose();
  Eigen::MatrixXd Jinv = Jt * (J * Jt).inverse();

  // Get the joint-space velocities by multiplying inverse Jacobian with x.
  VectorXd qdot = Jinv * xdot;
  return qdot;
}

/* ********************************************************************************************* */
void moveHand (const Eigen::VectorXd& dx, bool right, double limitSQ) {
	
	cout << "moving" << endl;
	// Get the initial hand location
	kinematics::BodyNode* eeNode = krang->getNode(right ? "rGripper" : "lGripper");
	Eigen::Vector3d init = eeNode->getWorldTransform().topRightCorner<3,1>();
	while(true) {

		// Stop if reached too far	
		Eigen::Vector3d curr = eeNode->getWorldTransform().topRightCorner<3,1>();
		double distSQ = (curr - init).squaredNorm();
		cout << "distSQ: " << distSQ << ", vs. limitSQ: " << limitSQ << endl;
		if(distSQ > limitSQ) break;

		// Update the joint values
		Eigen::VectorXd qdot = workToJointVelocity(eeNode, dx);
		qdot = qdot.normalized() * 0.05;
		Eigen::VectorXd q = krang->getConfig(right ? right_arm_ids : left_arm_ids);
		krang->setConfig(right ? right_arm_ids : left_arm_ids, q + qdot);
		viewer->DrawGLScene();
	}
	cout << "moving done" << endl;
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

  // If there is no path, just wait
  if(path.empty()) {

		if(started) {
			// Close the right hand
			collision::printVS = true;
			Eigen::VectorXd dx (6);
			dx << 0.0, (side ? -1.0 : 1.0), 0.0, 0.0, 0.0, 0.0;
			moveHand(dx, !side, 0.006);
			started = false;
		}

    Start(0.01 * 1e3);  
    return;
  }

	started = true;

  // Traverse the path
  static size_t pathIdx = 0;
	Eigen::VectorXd temp = *(path.begin());
  krang->setConfig(side ? right_arm_ids : left_arm_ids, temp);
  path.pop_front();
	path2.push_back(temp);

	// Update the manipulated object if one exists
	if(manipData != NULL) {

		// Get the object frame in the world frame
		const char* name = manipData->right ? "rGripper" : "lGripper";
		Eigen::Matrix4d wTh = krang->getNode(name)->getWorldTransform();
		Eigen::Matrix4d wTo = wTh * manipData->hTo;

		// Set the object dofs
		Eigen::VectorXd vals (6);
		vals.block<3,1>(0,0) = wTo.topRightCorner<3,1>();
		Eigen::Matrix3d R = wTo.topLeftCorner<3,3>();
		vals.block<3,1>(3,0) = math::matrixToEuler(R, math::XYZ);
		double temp = vals(5); vals(5) = vals(3); vals(3) = temp;
		mWorld->getSkeleton(manipData->objName)->setPose(vals);
	}

	// Update the scene
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
//  detector->deactivatePair(plate1->getNode("root"), plate2->getNode("root"));
//  detector->deactivatePair(plate1->getNode("root"), cinder1->getNode("root"));
//  detector->deactivatePair(plate1->getNode("root"), cinder2->getNode("root"));
 // detector->deactivatePair(plate2->getNode("root"), cinder1->getNode("root"));
 // detector->deactivatePair(plate2->getNode("root"), cinder2->getNode("root"));
  detector->deactivatePair(cinder1->getNode("root"), cinder2->getNode("root"));
  detector->deactivatePair(cinder1->getNode("root"), ground->getNode("ground"));
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

	// Move the hand
	Eigen::VectorXd dx (6);
	dx << 0.0, -1.0, 0.0, 0.0, 0.0, 0.0;
	moveHand(dx, true, 0.011);
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

	// Also, with points
	vector <int> tempIds;
	tempIds.push_back(0);
	tempIds.push_back(1);
	tempIds.push_back(2);
	const char* temp [] = {"A0", "A1", "B0", "B1", "C0", "C1"};
	for(size_t i = 0; i < 0; i++) {
		double x = result.at <LieVector>(i)(0), y = result.at <LieVector>(i)(1);
		Eigen::Vector3d conf(2.3, x, y);
		mWorld->getSkeleton(temp[i])->setConfig(tempIds, conf);
	}

	// Visualize the cinder blocks
	double x = result.at <LieVector>(box0)(0), y = result.at <LieVector>(box0)(1);
	Eigen::Vector3d conf(1.75, x, y);
	mWorld->getSkeleton("Cinder1")->setConfig(tempIds, conf);
	x = result.at <LieVector>(box1)(0), y = result.at <LieVector>(box1)(1);
	conf = Eigen::Vector3d (1.75, x, y);
	mWorld->getSkeleton("Cinder2")->setConfig(tempIds, conf);
	viewer->DrawGLScene();	
}

/* ********************************************************************************************* */
bool performIK (const Eigen::Vector3d& loc, double plate_angle, double phi, 
		Vector7d& qa, bool right) {

	// Get the initial arm configuration
	vector <int>& ids = right ? right_arm_ids : left_arm_ids;
	Eigen::VectorXd initial = krang->getConfig(ids);

	// Compute the goal position
	Eigen::Matrix4d goalT = Eigen::Matrix4d::Identity();
	goalT.block<3,1>(0,3) = loc;
	goalT.block<3,1>(0,0) = -Eigen::Vector3d(0,0,1);
	goalT.block<3,1>(0,1) = -Eigen::Vector3d(0,1,0); 
	goalT.block<3,1>(0,2) = -Eigen::Vector3d(1,0,0); 
	goalT.block<3,3>(0,0) = goalT.block<3,3>(0,0) * Eigen::AngleAxis<double>(M_PI - plate_angle, 
		Eigen::Vector3d(0.0, 0.0, 1.0)).matrix();
	// cout << "goalT:\n" << goalT << endl;
	// cout << "plate_angle:" << plate_angle << endl;
	
	// Make the IK call and return a bad error if cannot ik
	qa = Eigen::Matrix<double,7,1>::Zero();
	qa(0) = phi;
	Eigen::VectorXd qb = mWorld->getSkeleton("Krang")->getConfig(lower_dofs);
	bool result = singleArmIK(qb, goalT, right, qa);
	if(result) {

		// Set the configuration
		// cout << "qa: " << qa.transpose() << endl;
		krang->setConfig(ids, qa);

		// Check for collisions
		if(!mWorld->checkCollision(false)) {
			// cout << "successful\n\n\n" << endl;
			return true;
		}
		else {
			krang->setConfig(ids, initial);
			// cout << "collision" << endl;
		}
	}
	return false;
}

/* ********************************************************************************************* */
bool sampleGrasp (const char* plateName, bool right, Eigen::VectorXd& goal) {

 	//Get the top corner of the object
	Eigen::VectorXd plate_conf = mWorld->getSkeleton(plateName)->getConfig(plate_ids);
	Eigen::Vector3d pcorner = plate_conf.topRightCorner<3,1>();
	pcorner(0) -= 0.25;
	pcorner(1) -= cos(plate_conf(3)) * plateLength / 2;
	pcorner(2) -= sin(plate_conf(3)) * plateLength / 2;

	vector <int> tempIds;
	tempIds.push_back(0);
	tempIds.push_back(1);
	tempIds.push_back(2);

	// Keep sampling points along the edge until there is a collision-free inverse kinematics
	for(size_t sample_idx = 0; sample_idx < 100; sample_idx++) {

		// Uniform sampling
		double dist = (((double) rand()) / RAND_MAX) * plateLength;
		Eigen::Vector3d pcontact = pcorner;
		pcontact(1) += cos(plate_conf(3)) * dist;
		pcontact(2) += sin(plate_conf(3)) * dist;
		mWorld->getSkeleton("C0")->setConfig(tempIds, pcontact);
		viewer->DrawGLScene();	

		// Perform inverse kinematics for the contact point
		Eigen::Matrix<double, 7, 1> q_arm;
		for(double phi = 0.0; phi < 2*M_PI; phi += M_PI/4.0) {
			if(performIK(pcontact, plate_conf(3), phi, q_arm, right)) {
				goal = q_arm;
				viewer->DrawGLScene();	
				return true;
			}
		}
	}
	return false;
}

/* ********************************************************************************************* */
bool sampleGraspCinder (const char* cinderName, Eigen::VectorXd& goal) {

	// Hardcode all the grasps
	static const size_t numGrasps = 14;
	double grasps [][6] = {
		{0.075, 0.0975, -0.04, 0.0, M_PI, 0.0},
		{0.075, 0.2925, -0.04, 0.0, M_PI, 0.0},
		{-0.075, 0.0975, -0.04, 0.0, M_PI, 0.0},
		{-0.075, 0.2925, -0.04, 0.0, M_PI, 0.0},
		{0.0, 0.39, -0.04, 0.0, M_PI, M_PI_2},
		{0.0, 0.215, -0.04, 0.0, M_PI, M_PI_2},
		{0.0, 0.02, -0.04, 0.0, M_PI, M_PI_2},
		{0.0, 0.02, 0.04, 0.0, 0.0, M_PI_2},
		{-0.075, 0.2925, 0.04, 0.0, 0.0, 0.0},
		{-0.075, 0.0975, 0.04, 0.0, 0.0, 0.0},
		{0.075, 0.2925, 0.04, 0.0, 0.0, 0.0},
		{0.075, 0.0975, 0.04, 0.0, 0.0, 0.0},
		{0.0, 0.39, 0.04, 0.0, 0.0, M_PI_2},
		{0.0, 0.215, 0.04, 0.0, 0.0, M_PI_2}
	};

	// Find out which grasps are reachable
	double maxManip = -1.0;;
	Eigen::VectorXd bestConf;
	static const bool right = true;
	for(size_t grasp_idx = 0; grasp_idx < numGrasps; grasp_idx++) {

		// Get the local transform
		Eigen::Vector3d rpy;
		for(size_t i = 3; i < 6; i++) 
			rpy(i-3) = grasps[grasp_idx][i];
		Eigen::Matrix4d oTh = Eigen::Matrix4d::Identity(); 
		oTh.topLeftCorner<3,3>() = math::eulerToMatrix(rpy, math::XYZ);
		oTh.topRightCorner<3,1>() << grasps[grasp_idx][0], grasps[grasp_idx][1], 
			grasps[grasp_idx][2];
		
		// Get the goal hand transform
		Eigen::Matrix4d wTo = mWorld->getSkeleton(cinderName)->getNode("root")->getWorldTransform();
		Eigen::Matrix4d wTh = wTo * oTh;

		// Perform IK
		Eigen::Matrix<double,7,1> qa = Eigen::Matrix<double,7,1>::Zero();
		Eigen::VectorXd qb = mWorld->getSkeleton("Krang")->getConfig(lower_dofs);
		bool result = singleArmIK(qb, wTh, right, qa);
		if(!result) {
			cout << "grasp_idx: " << grasp_idx << " failed" << endl;
			continue;
		}
		krang->setConfig(right ? right_arm_ids : left_arm_ids, qa);

		// Compute the manipulability	
		kinematics::BodyNode* eeNode = krang->getNode(right ? "rGripper" : "lGripper");
		MatrixXd Jlin = eeNode->getJacobianLinear().topRightCorner<3,7>();
		MatrixXd Jang = eeNode->getJacobianAngular().topRightCorner<3,7>();
		MatrixXd J (6,7);
		J << Jlin, Jang;
		Eigen::MatrixXd Jt = J.transpose();
		double manip = sqrt((J*Jt).determinant());

		// See if this solution is the best
		if(manip > maxManip) {
			maxManip = manip;
			bestConf = qa;
		}
	}

	// Return the best configuration if one is found
	goal = bestConf;
	if(maxManip != -1) {
		krang->setConfig(right ? right_arm_ids : left_arm_ids, goal);
		return true;
	}
	return false;
}

/* ********************************************************************************************* */
void motionPlanCinder (bool first) {

	const char* plateName= first ? "Cinder2" : "Cinder1";
	const char* handName= "lGripper";

	// Find a good initial grasp for the second plate
	Eigen::VectorXd graspPose (7);
	bool successful = sampleGraspCinder(plateName, graspPose);
	cout << "successful: " << successful << endl;
	if(!successful) return;
	return;

	// Remove the blue object
	Eigen::VectorXd temp (6);
	temp(2) = 10.0;
	mWorld->getSkeleton("C0")->setPose(temp);

	// Compute the transformation between the hand and the plate to use it in motion planning
	Eigen::Matrix4d wTh = krang->getNode(handName)->getWorldTransform();
	Eigen::Matrix4d wTo = mWorld->getSkeleton(plateName)->getNode("root")->getWorldTransform();
	Eigen::Matrix4d hTo = wTh.inverse() * wTo;

	// Create the manipulation data to be used in motion planning
	manipData = new ManipData();
	sprintf(manipData->objName, "%s", plateName);
	manipData->right = right;
	manipData->hTo = hTo;

	// Move the arm to the initial configuration
	planning::PathPlanner <ManipRRT> planner (*mWorld);
	Eigen::VectorXd goalConf (7);
	if(right) {
		goalConf = -defaultConf;
		goalConf(6) += 2*M_PI;
	}
	else goalConf = defaultConf;
	vector <int>& ids = right ? right_arm_ids : left_arm_ids;
  bool success = planner.planPath(krang, ids, graspPose, goalConf, path);
	planner.maxNodes = 5000;
  if(success) {
    cout << "Found path: " << path.size() << endl;
    ManipShortener shortener (mWorld, krang, ids);
    shortener.shortenPath(path);
    cout << "Shortened path: " << path.size() << endl;
  }
  else cout << "Planner failed." << endl;
}


/* ********************************************************************************************* */
void motionPlanPlates (bool right) {

	const char* plateName= right ? "Plate1" : "Plate2";
	const char* handName= right ? "rGripper" : "lGripper";

	// Find a good initial grasp for the second plate
	Eigen::VectorXd graspPose (7);
	bool successful = sampleGrasp(plateName, right, graspPose);
	cout << "successful: " << successful << endl;
	if(!successful) return;

	// Remove the blue object
	Eigen::VectorXd temp (6);
	temp(2) = 10.0;
	mWorld->getSkeleton("C0")->setPose(temp);

	// Compute the transformation between the hand and the plate to use it in motion planning
	Eigen::Matrix4d wTh = krang->getNode(handName)->getWorldTransform();
	Eigen::Matrix4d wTo = mWorld->getSkeleton(plateName)->getNode("root")->getWorldTransform();
	Eigen::Matrix4d hTo = wTh.inverse() * wTo;

	// Create the manipulation data to be used in motion planning
	manipData = new ManipData();
	sprintf(manipData->objName, "%s", plateName);
	manipData->right = right;
	manipData->hTo = hTo;

	// Move the arm to the initial configuration
	planning::PathPlanner <ManipRRT> planner (*mWorld);
	Eigen::VectorXd goalConf (7);
	if(right) {
		goalConf = -defaultConf;
		goalConf(6) += 2*M_PI;
	}
	else goalConf = defaultConf;
	vector <int>& ids = right ? right_arm_ids : left_arm_ids;
  bool success = planner.planPath(krang, ids, graspPose, goalConf, path);
	planner.maxNodes = 5000;
  if(success) {
    cout << "Found path: " << path.size() << endl;
    ManipShortener shortener (mWorld, krang, ids);
    shortener.shortenPath(path);
    cout << "Shortened path: " << path.size() << endl;
  }
  else cout << "Planner failed." << endl;
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
	else if(evt.GetId() == 1234) motionPlanPlates(false);
	else if(evt.GetId() == 1235) {

		// Move the hand
		if(!offsetRightArm) {
			Eigen::VectorXd dx (6);
			dx << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
	//		Eigen::VectorXd bla = Eigen::VectorXd::Zero(7);
	//		krang->setConfig(left_arm_ids, bla);
			moveHand(dx, false, 0.011);
			offsetRightArm = true;
		}

		// Remove the other plate
		side = true;
		Eigen::VectorXd temp (6);
		temp(2) = 12.0;
		mWorld->getSkeleton("Plate2")->setPose(temp);
		motionPlanPlates(true);
	}
	else if(evt.GetId() == 1236) {

		// Reset the arms
		krang->setConfig(left_arm_ids, defaultConf);
		krang->setConfig(right_arm_ids, -defaultConf);

		// Remove the other plate
		side = true;
		Eigen::VectorXd temp (6);
		temp(2) = 14.0;
		mWorld->getSkeleton("Plate1")->setPose(temp);
		temp(2) = 12.0;
		mWorld->getSkeleton("Plate2")->setPose(temp);

		// Move the robot close
		Eigen::Vector2d temp2 (1.5, 2.7);
		vector <int> fudge;
		fudge.push_back(0);
		fudge.push_back(8);
		krang->setConfig(fudge, temp2);

		// Do the motion plan
		motionPlanCinder(true);
	}

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


