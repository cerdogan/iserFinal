/**
 * @file 09-push.cpp
 * @author Can Erdogan
 * @date Feb 16, 2014
 * @brief Pushing the lever task: (1) back up away from the design on the ground, (2) fix the arms,
 * (3) stand up, (4) increase waist angle, (5) which induces another balance state with different
 * gains, wait there for a little while to stabilize, (6) get vision date about the tip of the
 * lever, (7) reach out to tip, (8) grasp, (9) go down wait waist until 165 degs and (10)
 * balance low (just like stand up).
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

fstream mylog;

// initializers for the workspace control constants
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.1;
const double DAMPING_GAIN = 0.005;
Krang::WorkspaceControl* workspace;
Krang::Vector7d nullspace_q_ref; ///< nullspace configurations for the arms
Krang::Vector7d nullspace_q_mask; ///< nullspace configurations for the arms

Krang::Hardware* krang;                                   ///< connects to hardware
somatic_d_t daemon_cx;        
simulation::World* world;
dynamics::SkeletonDynamics* robot;
ach_channel_t chan;
ach_channel_t cinder_chan;

Vector7d K_stand, K_bal;			// extra for com offset
Vector6d state0;	
bool dbg = false;
bool start = false;

Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();		

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

enum Step {
	STEP_BACKUP = 0,
	STEP_FIXARMS,
	STEP_STANDUP,
	STEP_WAIST,
	STEP_STABILIZE,
	STEP_VISION,
	STEP_REACHOUT,
	STEP_GRASP,
	STEP_PUSH,
	STEP_REST
};

enum CtrlMode {
	CTRL_GROUND = 0,
	CTRL_STAND,
	CTRL_BALANCE,
	CTRL_SIT
};

map <Step, CtrlMode> stepCtrlModes;
Step step;
CtrlMode ctrlMode;
bool changePermission = false;

/* ******************************************************************************************** */
/// Finds the consensus in the data and interprets it to get the motion parameters. Returns the
/// consensus data
Eigen::VectorXd analyzeKinectData (const std::vector <Eigen::VectorXd>& data, bool extra) {

	static const bool debug = 0;

	// Get data ~50 times and make a histogram of results
	vector <pair<Vector6d, size_t> > meanValues;
	for(int data_idx = 0; data_idx < data.size(); data_idx++) {

		// Find the data point most closed to if one exists
		Eigen::VectorXd dataPoint = data[data_idx];
		bool foundOne = false;
		for(size_t mean_idx = 0; mean_idx < meanValues.size(); mean_idx++) {
			pair <Vector6d, size_t>& meanPair = meanValues[mean_idx];
			double distSQ = (meanPair.first - dataPoint).squaredNorm();
			if(distSQ < 0.1) {
				foundOne = true;
				meanPair.first = (meanPair.first * meanPair.second + dataPoint) / (meanPair.second + 1);
				meanPair.second++;
			}
		}

		// If could not find any data points close by, add it to the list
		if(!foundOne) meanValues.push_back(make_pair(dataPoint, 1));
	}
		
	// Find the most popular cluster
	size_t maxSize = 0, bestMeanIdx = 0;
	if(debug) cout << "Clusters: \n";  
	for(size_t mean_idx = 0; mean_idx < meanValues.size(); mean_idx++) {
		if(debug) cout << "\tcluster: " << mean_idx << ", size: " << meanValues[mean_idx].second 
			<< ", mean: " << meanValues[mean_idx].first.transpose() << endl;
		if(meanValues[mean_idx].second > maxSize) {
			maxSize = meanValues[mean_idx].second;
			bestMeanIdx = mean_idx;
		}
	}
	
	// Set the hole location and the normal
	Eigen::Vector3d hole = meanValues[bestMeanIdx].first.topLeftCorner<3,1>();
	Eigen::Vector3d normal = meanValues[bestMeanIdx].first.bottomLeftCorner<3,1>().normalized();
	if(extra) {
		hole(2) = 0.455;
		Eigen::Vector3d perp (-normal(1), normal(0), 0.0);
		hole -= perp * 0.06;
	}

	// Set the output
	Eigen::VectorXd output (6);
	output.block<3,1>(0,0) = hole;
	output.block<3,1>(3,0) = normal;
	return output;
}

/* ******************************************************************************************** */
bool getVecData (ach_channel_t& chan, Eigen::VectorXd& data) {

	// Get data
	struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );
	int result;
	size_t numBytes = 0;
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &chan, &numBytes, 
		&abstimeout, ACH_O_WAIT, &result);
	if(numBytes == 0) return false;

	// Read the message
	Somatic__Vector* msg = somatic__vector__unpack(&(daemon_cx.pballoc), numBytes, 
		buffer);
	for(size_t i = 0; i < 3; i++) data(i) = msg->data[i];
	for(size_t i = 0; i < 3; i++) data(i+3) = msg->data[i];
	return true;
}

/* ******************************************************************************************** */
const char* stepName (Step step) {
	switch (step) {
		case STEP_BACKUP: return "STEP_BACKUP";
		case STEP_FIXARMS: return "STEP_FIXARMS";
		case STEP_STANDUP: return "STEP_STANDUP";
		case STEP_WAIST: return "STEP_WAISTJ";
		case STEP_STABILIZE: return "STEP_STABILIZE";
		case STEP_VISION: return "STEP_VISION";
		case STEP_REACHOUT: return "STEP_REACHOUT";
		case STEP_GRASP: return "STEP_GRASP";
		case STEP_PUSH: return "STEP_PUSH";
		case STEP_REST: return "STEP_REST";
		default: break;
	};
}

/* ******************************************************************************************** */
const char* ctrlModeName (CtrlMode ctrlMode) {
	switch (ctrlMode) {
		case CTRL_GROUND: return "CTRL_GROUND";
		case CTRL_STAND: return "CTRL_STAND";
		case CTRL_BALANCE: return "CTRL_BALANCE";
		case CTRL_SIT: return "CTRL_SIT";
		default: break;
	};
}

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {

	// Get the gains
	Vector7d* kgains [] = {&K_stand, &K_bal};
	ifstream file ("/home/cerdogan/Documents/Software/project/krang/iser/data/gains-09.txt");
	assert(file.is_open());
	char line [1024];
	for(size_t k_idx = 0; k_idx < 2; k_idx++) {
		*kgains[k_idx] = Vector7d::Zero();
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
	delete workspace;
	delete krang;
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
		else if(input >= '0' && input <= '9') step = (Step) (input - 48);
		else if(input=='r') readGains();
		else if(input=='c') changePermission = true;
		pthread_mutex_unlock(&mutex);
	}
}

/* ******************************************************************************************** */
void computeTorques (const Vector6d& state, double& ul, double& ur) {

	// Set reference based on the mode
	Vector6d refState;
	if(dbg) DISPLAY_VECTOR(state0);
	if(ctrlMode == CTRL_STAND || ctrlMode == CTRL_BALANCE) 
		refState << 0.0, 0.0, state0(2), 0.0, state0(4), 0.0;
	else {
		ul = ur = 0.0;
		return;
	}
	if(dbg) DISPLAY_VECTOR(refState);

	// Set the gains
	Vector6d K;
	if(ctrlMode == 1) K = K_stand.block<6,1>(0,0);
	else if(ctrlMode == 2) K = K_bal.block<6,1>(0,0);
	else assert(false);
	if(dbg) DISPLAY_VECTOR(K);

	// Compute the error
	Vector6d error = state - refState;
	if(dbg) DISPLAY_VECTOR(error);

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
  krang->updateSensors(dt);
	cout << "ft: " << krang->fts[LEFT]->lastExternal.transpose() << endl;

	// Calculate the COM of the body
	Eigen::Vector3d com = robot->getWorldCOM();
	com(2) -= 0.264;
	if(ctrlMode == CTRL_BALANCE) com(0) += K_bal(6); 
	else if(ctrlMode == CTRL_STAND) com(0) += K_stand(6); 
	if(dbg) cout << "com: " << com.transpose() << endl;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	state(0) = atan2(com(0), com(2));
	state(1) = krang->imuSpeed;
	state(2) = (krang->amc->pos[0] + krang->amc->pos[1]) / 2.0 + krang->imu;
	state(3) = (krang->amc->vel[0] + krang->amc->vel[1]) / 2.0 + krang->imuSpeed;
	state(4) = (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
	state(5) = (krang->amc->vel[1] - krang->amc->vel[0]) / 2.0;
}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// Get the initial state and set the backup values
	krang->updateSensors(0.0);
	Eigen::Vector2d curr (krang->amc->pos[0], krang->amc->pos[1]); 
	state0 << 0.0, 0.0, (curr(0) + curr(1)) / 2.0 + krang->imu, 0.0, 
		(curr(1) - curr(0)) / 2.0, 0.0;
	Eigen::Vector2d desiredBackVals = curr - Eigen::Vector2d(0.6, 0.6);
	
	// start some timers
	Vector6d state;	
	size_t c_ = 0;
	int balancedCounter = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	vector <Eigen::VectorXd> data;
	Eigen::VectorXd visionData;
	step = STEP_VISION;
	while(!somatic_sig_received) {

		// Prepare for this loop 
		pthread_mutex_lock(&mutex);
		dbg = (c_++ % 20 == 0);
		if(dbg) cout << "\nstep: " << stepName(step) << endl;
		ctrlMode = stepCtrlModes[step];
		if(dbg) cout << "\nctrlMode: " << ctrlModeName(ctrlMode) << endl;
		
		// Update times
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Get the locomotion state 
		getState(state, dt); 
		if(dbg) DISPLAY_VECTOR(state);

		// Compute the torques based on the state and the mode
		double ul, ur;
		computeTorques(state, ul, ur);
	
		// Apply the torque
		double input [2] = {ul, ur};
		if(dbg) cout << "start: " << start << "\nu: {" << ul << ", " << ur << "}" << endl;
		if(!start) input[0] = input[1] = 0.0;
	//	if((step != STEP_BACKUP) && (step != STEP_REST))
	//		somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2,NULL);

		// Decide on what to do specifically for this step
		switch(step) {

/*
			case STEP_BACKUP: {
				// Check if reached desired position
				Eigen::Vector2d curr (krang->amc->pos[0], krang->amc->pos[1]); 
				if(dbg) DISPLAY_VECTOR(curr);
				if(dbg) DISPLAY_VECTOR(desiredBackVals);
				if((curr(0) < desiredBackVals(0)) && (curr(1) < desiredBackVals(1))) {
					double u [] = {0.0, 0.0};
					somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, u, 2,NULL);
					state0(2) = (curr(0) + curr(1)) / 2.0 + krang->imu;
					state0(4) = (curr(1) - curr(0)) / 2.0;
					if(changePermission) {
						step = STEP_FIXARMS;
						changePermission = false;
					}
					else if(dbg) cout << "\033[1;31mMoving the left arm to front pose. Ready?" << "?\033[0m\n" <<endl;
				}
				else {
					// Go backward slowly 
					double u [] = {-10.5, -10.5};
					if(start) somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, u, 2,NULL);
				}
			} break;

			case STEP_FIXARMS: {
				Vector7d goal;
			//	goal << 1.400, -1.000, 0.000, -0.800, 0.000, -0.800, 0.000;
//				goal << 1.083,  -1.000,   0.000,  -1.382,   0.007,   0.852,  -0.575;
				goal << 1.083,  -0.451,   0.074,  -1.999,   0.007,   0.852,  -0.575;
				somatic_motor_setpos(&daemon_cx, krang->arms[Krang::LEFT], goal.data(), 7);
				bool reached = true;
				for(size_t i = 0; i < 7; i++) {
					if(fabs(goal(i) - krang->arms[LEFT]->pos[i]) > 0.005) {
						reached = false;
						break;
					}
				}
				if(reached) {
					if(changePermission) {
						step = STEP_STANDUP;
						changePermission = false;
					}
					else if(dbg) cout << "\033[1;31mReady to standup? " << "?\033[0m\n" << endl;
				}
			} break;

			case STEP_STANDUP: {
				if(fabs(state(0)) < 0.064) balancedCounter++;	
				if((balancedCounter > 300)) {
					if(changePermission) {
						balancedCounter = 0;
						step = STEP_WAIST;
						changePermission = false;
					}
					else if(dbg) cout << "\033[1;31mReady to move the waist? " << "?\033[0m\n" << endl;
				}
			} break;
		
			case STEP_WAIST: {

				// Send a current value
				double current = -1.8;
				somatic_vector_set_data(waistDaemonCmd->data, &current, 1);
				int r = SOMATIC_PACK_SEND(krang->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
				if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
					ach_result_to_string(static_cast<ach_status_t>(r)));

				// Check for stop condition
				if(krang->waist->pos[0] < 2.26) {
					somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
					int r = SOMATIC_PACK_SEND(krang->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
					if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
						ach_result_to_string(static_cast<ach_status_t>(r)));
					step = STEP_STABILIZE;
				}
			} break;

			case STEP_STABILIZE: {
				if(fabs(state(0)) < 0.064) balancedCounter++;	
				if(balancedCounter > 200) {
					if(changePermission) {
						balancedCounter = 0;
						step = STEP_VISION;
						changePermission = false;
					} 
					else if(dbg) cout << "\033[1;31mReady to start vision?" << "?\033[0m\n" << endl;
				}
			} break;

*/
			case STEP_VISION: {

				// Collect the data - make sure the cinder block is detected
				Eigen::VectorXd dataPoint (6);
				if(getVecData(chan, dataPoint)) {
					if(dataPoint(0) < 9.99) 
						data.push_back(dataPoint);
				}
				if(dbg) DISPLAY_SCALAR(data.size());

				if(data.size() > 200) {
					visionData = analyzeKinectData(data, false);
					if(changePermission) {
						data.clear();
						step = STEP_REACHOUT;
						changePermission = false;
					}
					else if(dbg) {
						DISPLAY_VECTOR(visionData);
						cout << "\033[1;31mReady to reach out? " << "?\033[0m\n" << endl;
					}
				}
			} break;

			case STEP_REACHOUT: {

				static bool reached = false;

				// Set the workspace direction
				Eigen::Vector3d handPos = 
					robot->getNode("lGripper")->getWorldTransform().topRightCorner<3,1>();
				if(dbg) DISPLAY_VECTOR(handPos);
				Vector6d xdot = Vector6d::Zero();
				Eigen::Vector3d posErr = visionData.block<3,1>(0,0) - handPos;
				if(dbg) DISPLAY_VECTOR(visionData);

				if(dbg) DISPLAY_VECTOR(krang->fts[Krang::LEFT]->lastExternal);
				double magn = krang->fts[Krang::LEFT]->lastExternal.topLeftCorner<3,1>().norm();

				if(reached || posErr.norm() < 0.05 || (magn > 30)) {
					reached = true;
					cout << "Reached here" << endl;
					Vector7d zero = Vector7d::Zero();
					somatic_motor_setvel(&daemon_cx, krang->arms[Krang::LEFT], zero.data(), 7);
					if(changePermission) {
						step = STEP_GRASP;
						changePermission = false;
						pthread_mutex_unlock(&mutex);
						continue;
					}
					else if(dbg) cout << "\033[1;31mReady to grasp? " << "?\033[0m\n" << endl;
					pthread_mutex_unlock(&mutex);
					continue;
				} 
				else xdot.block<3,1>(0,0) = posErr.normalized();

				// Nullspace: construct a qdot that the jacobian will bias toward using the nullspace
				Eigen::VectorXd q = robot->getConfig(*workspace->arm_ids);
				Krang::Vector7d nullspace_qdot_ref = (nullspace_q_ref - q).cwiseProduct(nullspace_q_mask);

				// Jacobian: compute the desired jointspace velocity from the inputs and sensors
				Eigen::VectorXd qdot_jacobian;
				workspace->refJSVelocity(xdot, nullspace_qdot_ref, qdot_jacobian);
				if(dbg) DISPLAY_VECTOR(qdot_jacobian);

				// Avoid joint limits
				Eigen::VectorXd qdot_avoid(7);
				Krang::computeQdotAvoidLimits(robot, *workspace->arm_ids, q, qdot_avoid);

				// Add qdots together to get the overall movement
				Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;
				qdot_apply = qdot_apply.normalized() * 0.10;
				if(dbg) DISPLAY_VECTOR(qdot_apply);

				// And apply that to the arm
				if(!start) qdot_apply = Vector7d::Zero();
				somatic_motor_setvel(&daemon_cx, krang->arms[Krang::LEFT], qdot_apply.data(), 7);

			} break;

			case STEP_GRASP: {
				cout <<"hello world" << endl;
				system("echo 0.0 | sudo somatic_motor_cmd lgripper pos");
				step = STEP_PUSH;
				cout <<"hello world" << endl;
			} break;

			case STEP_PUSH: {

				Vector7d goal;
				goal = eig7(krang->arms[Krang::LEFT]->pos);
				goal(0) -= 2.5;
				somatic_motor_setpos(&daemon_cx, krang->arms[Krang::LEFT], goal.data(), 7);
/*
				// Send a current value
				double current = 0.0;
				somatic_vector_set_data(waistDaemonCmd->data, &current, 1);
				int r = SOMATIC_PACK_SEND(krang->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
				if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
					ach_result_to_string(static_cast<ach_status_t>(r)));

				// Check for stop condition
				if(krang->waist->pos[0] > 2.85) {
					somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
					int r = SOMATIC_PACK_SEND(krang->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
					if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
						ach_result_to_string(static_cast<ach_status_t>(r)));
					step = STEP_REST;
				}
*/

			} break;
	
			case STEP_REST: {
				double u [2] = {15, 15};
				if(start) 
					somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, u, 2,NULL);
				if(krang->imu < -1.82) {
					u[0] = u[1] = 0.0;
					somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, u, 2,NULL);
					return;
				}
			} break;
		}

		// Update the locomotion mode
		pthread_mutex_unlock(&mutex);
	}
}

/* ******************************************************************************************** */
/// Initialization
void init() {

	// Populate the control each state needs
	CtrlMode modes [] = {CTRL_GROUND, CTRL_GROUND, CTRL_STAND, CTRL_STAND, CTRL_BALANCE, 
		CTRL_BALANCE, CTRL_BALANCE, CTRL_BALANCE, CTRL_BALANCE, CTRL_SIT};
	for(size_t i = 0; i < 10; i++) stepCtrlModes[(Step)i] = modes[i];

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon 
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton("Krang");

	// Initialize the daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "09-push";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware
	Krang::Hardware::Mode mode = (Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL_GRIPSCH);
	krang = new Krang::Hardware(mode, &daemon_cx, robot);
	Vector7d lq0 = eig7(krang->arms[LEFT]->pos);

	// Set up the workspace stuff
	workspace = new Krang::WorkspaceControl(robot, Krang::LEFT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
																				  0.0, 0.0, 0.0, 0.0);	

	// Set up nullspace stuff
	nullspace_q_ref = (Krang::Vector7d()   << 0, -1.0, 0, -0.5, 0, -0.8, 0).finished();
	nullspace_q_mask = (Krang::Vector7d()  << 0,    0, 0,    1, 0,    0, 0).finished();

	// Read control gains
	readGains();

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);

	// Initialize the waist message
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__REAL_CURRENT_MODE);

	// Open the channel to read load data
	somatic_d_channel_open(&daemon_cx, &chan, "lever", NULL); 

	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	mylog.open("mylog");
	init();
	run();
	destroy();
	mylog.close();
}
