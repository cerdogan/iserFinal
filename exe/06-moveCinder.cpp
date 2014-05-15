/**
 * @file 06-navigateCinder.cpp
 * @author Can Erdogan
 * @date Feb 13, 2014
 * @brief 
 * Does the following:
 *  1) Stands up.
 *  2) Given the cinder block configuration wrt robot, turns towards the desired Krang position next to it.
 *  3) After initial turn is accomplished, gets data from the Kinect, updates the cinder block position.
 *  4) Turns towards to Krang goal position again.
 *  5) Move forward until it reaches that position (maybe a bit more back.)
 *  6) Get data from Kinect. 
 *  7) If in satisfactory range, jump to step 9.
 *  8) If can reach the desired position by going forward, jump to 4).
 *  9) Else, jump to 2).
 * 10) Make a final turn to position itself parallel (+th=0.1) to the block. (Ignore any error and pray).
 * 11) Sit down.
 */

#include "controllers.h"
#include "vision.h"
using namespace std;

/* ******************************************************************************************** */
somatic_d_t daemon_cx;				///< The context of the current daemon
Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
simulation::World* world;			///< the world representation in dart
ach_channel_t cinder_chan;

bool dbg = false;
bool startWithTurn = true;

/* ******************************************************************************************** */
void init() {

	// Populate the control each state needs
	CTRL_MODE modes [] = {CTRL_BALANCE, CTRL_TURN, CTRL_BALANCE, CTRL_FORWARD, CTRL_BALANCE, 
		CTRL_TURN, CTRL_BALANCE, CTRL_SIT};
	for(size_t i = 0; i < 8; i++) stepCtrlModes[(STEP)i] = modes[i];

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "06-moveCinder";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL_GRIPSCH, &daemon_cx, robot); 

	// Check that the waist is at the expected angle
	assert(fabs(krang->waist->pos[0] - 2.617) < 0.05 && "The gains and offsets are set for 150 deg");

	// Open the ach channel
	somatic_d_channel_open(&daemon_cx, &cinder_chan, "load", NULL); 

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void getState(Vector6d& state, double dt) {

	// Read motor encoders, imu and ft and update dart skeleton
  krang->updateSensors(dt);

	// Calculate the COM	
	Eigen::Vector3d robotCom = robot->getWorldCOM();
	robotCom(2) -= 0.264;
	robotCom(0) += 0.01;
	if(dbg) cout << std::setprecision(8) << "robotCom: " << robotCom.transpose() << endl;

	// Get the com of the object
	Eigen::Matrix4d T = robot->getNode("lGripper")->getWorldTransform();
	Eigen::Vector3d objectCOM = T.topRightCorner<3,1>() + T.block<3,1>(0,2) * 0.20;

	// Combine the two coms
	Eigen::Vector3d com = (robotCom* 145.0 + objectCOM * 13.2) / (145 + 13.2);
	if(dbg) cout << "com: " << com.transpose() << endl;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	state(0) = atan2(com(0), com(2));
	state(1) = krang->imuSpeed;
	state(2) = (krang->amc->pos[0] + krang->amc->pos[1]) / 2.0 + krang->imu;
	state(3) = (krang->amc->vel[0] + krang->amc->vel[1]) / 2.0 + krang->imuSpeed;
	state(4) = (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
	state(5) = (krang->amc->vel[1] - krang->amc->vel[0]) / 2.0;
}

/* ******************************************************************************************** */
bool stepSatisfied (STEP step, const Eigen::Vector3d& motionParams, 
		const Eigen::VectorXd& state) {
	return true;
}

/* ******************************************************************************************** */
void run () {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Initialize the motion with a 180 turn. 
	static const double wheelRadius = 0.264;	// r, cm
	static const double rotationRadius = 0.350837; // R, cm
	double th = (startWithTurn ? (M_PI + 0.0) : 0.0) * rotationRadius / wheelRadius;
	Eigen::Vector3d motionParams (th, 0.0, 0.0);
	DISPLAY_VECTOR(motionParams);

	// Continue processing data until stop received
	size_t c_ = 0;
	Vector6d state;
	STEP step = STEP_STANDUP;
	struct timespec t_now, t_prev = aa_tm_now();
	vector <Eigen::VectorXd> data;
	while(!somatic_sig_received) {

		dbg = (c_++ % 30 == 0);
		if(dbg) cout << "\nStep: " << stepName(step) << endl;
		if(dbg) cout << "Controller: " << ctrlModeName(stepCtrlModes[step]) << endl;

		if(done) step = STEP_SIT;
		if(nextStep) {
			step = (STEP) ((int) step + 1);
			nextStep = false;
		}

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;
 
		// Get the state 
		getState(state, dt); 
		if(dbg) DISPLAY_VECTOR(state);

		// For certain steps such as standing up, turning and going forward, all we can do
		// is to wait for the controller to converge to the desired value
		bool reached = controller(step, state, dt, motionParams);
		if(dbg) cout << "Controller output: " << reached << endl;

		// If the entire point of this step is to achieve the controller output, go to the next one
		if(reached && ((step == STEP_STANDUP) || (step == STEP_TURN_1) || (step == STEP_FORWARD) || 
				(step == STEP_TURN_2) || (step == STEP_SIT))) {
			if(step == STEP_SIT) return;
			if(changePermitted) {
				step = (STEP) ((int) step + 1);
				cout << "\033[1;33mStarting step: " << stepName(step) << "\033[0m\n";
				changePermitted = false;
				continue;
			}
			else if(dbg) {
				 cout << "\033[1;31mReady to start step: " << stepName((STEP) (step + 1)) << "?\033[0m\n";
			}
		}

		// If this is a localization step, gather data (insist no speed), and analyze when possible 
		if(((step == STEP_TURN_DETECT_1) || (step == STEP_FORWARD_DETECT) 
			|| (step == STEP_TURN_DETECT_2)) && (fabs(state(3)) < 0.15)) {
		
			if(!firstDetect && changePermitted) {
				step = (STEP) ((int) step + 1);
				cout << "\033[1;31mStarting step: " << stepName(step) << "\033[0m\n";
				changePermitted = false;
				data.clear();
				continue;
			}
			else if(!firstDetect) {
				if(dbg) cout << "\033[1;31mReady to start step: " << stepName((STEP) (step + 1)) << "?\033[0m\n";
				continue;
			}

			// Collect the data - make sure the cinder block is detected
			Eigen::VectorXd dataPoint (6);
			if(getLoad(dataPoint)) {
				if(dataPoint(0) < 9.99) 
					data.push_back(dataPoint);
			}
			if(dbg) DISPLAY_SCALAR(data.size());

			// If reached sufficient number of data points, get the consensus and drive the goal positions
			if(data.size() > 1) {
				Eigen::VectorXd visionData = analyzeKinectData(data);
				if(dbg) DISPLAY_VECTOR(visionData);
				if(firstDetect) {
					motionParams = loadToPose(visionData);// so we can see the new model before change
					motionParams = Eigen::Vector3d(0.0, 4.0, 0.0);
				}
				if(changePermitted) {
					step = (STEP) ((int) step + 1);
					if(firstDetect) step = STEP_TURN_1;
					cout << "\033[1;31mStarting step: " << stepName(step) << "\033[0m\n";
					changePermitted = false;
					data.clear();
					firstDetect = false;
					continue;
				}
				else if(dbg) {
					 cout << "\033[1;31mReady to start step: " << stepName((STEP) (step + 1)) << "?\033[0m\n";
				}
			}
		}
	}

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	if((argc > 1) && (strcmp(argv[1], "-n"))) startWithTurn = false;

	withCinder = true;

	// Read the gains
	readGains();

	// Load the world and the robot
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton(0);

	// Initialize the daemon and the drivers
	init();
	
	// Print the f/t values
	run();

	// Destroy the daemon and the robot
	somatic_d_destroy(&daemon_cx);
	delete krang;
	return 0;
}

