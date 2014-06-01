/**
 * @file controllers.cpp
 * @author Can Erdogan
 * @date Feb 18, 2014
 * @brief Contains the controller information for the balancing acts.
 */

#include "controllers.h"

using namespace Krang;
using namespace std;

Eigen::VectorXd K_bal (6);
Eigen::VectorXd K_turn (9);
Eigen::VectorXd K_forw (6);
std::map <STEP, CTRL_MODE> stepCtrlModes;
dynamics::SkeletonDynamics* robot;			///< the robot representation in dart
bool changePermitted = false;
bool firstDetect = true;
bool start = false, done = false, nextStep = false;
bool startOver = false;
double extraUForw = 0;
double extraUSpin = 0;
bool withCinder;

/* ******************************************************************************************** */
bool controller (STEP step, Vector6d state, double dt, const Eigen::Vector3d& motionParams) { 

	if(dbg) DISPLAY_VECTOR(motionParams);

	// Get the integral parameters for turning
	size_t integralWindow = (int) K_turn(6);
	double Ksint = K_turn(7), intErrorLimit = K_turn(8);
	static vector <double> spinErrors;
	static int balancedCounter = 0;
	static int turnCounter = 0;
	static int forwardCounter = 0;
	static int spinIdx = 0;
	CTRL_MODE ctrlMode = stepCtrlModes[step];
	if((spinIdx == 0) && (ctrlMode == CTRL_TURN)) {
		for(size_t i = 0; i < integralWindow; i++)
			spinErrors.push_back(0.0);
	}

	// Define the variables for velocity profiles
	static const double acceleration = 0.008;		
	static const double deceleration = 0.008;	
	static const double maxVel = 0.12;	
	static double accTime = -1.0;
	static double decTime = -1.0;
	static double cruiseTime = -1.0;

	// Determine if the step has changed. If so, set the initial state, state0. Or increment time.
	static double ctrlTime = 0.0;
	static Vector6d state0 = state;
	static STEP lastStep = step;
	if(step != lastStep) {
//		if((lastStep == STEP_TURN_DETECT_1) || (lastStep == STEP_FORWARD_DETECT) || 
//			 	(lastStep == STEP_TURN_DETECT_2)) { 
			cout << "Resetting state0: " << endl;
			state0 = state;
			DISPLAY_VECTOR(state0);
		//}
		ctrlTime = 0.0;
		if(ctrlMode != CTRL_TURN) {
			spinErrors.clear();
			spinIdx = 0;
		}
		accTime = -1.0;
	}
	else ctrlTime += dt;
	lastStep = step;
	if((ctrlMode == CTRL_FORWARD) && dbg) {
		Eigen::VectorXd times (4);
		times << ctrlTime, accTime, decTime, cruiseTime;
		DISPLAY_VECTOR(times);
	}
	if(dbg) DISPLAY_VECTOR(state0);

	// Compute the acceleration, deceleration and cruise times if moving forward
	if((step == STEP_FORWARD) && (accTime < 0.0))
		createProfile(motionParams(1) + 1.5, maxVel, acceleration, deceleration, accTime, decTime, cruiseTime);

	// Set the reference state using the initial state and the motion parameters
	Vector6d refState;
	if(ctrlMode == CTRL_BALANCE) refState << 0.0, 0.0, state0(2), 0.0, state0(4), 0.0;
	else if(ctrlMode == CTRL_TURN) {
		double change = ((step == STEP_TURN_1) ? motionParams(0) : motionParams(2));
		double refAngle;
		if(change > 0.0) refAngle = min(state0(4) + change, state(4) + 0.5);
		else if(change < 0.0) refAngle = max(state0(4) + change, state(4) - 0.5);
		else refAngle = state0(4);
		refState << 0.0, 0.0, state0(2) + (firstDetect ? 0.0 : 0.0), 0.0, refAngle, 0.0;
	}
	else if(ctrlMode == CTRL_FORWARD) {
		refState << 0.0, 0.0, 0.0, 0.0, state0(4), 0.0; 
		getForwardReference(state0, ctrlTime, acceleration, deceleration, accTime, decTime, cruiseTime, 
			refState(2), refState(3));
	}

	// Compute the error 
	Vector6d error = state - refState;
	if(dbg) {
		cout << "\033[1;36m";
		DISPLAY_VECTOR(error);
		cout << "\033[0m\n";
	}

	// Set the gains
	Eigen::VectorXd K;
	if(ctrlMode == CTRL_BALANCE) K = K_bal;
	else if(ctrlMode == CTRL_TURN) K = K_turn;
	else if(ctrlMode == CTRL_FORWARD) K = K_forw;
	else if(ctrlMode == CTRL_SIT) K = Eigen::VectorXd::Zero(6);
	if(dbg) DISPLAY_VECTOR(K);

	// Compute the input for going forward
	double u_x = K(2) * error(2) + K(3) * error(3);
	if(!withCinder) u_x = max(-10.0, min(10.0, u_x));
	else u_x = max(-15.0, min(15.0, u_x));
	u_x += extraUForw;

	// Update the reference balancing angle due to torque from the wheels
	static const double totalMass = 142.00;
	Eigen::Vector3d com = robot->getWorldCOM();
	double com_x = -(u_x / 1.7) / (totalMass * 9.81);
	double normSq = com(0) * com(0) + com(2) * com(2);
	double com_z = sqrt(normSq - com_x * com_x);
	refState(0) = atan2(-com_x, com_z);
	if(dbg) DISPLAY_VECTOR(refState);

	// Compute the balancing input
	double u_theta = K(0) * error(0) + K(1) * error(1);

	// Compute the spin input
	double u_spin = K(4) * error(4) + K(5) * error(5);

	// Add integral
	if((ctrlMode == CTRL_TURN) && (error(4) < intErrorLimit)) {
		
		// Update the errors
		spinErrors[spinIdx % integralWindow] = error(4);
		spinIdx++;

		// Compute the total error
		double totalError = 0.0;
		for(size_t i = 0; i < integralWindow; i++)
			totalError += spinErrors[i];

		// Compute the addition
		double u_spin_int = totalError * Ksint;
		if(dbg) printf("u_spin_int: %lf\n", u_spin_int);
		u_spin += u_spin_int;
	}
	u_spin += extraUSpin;
	u_spin = max(-11.0, min(11.0, u_spin));
	if((ctrlMode == CTRL_TURN) && (fabs(state(5)) > 0.6)) u_spin = 0.0;
	if(dbg) printf("u_theta: %lf, u_x: %lf, u_spin: %lf\n", u_theta, u_x, u_spin);

	// Compute the output torques
	double ul = u_theta + u_x + u_spin;
	double ur = u_theta + u_x - u_spin;
	ul = max(-50.0, min(50.0, ul));
	ur = max(-50.0, min(50.0, ur));
	if(dbg) {
		cout << "extraUForw: " << extraUForw << endl;
		cout << "extraUSpin: " << extraUSpin << endl;
	}
	if(dbg) cout << "u: {" << ul << ", " << ur << "}" << endl;

	// Set the special case for sitting down
	if(ctrlMode == CTRL_SIT) {
		ul = ur = 15.0;
	}

	// Set the wheel torques
	double input [2] = {ul, ur};
	if(!start) input[0] = input[1] = 0.0;
	if(dbg) DISPLAY_SCALAR(start);
	somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);

	// Return the state of the controller and for sitting down, set torque to 0
	if(ctrlMode == CTRL_TURN) {
		bool reached = (fabs(error(4)) < 0.05); // && (fabs(error(2)) < 0.20);
		if(reached) turnCounter++;
		else turnCounter = 0;
		return (turnCounter > 40);
	}

	else if(ctrlMode == CTRL_FORWARD) {
		bool reached = (ctrlTime > (0.0 + accTime + decTime + cruiseTime));
//			(fabs(error(2)) < 0.40);
		if(reached) forwardCounter++;
		else forwardCounter = 0;
		return (forwardCounter > 40);
	}

	else if(ctrlMode == CTRL_BALANCE) {
		if(fabs(error(0)) < 0.05) balancedCounter++;
		else balancedCounter = 0.0;
		if(dbg) DISPLAY_SCALAR(balancedCounter);
		return (balancedCounter > 40);
	}

	else if(ctrlMode == CTRL_SIT) {
		bool reached = (krang->imu < -1.82);
		if(reached) {
			double z [2] = {0.0, 0.0};
			somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, z, 2, NULL);
		}
		return reached;
	}
}

/* ******************************************************************************************** */
void createProfile(double dist, double maxVel, double acc, double dec, double& accTime, 
		double& decTime, double& cruiseTime) {
	
	// Start with the assumption that we can have a velocity profile with maximum velocity
	accTime = maxVel / acc;
	decTime = maxVel / dec;
	double distSoFar = 0.5 * maxVel * accTime + 0.5 * maxVel * decTime;
	double distLeft = dist - distSoFar;
	cruiseTime = distLeft / maxVel;

	// If the distance left is negative, that means we would overshoot the goal with maximum
	// velocity. Then choose a velocity profile without cruise time and compute what time should be. 
	if(distLeft < 0.0) {
		accTime = decTime = dist / acc;
		cruiseTime = 0.0;
	}
	
}

/* ******************************************************************************************** */
void getForwardReference(const Eigen::VectorXd& state0, double time, double acc, double dec, 
		double accTime, double decTime, double cruiseTime, double& refPos, double& refVel) {

	// Get the current reference velocity
	if(time < accTime) refVel = time * acc;
	else if(time < (accTime + cruiseTime)) refVel = accTime * acc;
	else if(time < (accTime + cruiseTime + decTime))
		refVel = accTime * acc - (time - accTime - cruiseTime) * dec;
	else refVel = 0.0;
	
	// Get the position from the reference velocities
	if(time < accTime) refPos = (time * (time * acc)) / 2.0;
	else if(time < (accTime + cruiseTime))
		refPos = (accTime * (accTime * acc)) / 2.0 + 
							(time - accTime) * (accTime * acc);
	else if(time < (accTime + cruiseTime + decTime))
		refPos = (accTime * (accTime * acc)) / 2.0 + 
						 (cruiseTime * (accTime * acc)) + 
						 (refVel * (time - accTime - cruiseTime)) + 
						 (time - accTime - cruiseTime) * (acc * accTime - refVel) / 2.0;
	else refPos = (accTime * (acc * accTime)) / 2.0 + 
							  (cruiseTime * (accTime * acc)) + 
								(decTime * (dec * decTime)) / 2.0;
	refPos += state0(2);
}

/* ******************************************************************************************** */
const char* stepName (STEP step) {
	switch (step) {
		case STEP_STANDUP: return "STEP_STANDUP";
		case STEP_TURN_1: return "STEP_TURN_1";
		case STEP_TURN_DETECT_1: return "STEP_TURN_DETECT_1";
		case STEP_FORWARD: return "STEP_FORWARD";
		case STEP_FORWARD_DETECT: return "STEP_FORWARD_DETECT";
		case STEP_TURN_2: return "STEP_TURN_2";
		case STEP_TURN_DETECT_2: return "STEP_TURN_DETECT_2";
		case STEP_SIT: return "STEP_SIT";
		default: break;
	};
}

/* ******************************************************************************************** */
const char* ctrlModeName (CTRL_MODE ctrlMode) {
	switch (ctrlMode) {
		case CTRL_BALANCE: return "CTRL_BALANCE";
		case CTRL_TURN: return "CTRL_TURN";
		case CTRL_FORWARD: return "CTRL_FORWARD";
		case CTRL_SIT: return "CTRL_SIT";
		default: break;
	};
}

/* ********************************************************************************************* */
void *kbhit(void *) {
	char input;
	while(true){ 
		input=cin.get(); 
		if(input=='s') start = !start;
		if(input=='c') changePermitted = true;
		if(input=='r') startOver = true;
		if(input=='x') extraUForw = extraUSpin = 0.0;
		if(input=='i') extraUForw += 0.5;
		if(input=='k') extraUForw -= 0.5;
		if(input=='j') extraUSpin += 0.5;
		if(input=='l') extraUSpin -= 0.5;
		if(input=='d') done = true;
		if(input=='n') nextStep = true;
	}
}

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {

	// Get the gains
	Eigen::VectorXd* kgains [] = {&K_bal, &K_turn, &K_forw};
	fstream file;
	if(withCinder) file.open("/home/cerdogan/Documents/Software/project/krang/iser/data/gains-06.txt");
	else file.open("/home/cerdogan/Documents/Software/project/krang/iser/data/gains-02.txt");
	assert(file.is_open());
	char line [1024];
	for(size_t k_idx = 0; k_idx < 3; k_idx++) {
		file.getline(line, 1024);
		std::stringstream stream(line, std::stringstream::in);
		size_t i = 0;
		double newDouble;
		while (stream >> newDouble) (*kgains[k_idx])(i++) = newDouble;
	}
	DISPLAY_VECTOR(K_bal);
	DISPLAY_VECTOR(K_turn);
	DISPLAY_VECTOR(K_forw);
	sleep(1);
}
