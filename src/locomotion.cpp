/**
 * @file controllers.cpp
 * @author Can Erdogan
 * @date Jun 02, 2014
 * @brief Controller for the base.
 */

#include "locomotion.h"

Eigen::Vector4d K;	//< the gains for x and th of the state. y is ignored.
Eigen::Vector3d locoGoal = Eigen::Vector3d::Zero();
Vector6d refState = Vector6d::Zero();
Eigen::Vector4d wheelsState;		 //< wheel pos and vels in radians (lphi, lphi., rphi, rphi.)
Eigen::Vector4d lastWheelsState; //< last wheel state used to update the state 
bool initialized = false;
bool dbg = false;

using namespace std;

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {
	ifstream file ("/home/cerdogan/Documents/Software/project/krang/experiments/navigation/data/gains-04.txt");
	assert(file.is_open());
	char line [1024];
	K = Eigen::Vector4d::Zero();
	file.getline(line, 1024);
	std::stringstream stream(line, std::stringstream::in);
	size_t i = 0;
	double newDouble;
	while ((i < 4) && (stream >> newDouble)) K(i++) = newDouble;
}

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void updateWheelsState(Eigen::Vector4d& wheelsState, double dt) {

	// Model constants
	static const double width = 0.69; //< krang base width in meters (measured: 0.69)
	static const double wheel_diameter = 0.536;
	static const double k1 = 0.475;//< scaling constant to make angle actually work. old: 0.55
	static const double k2 = 0.503; //< scaling constant to make displacement actually work: 5

	// Update the sensor information
	hw->updateSensors(dt);

	// Change the wheel values from radians to meters
	double tleft = hw->amc->pos[0] * wheel_diameter;  //< traversed distances for lwheel in meters
	double tright = hw->amc->pos[1] * wheel_diameter; //< traversed distances for rwheel in meters
	double vleft = hw->amc->vel[0] * wheel_diameter;  //< left wheel velocity in m/s
	double vright = hw->amc->vel[1] * wheel_diameter; //< right wheel velocity in m/s

	// Set the state
	wheelsState(0) = k2*(tleft + tright)/2.0;// + hw->imu;
	wheelsState(1) = k2*(vleft + vright)/2.0;// + hw->imuSpeed;
	wheelsState(2) = k1*(tright - tleft)/width; // (hw->amc->pos[1] - hw->amc->pos[0]) / 2.0;
	wheelsState(3) = k1*(vright - vleft)/width; // TODO: verify
}

/* ******************************************************************************************** */
/// Update the state using the wheels information with Euler approximation (we use the last
/// theta with the forward motion to reflect changes on x and y axes)
void updateState(Eigen::Vector4d& wheelsState, Eigen::Vector4d& lastWheelsState, Vector6d& state) {

	// Compute the change in forward direction and orientation
	double dx = wheelsState[0] - lastWheelsState[0];
	double dt = wheelsState[2] - lastWheelsState[2];
	double last_theta = state[4]; 

	state[0] = state[0] + dx*cos(last_theta);
	state[1] = wheelsState[1];
	state[2] = state[2] + dx*sin(last_theta);
	state[3] = 0;
	state[4] = state[4] + dt;
	state[5] = wheelsState[3];
}

/* ******************************************************************************************** */
void computeTorques (const Vector6d& state, double& ul, double& ur) {

	static double lastUx = 0.0;

	// Compute the linear pos error by projecting the reference state's position in the current
	// state frame to the current heading direction
	Eigen::Vector2d dir (cos(state(4)), sin(state(4)));
	Eigen::Vector2d refInCurr (refState(0) - state(0), refState(2) - state(2));
	double linear_pos_err = dir.dot(refInCurr);
	double linear_vel_err = state(1);
	
	// Compute the angular position error by taking the difference of the two headings
	double angular_pos_err = refState(4) - state(4);
	double angular_vel_err = state(5);

	// Compute the error and set the y-components to 0
	Eigen::Vector4d error (linear_pos_err, linear_vel_err, angular_pos_err, angular_vel_err);
	if(dbg) cout << "error: " << error.transpose() << endl;
	if(dbg) cout << "K: " << K.transpose() << endl;

	// Compute the forward and spin torques (note K is 4x1 for x and th)
	double u_x = (K(0)*error(0) + K(1)*error(1));
	double u_spin = (K(2)*error(2) + K(3)*error(3));

	// Limit the output torques
	u_spin = max(-20.0, min(20.0, u_spin));
	u_x= max(-15.0, min(15.0, u_x));
	if(dbg) printf("u_x: %lf, u_spin: %lf\n", u_x, u_spin);
	ul = u_x - u_spin;
	ur = u_x + u_spin;
	ul = max(-30.0, min(30.0, ul));
	ur = max(-30.0, min(30.0, ur));
	if(dbg) printf("ul: %lf, ur: %lf\n", ul, ur);
}

/* ********************************************************************************************* */
bool checkReach (LocoMode lMode) {
	static const double angleToler = 1.0 / 180.0 * M_PI;
	static const double distToler = 0.05;	// meters
	if((lMode == TURN1) && (fabs(refState(4) - state(4)) < angleToler)) return true;
	else if(lMode == FORW) {
		double distSQ = SQ(refState(0) - state(0)) + SQ(refState(1) - state(1));
		if(distSQ < SQ(distToler)) return true;
	}
	else if((lMode == TURN2) && (fabs(refState(4) - state(4)) < angleToler)) return true;
	else assert(false && "Unknown loco mode");
	return false;
}

/* ********************************************************************************************* */
bool locomotion (Mode mode) {

	// Initialize variables
	static struct timespec t_now, t_prev;
	static LocoMode lMode;
	if(!initialized) {

		// Set the reference state if perception is not needed for it
		if(mode == A1) locoGoal = Eigen::Vector3d(0.5, 1.0, M_PI_2);
		else assert(false && "unknown loco goal");

		// Set the time
		t_prev = aa_tm_now();

		// Set the state, refstate and limits
		updateWheelsState(wheelsState, 0.0);
		lastWheelsState = wheelsState;
		updateState(wheelsState, lastWheelsState, state);

		// Read the gains
		readGains();

		// Set the reference state
		lMode = TURN1;
		refState = state;
		refState(1) = refState(3) = refState(5) = 0.0;
 		refState(4) = atan2(locoGoal(1) - state(2), locoGoal(0) - state(0));
	}
	
	// Get the current time and compute the time difference and update the prev. time
	t_now = aa_tm_now();						
	double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
	t_prev = t_now;

	// Get the state and update odometry
	lastWheelsState = wheelsState;
	updateWheelsState(wheelsState, dt); 
	if(dbg) cout << "wheelsState: " << wheelsState.transpose() << endl;
	updateState(wheelsState, lastWheelsState, state);
	if(dbg) cout << "state: " << state.transpose() << endl;

	// Check if reached final configuration
	bool reached = checkReach (lMode);
	if(reached) {
		if(lMode == TURN1) {
			refState = state;
			refState(1) = refState(3) = refState(5) = 0.0;
			refState(0) = locoGoal(0);
			refState(1) = locoGoal(1);
		}
		else if(lMode == FORW) {
			refState = state;
			refState(1) = refState(3) = refState(5) = 0.0;
			refState(4) = locoGoal(2);
		}
		else if(lMode == TURN2) {
			initialized = false;
			return true;
		}
		lMode = (LocoMode) (lMode + 1);
	}
	
	// Compute the torques based on the state and the mode
	double ul, ur;
	computeTorques(state, ul, ur);

	// Apply the torque
	double input [2] = {ul, ur};
	if(dbg) cout << "u: {" << ul << ", " << ur << "}" << endl;
	if(!sending_commands) input[0] = input[1] = 0.0;
	somatic_motor_cmd(&daemon_cx, hw->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
	return false;
}
/* ********************************************************************************************* */
