/**
 * @file controllers.cpp
 * @author Can Erdogan
 * @date Jun 02, 2014
 * @brief Controller for the base.
 */

#include "locomotion.h"
#include <vector>

using namespace std;

Eigen::Vector4d K_normal;	//< the gains for x and th of the state. y is ignored.
Eigen::Vector4d K_smallCinder;	//< the gains for x and th of the state. y is ignored.
Eigen::Vector3d locoGoal = Eigen::Vector3d::Zero();
Vector6d refState = Vector6d::Zero();
Eigen::Vector4d wheelsState;		 //< wheel pos and vels in radians (lphi, lphi., rphi, rphi.)
Eigen::Vector4d lastWheelsState; //< last wheel state used to update the state 
bool initialized = false;
bool dbg = false;
size_t integralWindow = 0;
int integralIndex = 0;
double Kxint, Kthint, x_int_lim, th_int_lim;
vector <Eigen::Vector2d> integralErrors;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

Eigen::VectorXd smallGraspPose = 
	(Eigen::VectorXd (7) << 0.673,0.74,0.404,0.83,-1.727,1.825,-1.641).finished();

LocoMode lMode;
Mode mode_;

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {

	// Get the PD gains
	ifstream file ("/home/cerdogan/Documents/Software/project/krang/iser/data/gains-04.txt");
	assert(file.is_open());
	char line [1024];
	file.getline(line, 1024);
	std::stringstream streampd(line, std::stringstream::in);
	size_t i = 0;
	double newDouble;
	while ((i < 4) && (streampd >> newDouble)) K_normal(i++) = newDouble;

	// Read the PD gains for holding the small cinder
	file.getline(line, 1024);
	std::stringstream streampd_sc(line, std::stringstream::in);
	i = 0;
	while ((i < 4) && (streampd_sc >> newDouble)) K_smallCinder(i++) = newDouble;

	// Get the integral control options
	file.getline(line, 1024);
	std::stringstream stream(line, std::stringstream::in);
	stream >> integralWindow;

	file.getline(line, 1024);
	std::stringstream stream2(line, std::stringstream::in);
	stream2 >> Kxint;
	stream2 >> Kthint;

	file.getline(line, 1024);
	std::stringstream stream3(line, std::stringstream::in);
	stream3 >> x_int_lim;
	stream3 >> th_int_lim;

	file.close();
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

	// Compute the state
	state[0] = state[0] + dx*cos(last_theta);
	state[1] = wheelsState[1];
	state[2] = state[2] + dx*sin(last_theta);
	state[3] = 0;
	state[4] = state[4] + dt;
	state[5] = wheelsState[3];

	// Set the robot config
	static int base_ids_a [3] = {0, 1, 3};
	static vector <int> base_ids (base_ids_a, base_ids_a + 3);  
	Eigen::Vector3d base_conf (state(0), state(2), state(4) - M_PI_2);
	krang->setConfig(base_ids, base_conf);
}

/* ******************************************************************************************** */
void computeTorques (const Vector6d& state, double& ul, double& ur) {

	static double lastUx = 0.0;

	// Compute the linear pos error by projecting the reference state's position in the current
	// state frame to the current heading direction
	Eigen::Vector2d dir (cos(state(4)), sin(state(4)));
	Eigen::Vector2d refInCurr (refState(0) - state(0), refState(2) - state(2));
	double linear_pos_err = dir.dot(refInCurr);
	double linear_vel_err = -state(1);
	
	// Compute the angular position error by taking the difference of the two headings
	double angular_pos_err = refState(4) - state(4);
	double angular_vel_err = state(5);

	// Compute the error and set the y-components to 0
	Eigen::Vector4d error (linear_pos_err, linear_vel_err, angular_pos_err, angular_vel_err);
	if(dbg) cout << "error: " << error.transpose() << endl;
	if(dbg) printf("Kxint: %lf, Kthint: %lf, x_int_lim: %lf, th_int_lim: %lf\n",
		Kxint, Kthint, x_int_lim, th_int_lim);

	// Select the gains based on the mode
	pthread_mutex_lock(&mutex);
	Eigen::Vector4d K;
	if((mode_ == A1) || (mode_ == A3)) K = K_normal;
	else if((mode_ == A5) || (mode_ == A7)) K = K_smallCinder;
	else assert(false && "unknown mode to set K gains for");
	if(dbg) cout << "K: " << K.transpose() << endl;
	pthread_mutex_unlock(&mutex);

	// Compute the forward and spin torques (note K is 4x1 for x and th)
	double u_x = (K(0)*error(0) + K(1)*error(1));
	double u_spin = (K(2)*error(2) + K(3)*error(3));

	// Add integral
	static int int_x_counter = 0, int_th_counter = 0;
	if((integralWindow > 0) && sending_commands) {

		// Update the errors
		integralErrors[integralIndex % integralWindow] = 
			Eigen::Vector2d(linear_pos_err, angular_pos_err);
		integralIndex++;

		// Compute the total error
		Eigen::Vector2d totalError (0.0, 0.0);
		for(size_t i = 0; i < integralWindow; i++)
			totalError += integralErrors[i];
		if(dbg) cout << "total int errors: x: " << totalError(0) << ", spin: "<< totalError(1) << endl;

		// Compute the addition if necessary
		double u_x_int = 0.0, u_th_int = 0.0;
		if(fabs(linear_pos_err) < x_int_lim) int_x_counter++;
		else int_x_counter = 0;
		if(lMode == TURN1 || lMode == TURN2) int_x_counter = 0;
		if(int_x_counter > 100) {
			u_x_int = totalError(0) * Kxint;
			u_x += u_x_int;
		}
		if(fabs(angular_pos_err) < th_int_lim) int_th_counter++;
		else int_th_counter = 0;
		if(lMode == FORW) int_th_counter = 0;
		if(int_th_counter > 100) {
			u_th_int = totalError(1) * Kthint;
			u_spin  += u_th_int;
		}
		if(dbg) printf("u_x_int: %lf, u_spin_int: %lf\n", u_x_int, u_th_int);
	}

	// Limit the output torques
	u_spin = max(-23.0, min(23.0, u_spin));
	u_x= max(-18.0, min(18.0, u_x));
	if(dbg) printf("u_x: %lf, u_spin: %lf\n", u_x, u_spin);
	ul = u_x - u_spin;
	ur = u_x + u_spin;
	ul = max(-23.0, min(23.0, ul));
	ur = max(-23.0, min(23.0, ur));
	if(dbg) printf("ul: %lf, ur: %lf\n", ul, ur);
}

/* ********************************************************************************************* */
bool checkReach (LocoMode lMode) {
	static const double angleToler = 3.0 / 180.0 * M_PI;
	static const double distToler = 0.05;	// meters
	static int c_ = 0;
	if(dbg) cout << "reach check: " << c_ << endl;
	if(lMode == TURN1) {
		if(fabs(refState(4) - state(4)) < angleToler) c_++;
		else c_ = 0;
	}
	else if(lMode == FORW) {
		Eigen::Vector2d dir (cos(state(4)), sin(state(4)));
		Eigen::Vector2d refInCurr (refState(0) - state(0), refState(2) - state(2));
		double linear_pos_err = dir.dot(refInCurr);
		if(fabs(linear_pos_err) < distToler) c_++;
		else c_ = 0;
	}
	else if(lMode == TURN2) {
		if(fabs(refState(4) - state(4)) < angleToler) c_++;
		else c_ = 0;
	}
	else assert(false && "Unknown loco mode");
	if(c_ > 150) {
		c_ = 0;
		return true;
	}
	return false;
}

/* ********************************************************************************************* */
bool locomotion (Mode mode) {

	mode_ = mode;

	// Initialize the dofs
	static int dofs_xy_a [] = {0, 1};
	static vector <int> dofs_xy (dofs_xy_a, dofs_xy_a + 2);  
	static int base_ids_a [3] = {0, 1, 3};
	static vector <int> base_ids (base_ids_a, base_ids_a + 3);  

	// Initialize variables
	static int c_ = 0;
	static struct timespec t_now, t_prev;
	if(!initialized) {

		// For visualization
		world->getSkeleton("KrangNext")->setPose(krang->getPose());

		// Set the reference state if perception is not needed for it
		if(mode == A1) locoGoal = Eigen::Vector3d(0.4, 0.8, M_PI_2);
		else if(mode == A3) {

			// Get the pose of the cinder block
			Eigen::VectorXd cinderPose = world->getSkeleton("Cinder2")->getPose();
			Eigen::Vector2d cinderLoc (cinderPose(0), cinderPose(1));

			// Estimate where the robot should be 
			Eigen::Vector2d dir (cos(cinderPose(3)), sin(cinderPose(3)));
			Eigen::Vector2d perp (-dir(1), dir(0));
			Eigen::Vector2d temp = cinderLoc - 0.50 * perp - 0.50 * dir;
			locoGoal = Eigen::Vector3d(temp(0), temp(1), cinderPose(3) + M_PI_2);
 
			// Set the arm pose for visualization
			world->getSkeleton("KrangNext")->setConfig(Krang::right_arm_ids,smallGraspPose);
		}
		else if(mode == A5) {
			locoGoal = krang->getConfig(base_ids);
			locoGoal(2) -= M_PI_2;
		}
		else if(mode == A7) {

			// Get the pose of the cinder block
			Eigen::VectorXd cinderPose = world->getSkeleton("Cinder2G")->getPose();
			Eigen::Vector2d cinderLoc (cinderPose(0), cinderPose(1));
			double th = cinderPose(3) - M_PI;

			// Estimate where the robot should be 
			Eigen::Vector2d dir (cos(th), sin(th));
			Eigen::Vector2d perp (-dir(1), dir(0));
			Eigen::Vector2d temp = cinderLoc - 0.30 * perp - 0.50 * dir;
			locoGoal = Eigen::Vector3d(temp(0), temp(1), th + M_PI_2);
 
			// Set the arm pose for visualization
			world->getSkeleton("KrangNext")->setConfig(Krang::right_arm_ids,smallGraspPose);

		}
		else assert(false && "unknown loco goal");

		// Update the visualization for the goal
		Eigen::Vector3d temp = locoGoal;
		temp(2) -= M_PI_2;
		world->getSkeleton("KrangNext")->setConfig(base_ids, temp);

		// Set the time
		t_prev = aa_tm_now();

		// Set the state, refstate and limits
		updateWheelsState(wheelsState, 0.0);
		lastWheelsState = wheelsState;
		updateState(wheelsState, lastWheelsState, state);

		// Read the gains and initialize the integral controller
		readGains();
		integralErrors.clear();
		for(size_t i = 0; i < integralWindow; i++)
			integralErrors.push_back(Eigen::Vector2d(0.0, 0.0));
		integralIndex = 0;

		// Set the reference state
		lMode = TURN1;
		refState = state;
		refState(1) = refState(3) = refState(5) = 0.0;
 		refState(4) = atan2(locoGoal(1) - state(2), locoGoal(0) - state(0));
		c_ = 0;
		initialized = true;
	}
	
	dbg = ((c_++ % 20) == 0);
	if(dbg) cout << "\n===========================================" << endl;

	// Get the current time and compute the time difference and update the prev. time
	t_now = aa_tm_now();						
	double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
	t_prev = t_now;

	// Get the state and update odometry
	lastWheelsState = wheelsState;
	updateWheelsState(wheelsState, dt); 
	if(dbg) cout << "wheelsState: " << wheelsState.transpose() << endl;
	if(dbg) cout << "lastWheelsState: " << lastWheelsState.transpose() << endl;
	updateState(wheelsState, lastWheelsState, state);
	if(dbg) cout << "state: " << state.transpose() << endl;

	// Check if reached final configuration
	bool reached = checkReach (lMode);
	if(reached) {
		if(lMode == TURN1) {
			refState = state;
			refState(1) = refState(3) = refState(5) = 0.0;
			refState(0) = locoGoal(0);
			refState(2) = locoGoal(1);
		}
		else if(lMode == FORW) {
			refState = state;
			refState(1) = refState(3) = refState(5) = 0.0;
			refState(4) = locoGoal(2);
		}
		else if(lMode == TURN2) {
			initialized = false;
			double input [2] = {0.0, 0.0};
			somatic_motor_cmd(&daemon_cx, hw->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
			cout << "state: " << state.transpose() << endl;
			cout << "refState: " << refState.transpose() << endl;
			cout << "reached the goal" << endl;
			return true;
		}
		lMode = (LocoMode) (lMode + 1);
	}
	if(dbg) cout << "refState: " << refState.transpose() << endl;
	if(dbg) cout << "loco mode: " << locoModeStr(lMode) << endl;
	
	// Compute the torques based on the state and the mode
	double ul, ur;
	computeTorques(state, ul, ur);

	// Apply the torque
	double input [2] = {ul, ur};
	if(dbg) cout << "u: {" << ul << ", " << ur << "}, start: " << sending_commands << endl;
	if(!sending_commands) input[0] = input[1] = 0.0;
	somatic_motor_cmd(&daemon_cx, hw->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
	return false;
}
/* ********************************************************************************************* */
