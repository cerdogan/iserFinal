/**
 * @file controllers.cpp
 * @author Can Erdogan
 * @date Jun 02, 2014
 * @brief Controller for the base.
 */

#include "locomotion.h"

Eigen::Vector3d locoGoal (0.0, 0.0, 0.0);

using namespace std;

/* ********************************************************************************************* */
bool locomotion (Mode mode) {
	cout << "hi" << endl;
	static int i = 0;
	i++;
	if(i > 5) return true;
	return false;
}
/* ********************************************************************************************* */
