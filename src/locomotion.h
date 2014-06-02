/**
 * @file controllers.h
 * @author Can Erdogan
 * @date Jun 02, 2014
 * @brief Controller for the base.
 */

#include "helpers.h"

enum LocoMode {
	TURN1 = 0,
	FORW = 1,
	TURN2 = 2
};

extern Vector6d refState;				//< reference state (x,x.,y,y.,th,th.)

/// Given a location (x,y,th) in the initial robot frame from start up
bool locomotion (Mode mode); 

/// Returns the name of the loco mode
inline const char* locoModeStr (LocoMode lMode) {
	if(lMode == TURN1) return "TURN1";
	if(lMode == FORW) return "FORW";
	if(lMode == TURN2) return "TURN2";
	else return "UKNOWN";
}
