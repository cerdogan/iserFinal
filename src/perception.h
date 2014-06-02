/**
 * @file perception.h
 * @author Can Erdogan
 * @date Jun 02, 2014
 * @brief Talks with the vision computer and handles frames.
 */

#include "helpers.h"

extern Eigen::Vector3d cinder1loc;
extern Eigen::Vector3d cinder2loc;
extern Eigen::Vector3d plate1loc;
extern Eigen::Vector3d plate2loc;

/// Looks for a certian object and sets its position
bool perception (Mode mode); 

