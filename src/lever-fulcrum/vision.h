/**
 * @file vision.h
 * @author Can Erdogan
 * @date Feb 18, 2014
 * @brief Contains the vision related helper functions.
 */

#include <Eigen/Dense>
#include <vector>
#include "controllers.h"

/// Determines the turn angles and the distance the robot should go from its current state
Eigen::Vector3d loadToPose (const Eigen::VectorXd& load);

/// Determines the turn angles and the distance the robot should go from its current state
Eigen::Vector3d cinderToPose (const Eigen::VectorXd& cinder);

/// Finds the consensus in the data and interprets it to get the motion parameters. Returns the
/// consensus data
Eigen::VectorXd analyzeKinectData (const std::vector <Eigen::VectorXd>& data, bool extra = true);

/// Reads the vector data coming from the vision computer
bool getVecData (ach_channel_t& chan, Eigen::VectorXd& data);

/// Reads the cinder block data coming from the vision computer and returns it in the robot frame
/// (e.g. the initial robot frame at time 0).
bool getCinder (Eigen::VectorXd& data);
bool getLoad (Eigen::VectorXd& data);

/// Read data from the localization executable
Eigen::VectorXd readCinderData ();

