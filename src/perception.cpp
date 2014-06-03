/**
 * @file perception.cpp
 * @author Can Erdogan
 * @date Jun 02, 2014
 * @brief Talks with the vision computer and handles frames.
 */

#include "perception.h"

using namespace std;

Eigen::Vector3d cinder1loc (0.0, 0.0, 0.0);
Eigen::Vector3d cinder2loc (0.0, 0.0, 0.0);
Eigen::Vector3d plate1loc (0.0, 0.0, 0.0);
Eigen::Vector3d plate2loc (0.0, 0.0, 0.0);

/* ******************************************************************************************** */
/// Finds the consensus in the data and interprets it to get the motion parameters. Returns the
/// consensus data
Eigen::VectorXd analyzeData (const std::vector <Eigen::VectorXd>& data) {

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

	// Set the output
	Eigen::VectorXd output (6);
	output.block<3,1>(0,0) = hole;
	output.block<3,1>(3,0) = normal;
	return output;
}
/* ********************************************************************************************* */
bool perception (Mode mode) {
	cout << "hi perc" << endl;
	usleep(1e5);
	return false;
}
/* ********************************************************************************************* */
