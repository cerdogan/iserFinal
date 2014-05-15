/**
 * @file 03-approachCinder-shorten.cpp
 * @author Can Erdogan
 * @date Feb 19, 2014
 * @brief Shortens the path to reach to the cinder block.
 */

#include <kore.hpp>
#include <kore/ik.hpp>
#include <kore/safety.hpp>
#include <kore/util.hpp>
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>

#include <iostream>
#include <fstream>
#include <iomanip>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <somatic/msg.h>
#include <list>

#define SQ(x) ((x) * (x))

using namespace std;
using namespace Krang;


/* ********************************************************************************************* */
// State variables

simulation::World* world;
dynamics::SkeletonDynamics* krang;
Hardware* hw;     

bool dbg = false;
bool resetInitPos = false;
bool sending_commands = false;
bool halt = false;

/* ********************************************************************************************* */
Eigen::VectorXd robotConfig (32);
Eigen::VectorXd cinderConfig (6);
list <Eigen::VectorXd> path;

/* ******************************************************************************************** */
void readFile () {

	fstream file ("path", fstream::in);
	char line [1024];
	file.getline(line, 1024);
	std::stringstream stream(line, std::stringstream::in);
	size_t i = 0;
	double newDouble;
	while (stream >> newDouble) robotConfig(i++) = newDouble;
	i = 0;
	file.getline(line, 1024);
	std::stringstream stream2(line, std::stringstream::in);
	while (stream2 >> newDouble) cinderConfig(i++) = newDouble;
	while(file.getline(line, 1024)) {
		i = 0;
		Eigen::VectorXd config (7);
		std::stringstream stream3(line, std::stringstream::in);
		while (stream3 >> newDouble) config(i++) = newDouble;
		path.push_back(config);
	}
}

/* ******************************************************************************************** */
int main () {

	// Load scene
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/04-World-Grasp.urdf");
	assert((world != NULL) && "Could not find the world");
	krang = world->getSkeleton("Krang");
	Krang::setupKrangCollisionModel(world, krang);
	cout << "Loaded the Krang model." << endl;

	// Read the file
	readFile();

	// Set the configurations
	krang->setPose(robotConfig);
	world->getSkeleton("Cinder")->setPose(cinderConfig);

	// Shorten the path
	planning::PathShortener shortener (world, krang, left_arm_ids);
	shortener.shortenPath(path);
	cout << "Shortened path: " << path.size() << endl;

	// Print the path
	size_t numPoints = path.size();
	fstream out ("short", fstream::out);
	for(size_t i = 0; i < numPoints; i++) {
		out << (*path.begin()).transpose() << endl;
		path.pop_front();
	}
	out.close();

}
