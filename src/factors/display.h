/**
 * @file display.h
 * @author Can Erdogan
 * @date Nov 08, 2013
 * @brief Helper functions to display designs in grip.
 */

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include "simTab.h"
#include "geometry.h"
#include "kinematics.h"

using namespace std;
using namespace Eigen;
using namespace dynamics;
using namespace simulation;

SkeletonDynamics* krang;
vector <Part> load, lever, fulcrum;
#define transform(x,T) ((T * Eigen::Vector4d(x(0), x(1), x(2), 1.0)).topLeftCorner<3,1>())
#define make5(v1,v2,v3,v4,v5) ((Eigen::VectorXd(5) << v1,v2,v3,v4,v5).finished())
#define make6(v1,v2,v3,v4,v5,v6) ((Eigen::VectorXd(6) << v1,v2,v3,v4,v5,v6).finished())
#define make7(v1,v2,v3,v4,v5,v6,v7) ((Eigen::VectorXd(7) << v1,v2,v3,v4,v5,v6,v7).finished())
#define make6f3(vs) ((Eigen::VectorXd(6) << vs(0),vs(1),vs(2),0,0,0).finished())

int obj_dofs_ [] = {0, 1, 2, 3, 4, 5};
vector <int> obj_dofs (obj_dofs_, obj_dofs_ + sizeof(obj_dofs_) / sizeof(int));

/* ********************************************************************************************* */
Eigen::VectorXd fulcrum_conf;
Eigen::VectorXd lever_conf;
Eigen::VectorXd load_conf;
Eigen::VectorXd clf_conf;
Eigen::VectorXd clo_conf;
Eigen::VectorXd ci1_conf;
Eigen::VectorXd ci2_conf;
Eigen::VectorXd base_conf;
Eigen::VectorXd larm_conf;
Eigen::VectorXd rarm_conf;
Eigen::VectorXd intersect1_conf;

/* ********************************************************************************************* */
/// Given the barycentric coordinates of a point and assuming it is on the lever, returns
/// its 3D location.
#define pv(x) { cout << #x << ": " << (x).transpose() << "\n"; }
Vector3d contactPoint (Part* part, size_t faceId, const VectorXd& tL, const VectorXd& c) {
	Eigen::Matrix4d Tl; 
	createTransform(tL, Tl);
	Eigen::Vector3d v1 = part->getVertexW(faceId, 0);
	Eigen::Vector3d v2 = part->getVertexW(faceId, 1);
	Eigen::Vector3d v3 = part->getVertexW(faceId, 2);
	pv(v1);
	pv(v2);
	pv(v3);
	Eigen::Vector3d v3local = c(0) * v1 + c(1) * v2 + (1 - c(0) - c(1)) * v3;
	Eigen::Vector3d vc = transform(v3local, Tl);
	return vc;
}

/* ********************************************************************************************* */
/// Set the intersection of the lever and load edges
Vector3d intersection(const Eigen::VectorXd& tl, const std::vector <Part>& lever) {

	// Get the vertex locations of the lever edge
	Eigen::Matrix4d Tl; 
	createTransform(tl, Tl);
	Eigen::Vector3d lv1 = transform(lever[0].getVertexW(3, 1), Tl);
	Eigen::Vector3d lv2 = transform(lever[0].getVertexW(3, 2), Tl);

	// example 2
	static const Eigen::Vector3d lov1 (-0.803149, 0.841124, 0.156209); // 34
	static const Eigen::Vector3d lov2 (-0.803149, 0.977522, 0.156209); // 41

	// Check if lines are parallel. 
	double edgeLength = (lov2 - lov1).norm();
	Eigen::Vector3d lv = (lv2 - lv1).normalized(), lov = (lov2 - lov1) / edgeLength;
	assert(fabs(lv.dot(lov) - 1.0) > 1e-5);
	pv(lov1);
	pv(lov2);

	// Find their intersection
	double a1 = lov(0), b1 = lov(1), a2 = lv(0), b2 = lv(1);
	double x1 = lov1(0), y1 = lov1(1), x2 = lv1(0), y2 = lv1(1);
	double t = (a2*y1 + b2*x2 - a2*y2 - b2*x1) / (a1*b2 - a2*b1);
	printf("t: %lf, edgeLength: %lf\n", t, edgeLength);

	// Return the intersection point
	return lov1 + t * lov;
}

/* ********************************************************************************************* */
/// Interprets the read data for exe/24-fullFaces
void interpretData24 (const vector <double>& vs, const std::vector <Part>& lever) {
	cout << "vs size: " << vs.size() << endl;
	for(size_t i = 0; i < vs.size(); i++) cout << "\tvs[" << i << "]: " << vs[i] << endl;
	size_t f = 0,l = 6,clf = 12, clo = 15, ci1 = 18, bq = 22, q1 = 29;
	fulcrum_conf = make6(vs[f], vs[f+1], vs[f+2], vs[f+3], vs[f+4], vs[f+5]);
	lever_conf = make6(vs[l], vs[l+1], vs[l+2], vs[l+3], vs[l+4], vs[l+5]);
	clf_conf = make6(vs[clf], vs[clf+1], vs[clf+2], 0.0, 0.0, 0.0);
	// load_conf = make6(-1, 1, 0.0, M_PI_2, 0.0, M_PI_2);
	if(vs.size() > clo) clo_conf = make6(vs[clo], vs[clo+1], vs[clo+2], 0.0, 0.0, 0.0);
	if(vs.size() > ci1) ci1_conf = make6(vs[ci1], vs[ci1+1], vs[ci1+2], 0.0, 0.0, 0.0);
	if(vs.size() > bq) base_conf = make5(vs[bq], vs[bq+1], vs[bq+2], vs[bq+3], vs[bq+4]);
	if(vs.size() > q1) larm_conf = make7(vs[q1], vs[q1+1], vs[q1+2], vs[q1+3], vs[q1+4], vs[q1+5], vs[q1+6]);
	// rarm_conf = make7(0, M_PI_2, 0, 0, 0, 0, 0);
	rarm_conf = make7(-M_PI/6.0, M_PI/3.0, 0.0, M_PI/6.0, 0.0, M_PI/6.0, 0.0);
}

/* ********************************************************************************************* */
/// Interprets the read data for exe/21-example2
void interpretData21 (const vector <double>& vs) {
	using namespace ex1;
	size_t f = 0, l = 3, bq = 16, rq = 28, lq = 22, ci1 = 13, ci2 = 16, bqt = 24, clf = 9, cllo = 11; 
	fulcrum_conf = make6(vs[f], vs[f+1], 0.0, vs[f+2], 0.0, M_PI_2);
	lever_conf = make6(vs[l], vs[l+1], vs[l+2], vs[l+3], vs[l+4], vs[l+5]);
	load_conf = make6(-1, 1, 0.3321, M_PI_2, 0.0, M_PI_2);
	cout << "clf: (" << vs[clf] << ", " << vs[clf+1] << ")" << endl;
	cout << "lever_fulc_face: " << lever_fulcrum_face << endl;
	clf_conf = make6f3(contactPoint(&lever[0], lever_fulcrum_face, lever_conf, 
		Vector2d(vs[clf],vs[clf+1])));
	clo_conf = make6f3(contactPoint(&lever[0], lever_load_face, lever_conf, 
		Vector2d(vs[cllo],vs[cllo+1])));
	base_conf = make5(vs[bq], vs[bq+1], vs[bq+2], vs[bq+3], vs[bq+4]);
	pv(base_conf);	
	larm_conf = make7(vs[lq], vs[lq+1], vs[lq+2], vs[lq+3], vs[lq+4], vs[lq+5], vs[lq+6]);
	pv(larm_conf);	
return;
	rarm_conf = make7(vs[rq], vs[rq+1], vs[rq+2], vs[rq+3], vs[rq+4], vs[rq+5], vs[rq+6]);
	ci1_conf = make6f3(contactPoint(&lever[0], lever_input_face, lever_conf, Vector2d(vs[ci1],vs[ci1+1])));
	ci2_conf = make6f3(contactPoint(&lever[0], lever_input_face, lever_conf, Vector2d(vs[ci2],vs[ci2+1])));
	cout << "Joint torques: (" << vs[bqt] << ", " << vs[bqt+1] << ")" << endl;
}

/* ********************************************************************************************* */
/// Interprets the read data for exe/19-dynamics
void interpretData19 (const vector <double>& vs) {
	using namespace ex1;
	size_t f = 0, l = 3, bq = 19, rq = 28, lq = 35, ci1 = 13, ci2 = 16, bqt = 24; 
	fulcrum_conf = make6(vs[f], vs[f+1], 0.0, vs[f+2], 0.0, M_PI_2);
	lever_conf = make6(vs[l], vs[l+1], vs[l+2], vs[l+3], vs[l+4], vs[l+5]);
	load_conf = make6(-1, 1, 0.3321, M_PI_2, 0.0, M_PI_2);
	base_conf = make5(vs[bq], vs[bq+1], vs[bq+2], vs[bq+3], vs[bq+4]);
	rarm_conf = make7(vs[rq], vs[rq+1], vs[rq+2], vs[rq+3], vs[rq+4], vs[rq+5], vs[rq+6]);
	larm_conf = make7(vs[lq], vs[lq+1], vs[lq+2], vs[lq+3], vs[lq+4], vs[lq+5], vs[lq+6]);
	ci1_conf = make6f3(contactPoint(&lever[0], lever_input_face, lever_conf, Vector2d(vs[ci1],vs[ci1+1])));
	ci2_conf = make6f3(contactPoint(&lever[0], lever_input_face, lever_conf, Vector2d(vs[ci2],vs[ci2+1])));
	cout << "Joint torques: (" << vs[bqt] << ", " << vs[bqt+1] << ")" << endl;
}

/* ********************************************************************************************* */
/// Interprets the read data for exe/18-full
void interpretData18 (const vector <double>& vs) {
	using namespace ex1;
	size_t f = 0, l = 3, bq = 19, rq = 27, lq = 34, ci1 = 13, ci2 = 16;
	fulcrum_conf = make6(vs[f], vs[f+1], 0.0, vs[f+2], 0.0, M_PI);
	lever_conf = make6(vs[l], vs[l+1], vs[l+2], vs[l+3], vs[l+4], vs[l+5]);
	load_conf = make6(-1, 1, 0.3321, M_PI_2, 0.0, M_PI_2);
	base_conf = make5(vs[bq], vs[bq+1], vs[bq+2], vs[bq+3], vs[bq+4]);
	rarm_conf = make7(vs[rq], vs[rq+1], vs[rq+2], vs[rq+3], vs[rq+4], vs[rq+5], vs[rq+6]);
	larm_conf = make7(vs[lq], vs[lq+1], vs[lq+2], vs[lq+3], vs[lq+4], vs[lq+5], vs[lq+6]);
	ci1_conf = make6f3(contactPoint(&lever[0], lever_input_face, lever_conf, Vector2d(vs[ci1],vs[ci1+1])));
	ci2_conf = make6f3(contactPoint(&lever[0], lever_input_face, lever_conf, Vector2d(vs[ci2],vs[ci2+1])));
}

/* ********************************************************************************************* */
/// Sets the pose of an object
inline void setPose (string name, VectorXd val) {
	cout << "Setting pose for '" << name.c_str() << "': " << val.transpose() << endl;
	mWorld->getSkeleton(name.c_str())->setConfig(obj_dofs, val);
}

/* ********************************************************************************************* */
void readFile (const char* filepath, vector <double>& vals) {

	// Open the file
	fstream file (filepath);
	assert(file.is_open() && "Could not open the file!");

	// Read each line
	double temp;
	std::string line;
	bool startReading = false;
	while(getline(file, line)) {
		size_t i = 0;
		stringstream stream (line, stringstream::in);
		if(line.find("Found result") != std::string::npos) startReading = true;
		if(line.find("====") != std::string::npos) continue;
		if(startReading)
			while(stream >> temp) vals.push_back(temp);
	}
}


