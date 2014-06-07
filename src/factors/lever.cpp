/**
 * @file lever.cpp
 * @author Can Erdogan
 * @date Oct 03, 2013
 * @brief The collection of factors necessary to design a lever structure.
 */

#include "lever.h"

using namespace std;
using namespace gtsam;

#define SQ(x) ((x) * (x))

/* ********************************************************************************************* */
inline void createTransform (const LieVector& t, Eigen::Matrix4d& T) {
	double x = t(0), y = t(1), z = t(2), phi = t(3), th = t(4), psi = t(5);
	double cphi = cos(phi), sphi = sin(phi);
	double cth = cos(th), sth = sin(th);
	double cpsi = cos(psi), spsi = sin(psi);
	T << cphi * cth, (-sphi * cpsi + cphi * sth * spsi), (sphi * spsi + cphi * sth * cpsi), x,
     sphi * cth, (cphi * cpsi + sphi * sth * spsi), (-cphi * spsi + sphi * sth * cpsi), y,
     -sth, cth * spsi, cth * cpsi, z, 0, 0, 0, 1;
}

/* ********************************************************************************************* */
Matrix Factors::Torque::numerDeriv(const LieVector& to, const LieVector& tf, 
		const LieVector& tl, const LieVector& v, size_t vid) const {

	// Compute initial error
	LieVector hx = errorProxy(to, tf, tl, v);
	
	// Prepare the matrix
	static const double delta = 1e-5;
	size_t m = hx.dim(), n;
	if(vid == 0) n = to.dim();
	else if(vid == 1) n = tf.dim();
	else if(vid == 2) n = tl.dim();
	else if(vid == 3) n = v.dim();
	Matrix H = zeros(m,n);

	// Fill the matrix by retracting a delta ammount for each coordinate
	double factor = 1.0/(2.0*delta);
	Vector d = zero(n), hxplus, hxmin;
	for (size_t j=0;j<n;j++) {
		d(j) +=   delta; 
		if(vid == 0) hxplus = hx.localCoordinates(errorProxy(to.retract(d), tf, tl, v));
		else if(vid == 1) hxplus = hx.localCoordinates(errorProxy(to, tf.retract(d), tl, v));
		else if(vid == 2) hxplus = hx.localCoordinates(errorProxy(to, tf, tl.retract(d), v));
		else if(vid == 3) hxplus = hx.localCoordinates(errorProxy(to, tf, tl, v.retract(d)));
		d(j) -= 2*delta; 
		if(vid == 0) hxmin = hx.localCoordinates(errorProxy(to.retract(d), tf, tl, v));
		else if(vid == 1) hxmin = hx.localCoordinates(errorProxy(to, tf.retract(d), tl, v));
		else if(vid == 2) hxmin = hx.localCoordinates(errorProxy(to, tf, tl.retract(d), v));
		else if(vid == 3) hxmin = hx.localCoordinates(errorProxy(to, tf, tl, v.retract(d)));
		d(j) +=   delta;
		H.col(j) << (hxplus-hxmin)*factor;
	}
	return H;
}

/* ********************************************************************************************* */
#define pv(x) {if(debug) cout << #x << ": " << (x).transpose() << "\n"; }
Vector Factors::Torque::errorProxy(const LieVector& torque, const LieVector& tf, 
		const LieVector& tl, const LieVector& v) const {

	bool debug = 0;

	pv(tf.vector());
	pv(tl.vector());
	pv(v.vector());

	// ===========================================================
	// Prepare input (all in world coordinates)

	// Create the transformation matrices 
	Eigen::Matrix4d Tf, Tl; 
	createTransform(tf, Tf);
	createTransform(tl, Tl);

	// Get the edge vertices in the world coordinates
	Eigen::Vector3d v1 = (Tf * Eigen::Vector4d(v1p(0), v1p(1), v1p(2), 1.0)).topLeftCorner<3,1>();
	pv(v1);
	Eigen::Vector3d v2 = (Tf * Eigen::Vector4d(v2p(0), v2p(1), v2p(2), 1.0)).topLeftCorner<3,1>();
	pv(v2);

	// Set the vertex which has a lower 'y-axis' value to be v1.
	if(v2(1) < v1(1)) {	
		Eigen::Vector3d temp = v2;
		v2 = v1;
		v1 = temp;
	}

	// Compute the direction
	Eigen::Vector3d v21n = (v2 - v1).normalized();
	pv(v21n);

	// Get the force direction in the world coordinates
	Eigen::Vector3d vfn = Tl.topLeftCorner<3,3>() * vfnp;
	pv(vfn);

	// Get the contact point in world coordinates
	Vector temp = v.vector();
	Eigen::Vector3d v3 (temp(0), temp(1), temp(2));
	pv(v3);
	
	// ===========================================================
	// Compute the torque

	// Compute the closest point on the edge to the contact point 
	Eigen::Vector3d vc = v1 + (v3 - v1).dot(v21n) * v21n;
	pv(vc);

	// The vector from v3 to vc is going to be the lever arm - compute the distance
	Eigen::Vector3d vc3 = vc - v3;
	double dist = vc3.norm();
	pv(vc3);

	// Compute the component of the force perpendicular to the lever arm
	Eigen::Vector3d vc3n = vc3 / dist;
	pv(vc3n);
	Eigen::Vector3d vfn2 = vfn - dot(vfn, v21n) * v21n;
	Eigen::Vector3d vfn3 = vfn2 - dot(vfn2, vc3n) * vc3n;
	pv(vfn3);

	// Compute the magnitude of the force in this direction
	double magn = vfn3.norm();
	double forceMagn = force * magn;

	// The torque distance times force
	double expected_torque = dist * forceMagn;

	// Determine the direction of the torque
	Eigen::Vector3d posDir = vc3n.cross(v21n);
	double posProj = (vfn3 / magn).dot(posDir);
	if(posProj > 0.0) expected_torque *= -1.0;

	// Compute the error
	temp = torque.vector();
	double actual_torque = temp(0);
	return Vector_(1, actual_torque - expected_torque);
}

/* ********************************************************************************************* */
Vector Factors::Torque::evaluateError(const LieVector& torque, const LieVector& tf, 
	const LieVector& tl, const LieVector& v, boost::optional <Matrix&> H1, 
	boost::optional <Matrix&> H2, boost::optional <Matrix&> H3, boost::optional <Matrix&> H4) const {

	// Get the error vector
	Vector error = errorProxy(torque, tf, tl, v);

	// Set the derivatives - get them using the numerical derivatives
	if(H1) *H1 = numerDeriv(torque, tf, tl, v, 0);
	if(H2) *H2 = numerDeriv(torque, tf, tl, v, 1);
	if(H3) *H3 = numerDeriv(torque, tf, tl, v, 2);
	if(H4) *H4 = numerDeriv(torque, tf, tl, v, 3);

	return error;

}

/* ********************************************************************************************* */
double Factors::MinDistance::value (const LieVector& v1, const LieVector& v2, 
	boost::optional <Matrix&> H1, boost::optional <Matrix&> H2) const {

	// Compute the squared distance
	size_t numElems = v1.dim();
	Eigen::VectorXd v1v = v1.vector();
	Eigen::VectorXd v2v = v2.vector();
	double val = (v1v - v2v).squaredNorm();

	// Compute the derivatives
	if(H1) {
		*H1 = Matrix_(1, numElems);
		for(size_t i = 0; i < numElems; i++) (*H1)(i) = 2 * v1v(i) - 2 * v2v(i);
	}
	if(H2) {
		*H2 = Matrix_(1, numElems);
		for(size_t i = 0; i < numElems; i++) (*H2)(i) = 2 * v2v(i) - 2 * v1v(i);
	}

	// Return the value
	return val;
}

/* ********************************************************************************************* */
Vector Factors::VertexOnLine::evaluateError(const LieVector& t, const LieVector& p, 
	boost::optional <Matrix&> H1, boost::optional <Matrix&> H2) const {

	// Get the variable names for convenience
	double vx = p(0), vy = p(1), vz = p(2);
	double x = t(0), y = t(1), z = t(2), phi = t(3), th = t(4), psi = t(5);
	double v1x = v1(0), v1y = v1(1), v1z = v1(2);
	double v21x = v21(0), v21y = v21(1), v21z = v21(2);

	// Precompute the cosine and sine values
	double cphi = cos(phi), sphi = sin(phi);
	double cth = cos(th), sth = sin(th);
	double cpsi = cos(psi), spsi = sin(psi);

	// Compute the error
	double error = SQ(v1x - v21x*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + SQ(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - v21z*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cpsi*cth + z*cpsi*cth) + SQ(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - v21y*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cth*spsi + z*cth*spsi);

	// Compute the derivatives
	if(H1) {

		double dx = 2*(cphi*cth - v21x*(v21z*(sphi*spsi + cphi*cpsi*sth) - v21y*(cpsi*sphi - cphi*spsi*sth) + v21x*cphi*cth))*(v1x - v21x*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + 2*(sphi*spsi - v21z*(v21z*(sphi*spsi + cphi*cpsi*sth) - v21y*(cpsi*sphi - cphi*spsi*sth) + v21x*cphi*cth) + cphi*cpsi*sth)*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - v21z*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cpsi*cth + z*cpsi*cth) - 2*(cpsi*sphi + v21y*(v21z*(sphi*spsi + cphi*cpsi*sth) - v21y*(cpsi*sphi - cphi*spsi*sth) + v21x*cphi*cth) - cphi*spsi*sth)*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - v21y*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cth*spsi + z*cth*spsi);
 
	double dy = 2*(cth*sphi - v21x*(v21y*(cphi*cpsi + sphi*spsi*sth) - v21z*(cphi*spsi - cpsi*sphi*sth) + v21x*cth*sphi))*(v1x - v21x*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) - 2*(cphi*spsi + v21z*(v21y*(cphi*cpsi + sphi*spsi*sth) - v21z*(cphi*spsi - cpsi*sphi*sth) + v21x*cth*sphi) - cpsi*sphi*sth)*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - v21z*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cpsi*cth + z*cpsi*cth) + 2*(cphi*cpsi - v21y*(v21y*(cphi*cpsi + sphi*spsi*sth) - v21z*(cphi*spsi - cpsi*sphi*sth) + v21x*cth*sphi) + sphi*spsi*sth)*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - v21y*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cth*spsi + z*cth*spsi);
 
	double dz = - 2*(v21z*(v21z*cpsi*cth - v21x*sth + v21y*cth*spsi) - cpsi*cth)*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - v21z*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cpsi*cth + z*cpsi*cth) - 2*(v21y*(v21z*cpsi*cth - v21x*sth + v21y*cth*spsi) - cth*spsi)*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - v21y*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cth*spsi + z*cth*spsi) - 2*(sth + v21x*(v21z*cpsi*cth - v21x*sth + v21y*cth*spsi))*(v1x - v21x*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi);
 
	double dphi =  2*(v21x*(v21x*(vy*cphi*cth - y*cphi*cth - vx*cth*sphi + x*cth*sphi) - v21y*(vx*(cphi*cpsi + sphi*spsi*sth) + vy*(cpsi*sphi - cphi*spsi*sth) - x*(cphi*cpsi + sphi*spsi*sth) - y*(cpsi*sphi - cphi*spsi*sth)) + v21z*(vx*(cphi*spsi - cpsi*sphi*sth) + vy*(sphi*spsi + cphi*cpsi*sth) - x*(cphi*spsi - cpsi*sphi*sth) - y*(sphi*spsi + cphi*cpsi*sth))) - vy*cphi*cth + y*cphi*cth + vx*cth*sphi - x*cth*sphi)*(v1x - v21x*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + 2*(x*(cphi*spsi - cpsi*sphi*sth) - vy*(sphi*spsi + cphi*cpsi*sth) - vx*(cphi*spsi - cpsi*sphi*sth) + y*(sphi*spsi + cphi*cpsi*sth) + v21z*(v21x*(vy*cphi*cth - y*cphi*cth - vx*cth*sphi + x*cth*sphi) - v21y*(vx*(cphi*cpsi + sphi*spsi*sth) + vy*(cpsi*sphi - cphi*spsi*sth) - x*(cphi*cpsi + sphi*spsi*sth) - y*(cpsi*sphi - cphi*spsi*sth)) + v21z*(vx*(cphi*spsi - cpsi*sphi*sth) + vy*(sphi*spsi + cphi*cpsi*sth) - x*(cphi*spsi - cpsi*sphi*sth) - y*(sphi*spsi + cphi*cpsi*sth))))*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - v21z*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cpsi*cth + z*cpsi*cth) + 2*(vx*(cphi*cpsi + sphi*spsi*sth) + vy*(cpsi*sphi - cphi*spsi*sth) - x*(cphi*cpsi + sphi*spsi*sth) - y*(cpsi*sphi - cphi*spsi*sth) + v21y*(v21x*(vy*cphi*cth - y*cphi*cth - vx*cth*sphi + x*cth*sphi) - v21y*(vx*(cphi*cpsi + sphi*spsi*sth) + vy*(cpsi*sphi - cphi*spsi*sth) - x*(cphi*cpsi + sphi*spsi*sth) - y*(cpsi*sphi - cphi*spsi*sth)) + v21z*(vx*(cphi*spsi - cpsi*sphi*sth) + vy*(sphi*spsi + cphi*cpsi*sth) - x*(cphi*spsi - cpsi*sphi*sth) - y*(sphi*spsi + cphi*cpsi*sth))))*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - v21y*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cth*spsi + z*cth*spsi);
 
	double dth =  - 2*(v21z*(v21y*(vz*spsi*sth - z*spsi*sth - vx*cphi*cth*spsi + x*cphi*cth*spsi - vy*cth*sphi*spsi + y*cth*sphi*spsi) + v21z*(vz*cpsi*sth - z*cpsi*sth - vx*cphi*cpsi*cth + x*cphi*cpsi*cth - vy*cpsi*cth*sphi + y*cpsi*cth*sphi) + v21x*(vz*cth - z*cth + vx*cphi*sth - x*cphi*sth + vy*sphi*sth - y*sphi*sth)) - vz*cpsi*sth + z*cpsi*sth + vx*cphi*cpsi*cth - x*cphi*cpsi*cth + vy*cpsi*cth*sphi - y*cpsi*cth*sphi)*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - v21z*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cpsi*cth + z*cpsi*cth) - 2*(v21y*(v21y*(vz*spsi*sth - z*spsi*sth - vx*cphi*cth*spsi + x*cphi*cth*spsi - vy*cth*sphi*spsi + y*cth*sphi*spsi) + v21z*(vz*cpsi*sth - z*cpsi*sth - vx*cphi*cpsi*cth + x*cphi*cpsi*cth - vy*cpsi*cth*sphi + y*cpsi*cth*sphi) + v21x*(vz*cth - z*cth + vx*cphi*sth - x*cphi*sth + vy*sphi*sth - y*sphi*sth)) - vz*spsi*sth + z*spsi*sth + vx*cphi*cth*spsi - x*cphi*cth*spsi + vy*cth*sphi*spsi - y*cth*sphi*spsi)*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - v21y*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cth*spsi + z*cth*spsi) - 2*(v21x*(v21y*(vz*spsi*sth - z*spsi*sth - vx*cphi*cth*spsi + x*cphi*cth*spsi - vy*cth*sphi*spsi + y*cth*sphi*spsi) + v21z*(vz*cpsi*sth - z*cpsi*sth - vx*cphi*cpsi*cth + x*cphi*cpsi*cth - vy*cpsi*cth*sphi + y*cpsi*cth*sphi) + v21x*(vz*cth - z*cth + vx*cphi*sth - x*cphi*sth + vy*sphi*sth - y*sphi*sth)) - vz*cth + z*cth - vx*cphi*sth + x*cphi*sth - vy*sphi*sth + y*sphi*sth)*(v1x - v21x*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi);
 
	double dpsi =  2*(v21y*(v21y*(vx*(sphi*spsi + cphi*cpsi*sth) - vy*(cphi*spsi - cpsi*sphi*sth) - x*(sphi*spsi + cphi*cpsi*sth) + y*(cphi*spsi - cpsi*sphi*sth) + vz*cpsi*cth - z*cpsi*cth) + v21z*(vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi)) - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - v21y*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cth*spsi + z*cth*spsi) + 2*(v21z*(v21y*(vx*(sphi*spsi + cphi*cpsi*sth) - vy*(cphi*spsi - cpsi*sphi*sth) - x*(sphi*spsi + cphi*cpsi*sth) + y*(cphi*spsi - cpsi*sphi*sth) + vz*cpsi*cth - z*cpsi*cth) + v21z*(vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi)) - vx*(cpsi*sphi - cphi*spsi*sth) + vy*(cphi*cpsi + sphi*spsi*sth) + x*(cpsi*sphi - cphi*spsi*sth) - y*(cphi*cpsi + sphi*spsi*sth) + vz*cth*spsi - z*cth*spsi)*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - v21z*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cpsi*cth + z*cpsi*cth) + 2*v21x*(v21y*(vx*(sphi*spsi + cphi*cpsi*sth) - vy*(cphi*spsi - cpsi*sphi*sth) - x*(sphi*spsi + cphi*cpsi*sth) + y*(cphi*spsi - cpsi*sphi*sth) + vz*cpsi*cth - z*cpsi*cth) + v21z*(vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi))*(v1x - v21x*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi);

		*H1 = Matrix_(1,6,dx,dy,dz,dphi,dth,dpsi);
	} 
 
	if(H2) {
		double dvx = 2*(cpsi*sphi + v21y*(v21z*(sphi*spsi + cphi*cpsi*sth) - v21y*(cpsi*sphi - cphi*spsi*sth) + v21x*cphi*cth) - cphi*spsi*sth)*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - v21y*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cth*spsi + z*cth*spsi) - 2*(sphi*spsi - v21z*(v21z*(sphi*spsi + cphi*cpsi*sth) - v21y*(cpsi*sphi - cphi*spsi*sth) + v21x*cphi*cth) + cphi*cpsi*sth)*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - v21z*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cpsi*cth + z*cpsi*cth) - 2*(cphi*cth - v21x*(v21z*(sphi*spsi + cphi*cpsi*sth) - v21y*(cpsi*sphi - cphi*spsi*sth) + v21x*cphi*cth))*(v1x - v21x*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi);
 
	double dvy =  2*(cphi*spsi + v21z*(v21y*(cphi*cpsi + sphi*spsi*sth) - v21z*(cphi*spsi - cpsi*sphi*sth) + v21x*cth*sphi) - cpsi*sphi*sth)*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - v21z*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cpsi*cth + z*cpsi*cth) - 2*(cth*sphi - v21x*(v21y*(cphi*cpsi + sphi*spsi*sth) - v21z*(cphi*spsi - cpsi*sphi*sth) + v21x*cth*sphi))*(v1x - v21x*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) - 2*(cphi*cpsi - v21y*(v21y*(cphi*cpsi + sphi*spsi*sth) - v21z*(cphi*spsi - cpsi*sphi*sth) + v21x*cth*sphi) + sphi*spsi*sth)*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - v21y*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cth*spsi + z*cth*spsi);
 
	double dvz = 2*(v21z*(v21z*cpsi*cth - v21x*sth + v21y*cth*spsi) - cpsi*cth)*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - v21z*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cpsi*cth + z*cpsi*cth) + 2*(v21y*(v21z*cpsi*cth - v21x*sth + v21y*cth*spsi) - cth*spsi)*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - v21y*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) - vz*cth*spsi + z*cth*spsi) + 2*(sth + v21x*(v21z*cpsi*cth - v21x*sth + v21y*cth*spsi))*(v1x - v21x*(v21y*(v1y + vx*(cpsi*sphi - cphi*spsi*sth) - vy*(cphi*cpsi + sphi*spsi*sth) - x*(cpsi*sphi - cphi*spsi*sth) + y*(cphi*cpsi + sphi*spsi*sth) - vz*cth*spsi + z*cth*spsi) + v21x*(v1x + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi) + v21z*(v1z - vx*(sphi*spsi + cphi*cpsi*sth) + vy*(cphi*spsi - cpsi*sphi*sth) + x*(sphi*spsi + cphi*cpsi*sth) - y*(cphi*spsi - cpsi*sphi*sth) - vz*cpsi*cth + z*cpsi*cth)) + vz*sth - z*sth - vx*cphi*cth + x*cphi*cth - vy*cth*sphi + y*cth*sphi);
 
		*H2 = Matrix_(1,3,dvx,dvy,dvz);

	}
	return Vector_(1,error);
}

/* ********************************************************************************************* */
/// @todo Optimize by computing the sub-expressions beforehand
double Factors::VertexInTri::value(const LieVector& t, const LieVector& p, 
	boost::optional <Matrix&> H1, boost::optional <Matrix&> H2) const {

	// Get the variable names for convenience
	double vx = p(0), vy = p(1), vz = p(2);
	double x = t(0), y = t(1), z = t(2), phi = t(3), th = t(4), psi = t(5);
	double a = plane(0), b = plane(1), c = plane(2), d = plane(3);

	// Precompute the cosine and sine values
	double cphi = cos(phi), sphi = sin(phi);
	double cth = cos(th), sth = sin(th);
	double cpsi = cos(psi), spsi = sin(psi);

	// Compute the error
	double error = d + vx*(c*(sphi*spsi + cphi*cpsi*sth) - b*(cpsi*sphi 
												 	- cphi*spsi*sth) + a*cphi*cth) 
										- x*(c*(sphi*spsi + cphi*cpsi*sth) - b*(cpsi*sphi 
												 	- cphi*spsi*sth) + a*cphi*cth) 
										+ vy*(b*(cphi*cpsi + sphi*spsi*sth) - c*(cphi*spsi 
													- cpsi*sphi*sth) + a*cth*sphi) 
										- y*(b*(cphi*cpsi + sphi*spsi*sth) - c*(cphi*spsi 
													- cpsi*sphi*sth) + a*cth*sphi) 
										+ vz*(c*cpsi*cth - a*sth + b*cth*spsi) 
										- z*(c*cpsi*cth - a*sth + b*cth*spsi);

	// Compute the derivatives 
	if(H1) {
		double dx = b*(cpsi*sphi - cphi*spsi*sth) - c*(sphi*spsi 
			+ cphi*cpsi*sth) - a*cphi*cth;
		double dy = c*(cphi*spsi - cpsi*sphi*sth) - b*(cphi*cpsi 
			+ sphi*spsi*sth) - a*cth*sphi;
		double dz = a*sth - c*cpsi*cth - b*cth*spsi;
		double dphi = vy*(c*(sphi*spsi + cphi*cpsi*sth) - b*(cpsi*sphi 
			- cphi*spsi*sth) + a*cphi*cth) - y*(c*(sphi*spsi 
			+ cphi*cpsi*sth) - b*(cpsi*sphi - cphi*spsi*sth) 
			+ a*cphi*cth) - vx*(b*(cphi*cpsi + sphi*spsi*sth) 
			- c*(cphi*spsi - cpsi*sphi*sth) + a*cth*sphi) 
			+ x*(b*(cphi*cpsi + sphi*spsi*sth) - c*(cphi*spsi 
			- cpsi*sphi*sth) + a*cth*sphi);
		double dth = vx*(c*cphi*cpsi*cth - a*cphi*sth + b*cphi*cth*spsi)
			- x*(c*cphi*cpsi*cth - a*cphi*sth + b*cphi*cth*spsi) 
			+ vy*(c*cpsi*cth*sphi - a*sphi*sth + b*cth*sphi*spsi) 
			- y*(c*cpsi*cth*sphi - a*sphi*sth + b*cth*sphi*spsi) 
			- vz*(a*cth + c*cpsi*sth + b*spsi*sth) + z*(a*cth 
			+ c*cpsi*sth + b*spsi*sth);
		double dpsi = vx*(b*(sphi*spsi + cphi*cpsi*sth) + c*(cpsi*sphi 
			- cphi*spsi*sth)) - vy*(b*(cphi*spsi - cpsi*sphi*sth) 
			+ c*(cphi*cpsi + sphi*spsi*sth)) + vz*(b*cpsi*cth 
			- c*cth*spsi) - x*(b*(sphi*spsi + cphi*cpsi*sth) 
			+ c*(cpsi*sphi - cphi*spsi*sth)) + y*(b*(cphi*spsi 
			- cpsi*sphi*sth) + c*(cphi*cpsi + sphi*spsi*sth)) 
			- z*(b*cpsi*cth - c*cth*spsi);
		*H1 = Matrix_(1,6,dx,dy,dz,dphi,dth,dpsi);
	}

	if(H2) {
		double dvx = c*(sphi*spsi + cphi*cpsi*sth) - b*(cpsi*sphi 
			- cphi*spsi*sth) + a*cphi*cth;
 		double dvy = b*(cphi*cpsi + sphi*spsi*sth) - c*(cphi*spsi 
			- cpsi*sphi*sth) + a*cth*sphi;
 		double dvz = c*cpsi*cth - a*sth + b*cth*spsi;
		*H2 = Matrix_(1,3,dvx,dvy,dvz);
	}

	return error;
}


/* ********************************************************************************************* */
/// @todo Optimize by computing the sub-expressions beforehand
Vector Factors::VertexOnTriPlane::evaluateError(const LieVector& t, const LieVector& p, 
	boost::optional <Matrix&> H1, boost::optional <Matrix&> H2) const {

	// Get the variable names for convenience
	double vx = p(0), vy = p(1), vz = p(2);
	double x = t(0), y = t(1), z = t(2), phi = t(3), th = t(4), psi = t(5);
	double a = plane(0), b = plane(1), c = plane(2), d = plane(3);

	// Precompute the cosine and sine values
	double cphi = cos(phi), sphi = sin(phi);
	double cth = cos(th), sth = sin(th);
	double cpsi = cos(psi), spsi = sin(psi);

	// Compute the error
	double error = d + vx*(c*(sphi*spsi + cphi*cpsi*sth) - b*(cpsi*sphi 
												 	- cphi*spsi*sth) + a*cphi*cth) 
										- x*(c*(sphi*spsi + cphi*cpsi*sth) - b*(cpsi*sphi 
												 	- cphi*spsi*sth) + a*cphi*cth) 
										+ vy*(b*(cphi*cpsi + sphi*spsi*sth) - c*(cphi*spsi 
													- cpsi*sphi*sth) + a*cth*sphi) 
										- y*(b*(cphi*cpsi + sphi*spsi*sth) - c*(cphi*spsi 
													- cpsi*sphi*sth) + a*cth*sphi) 
										+ vz*(c*cpsi*cth - a*sth + b*cth*spsi) 
										- z*(c*cpsi*cth - a*sth + b*cth*spsi);

	// Compute the derivatives 
	if(H1) {
		double dx = b*(cpsi*sphi - cphi*spsi*sth) - c*(sphi*spsi 
			+ cphi*cpsi*sth) - a*cphi*cth;
		double dy = c*(cphi*spsi - cpsi*sphi*sth) - b*(cphi*cpsi 
			+ sphi*spsi*sth) - a*cth*sphi;
		double dz = a*sth - c*cpsi*cth - b*cth*spsi;
		double dphi = vy*(c*(sphi*spsi + cphi*cpsi*sth) - b*(cpsi*sphi 
			- cphi*spsi*sth) + a*cphi*cth) - y*(c*(sphi*spsi 
			+ cphi*cpsi*sth) - b*(cpsi*sphi - cphi*spsi*sth) 
			+ a*cphi*cth) - vx*(b*(cphi*cpsi + sphi*spsi*sth) 
			- c*(cphi*spsi - cpsi*sphi*sth) + a*cth*sphi) 
			+ x*(b*(cphi*cpsi + sphi*spsi*sth) - c*(cphi*spsi 
			- cpsi*sphi*sth) + a*cth*sphi);
		double dth = vx*(c*cphi*cpsi*cth - a*cphi*sth + b*cphi*cth*spsi)
			- x*(c*cphi*cpsi*cth - a*cphi*sth + b*cphi*cth*spsi) 
			+ vy*(c*cpsi*cth*sphi - a*sphi*sth + b*cth*sphi*spsi) 
			- y*(c*cpsi*cth*sphi - a*sphi*sth + b*cth*sphi*spsi) 
			- vz*(a*cth + c*cpsi*sth + b*spsi*sth) + z*(a*cth 
			+ c*cpsi*sth + b*spsi*sth);
		double dpsi = vx*(b*(sphi*spsi + cphi*cpsi*sth) + c*(cpsi*sphi 
			- cphi*spsi*sth)) - vy*(b*(cphi*spsi - cpsi*sphi*sth) 
			+ c*(cphi*cpsi + sphi*spsi*sth)) + vz*(b*cpsi*cth 
			- c*cth*spsi) - x*(b*(sphi*spsi + cphi*cpsi*sth) 
			+ c*(cpsi*sphi - cphi*spsi*sth)) + y*(b*(cphi*spsi 
			- cpsi*sphi*sth) + c*(cphi*cpsi + sphi*spsi*sth)) 
			- z*(b*cpsi*cth - c*cth*spsi);
		*H1 = Matrix_(1,6,dx,dy,dz,dphi,dth,dpsi);
	}

	if(H2) {
		double dvx = c*(sphi*spsi + cphi*cpsi*sth) - b*(cpsi*sphi 
			- cphi*spsi*sth) + a*cphi*cth;
 		double dvy = b*(cphi*cpsi + sphi*spsi*sth) - c*(cphi*spsi 
			- cpsi*sphi*sth) + a*cth*sphi;
 		double dvz = c*cpsi*cth - a*sth + b*cth*spsi;
		*H2 = Matrix_(1,3,dvx,dvy,dvz);
	}

	return Vector_(1,error);
}

/* ********************************************************************************************* */
Vector Factors::PriorVertex::evaluateError (const LieVector& t, boost::optional <Matrix&> H) const {

	// Get the variable names for convenience
	double vx = v(0), vy = v(1), vz = v(2);
	double x = t(0), y = t(1), z = t(2), phi = t(3), th = t(4), psi = t(5);

	// Compute the location of the vertex in the world frame and the derivatives
	double val;
	double dx, dy, dz, dphi, dth, dpsi;
	switch(axis) {

		// X-axis
		case 0: {

			// Compute the x-axis location
			val = vx * cos(phi) * cos(th) +
						vy * (-sin(phi) * cos(psi) + cos(phi) * sin(th) * sin(psi)) + 
						vz * (sin(phi) * sin(psi) + cos(phi) * sin(th) * cos(psi)) + 
						x; 

			if(H) {
				dx = 1.0, dy = 0.0, dz = 0.0;
				dphi =  vz*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(th)) - vy*(cos(phi)*cos(psi)  
					+ sin(phi)*sin(psi)*sin(th)) - vx*cos(th)*sin(phi);
				dth = vz*cos(phi)*cos(psi)*cos(th) - vx*cos(phi)*sin(th) + vy*cos(phi)*cos(th)*sin(psi);
				dpsi = vy*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(th)) + vz*(cos(psi)*sin(phi) 
					- cos(phi)*sin(psi)*sin(th));
			}
		} break;

		// Y-axis
		case 1: {

			// Compute the y-axis location
			val = vx * sin(phi) * cos(th) + 
						vy * (cos(phi) * cos(psi) + sin(phi) * sin(th) * sin(psi)) +
						vz * (-cos(phi) * sin(psi) + sin(phi) * sin(th) * cos(psi)) + 
						y;

			if(H) {
				dx = 0.0, dy = 1.0, dz = 0.0;
				dphi = vz*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(th)) - vy*(cos(psi)*sin(phi) 
					- cos(phi)*sin(psi)*sin(th)) + vx*cos(phi)*cos(th);
				dth = vz*cos(psi)*cos(th)*sin(phi) - vx*sin(phi)*sin(th) + vy*cos(th)*sin(phi)*sin(psi);
				dpsi = - vy*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(th)) - vz*(cos(phi)*cos(psi) 
					+ sin(phi)*sin(psi)*sin(th));
			}
		} break;

		// Z-axis
		case 2: {
		
			// Compute the z-axis location
			val = vx * -sin(th) + 
						vy * cos(th) * sin(psi) +
						vz * cos(th) * cos(psi) + 
						z;

			if(H) {
				dx = 0.0, dy = 0.0, dz = 1.0, dphi = 0.0;
				dth = - vx*cos(th) - vz*cos(psi)*sin(th) - vy*sin(psi)*sin(th);
				dpsi = vy*cos(psi)*cos(th) - vz*cos(th)*sin(psi);
			}

		} break;
	}

	// Set the matrix and return the value
	if(H) *H = Matrix_(1,6,dx,dy,dz,dphi,dth,dpsi);
	return Vector_(1, val);
}

/* ********************************************************************************************* */
