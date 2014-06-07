/**
 * @file ramp.cpp
 * @author Can Erdogan
 * @date Apr 18, 2012
 * @brief The collection of factors for the ramp example.
 */

#include <ramp.h>

using namespace std;
using namespace gtsam;

/* ********************************************************************************************* */
Vector Factors::LimitAngle::errorProxy(const LieVector& p1, const LieVector& p2) {

	double angle = fabs(atan2((p2(1) - p1(1)), (p2(0) - p1(0))));
	double error = 0.0;
	static const double angleLimit = (34.5 / 180.0) * M_PI;
	if(angle > angleLimit) error = angle - angleLimit;
	return Vector_(1, error);
}

/* ********************************************************************************************* */
Vector Factors::LimitAngle::evaluateError(const LieVector& p1, const LieVector& p2,
		boost::optional <Matrix&> H1, boost::optional <Matrix&> H2) const {
	Vector error = errorProxy(p1, p2);
	if(H1) *H1 = numericalDerivative21(errorProxy, p1, p2);
	if(H2) *H2 = numericalDerivative22(errorProxy, p1, p2);
	return error;
}

/* ********************************************************************************************* */
Vector Factors::OnSurface::errorProxy(const LieVector& p1, const LieVector& p2) {

	// Set the error
	static const double offset = 0.03;
	double error = 0.0;
	if(p2(0) > (p1(0) + offset)) {
		error = p2(0) - (p1(0) + offset);
	}
	else if(p2(0) < (p1(0) - offset)) {
		error = (p1(0) - offset) - p2(0);
	}

	// Set the derivative
	return Vector_(1, error);
}

/* ********************************************************************************************* */
Vector Factors::OnSurface::evaluateError(const LieVector& p1, const LieVector& p2,
		boost::optional <Matrix&> H1, boost::optional <Matrix&> H2) const {

	// Get the error vector
	Vector error = errorProxy(p1, p2);

	// Set the derivatives - get them using the numerical derivatives
	if(H1) *H1 = numericalDerivative21(errorProxy, p1, p2);
	if(H2) *H2 = numericalDerivative22(errorProxy, p1, p2);

	return error;
}

/* ********************************************************************************************* */
Vector Factors::DiffSlopes::errorProxy(const LieVector& a0, const LieVector& a1, 
		const LieVector& b0, const LieVector& b1) const {

	// Compute the two slopes
	double s1 = (a1(1) - a0(1)) / (a1(0) - a0(0));
	double s2 = (b1(1) - b0(1)) / (b1(0) - b0(0));
	double diff = fabs(s2 - s1);
	// cout << "diff: " << diff << endl;

	// Compute the error
	double error = 0.0;
	if(diff < 0.05) error = 0.05 - diff;
	return Vector_(1, error);
}

/* ********************************************************************************************* */
Matrix Factors::DiffSlopes::numerDeriv(const LieVector& a0, const LieVector& a1, 
		const LieVector& b0, const LieVector& b1, size_t vid) const {

	// Compute initial error
	LieVector hx = errorProxy(a0, a1, b0, b1);
	
	// Prepare the matrix
	static const double delta = 1e-5;
	size_t m = hx.dim(), n;
	if(vid == 0) n = a0.dim();
	else if(vid == 1) n = a1.dim();
	else if(vid == 2) n = b0.dim();
	else if(vid == 3) n = b1.dim();
	Matrix H = zeros(m,n);

	// Fill the matrix by retracting a delta ammount for each coordinate
	double factor = 1.0/(2.0*delta);
	Vector d = zero(n), hxplus, hxmin;
	for (size_t j=0;j<n;j++) {
		d(j) +=   delta; 
		if(vid == 0) hxplus = hx.localCoordinates(errorProxy(a0.retract(d), a1, b0, b1));
		else if(vid == 1) hxplus = hx.localCoordinates(errorProxy(a0, a1.retract(d), b0, b1));
		else if(vid == 2) hxplus = hx.localCoordinates(errorProxy(a0, a1, b0.retract(d), b1));
		else if(vid == 3) hxplus = hx.localCoordinates(errorProxy(a0, a1, b0, b1.retract(d)));
		d(j) -= 2*delta; 
		if(vid == 0) hxmin = hx.localCoordinates(errorProxy(a0.retract(d), a1, b0, b1));
		else if(vid == 1) hxmin = hx.localCoordinates(errorProxy(a0, a1.retract(d), b0, b1));
		else if(vid == 2) hxmin = hx.localCoordinates(errorProxy(a0, a1, b0.retract(d), b1));
		else if(vid == 3) hxmin = hx.localCoordinates(errorProxy(a0, a1, b0, b1.retract(d)));
		d(j) +=   delta;
		H.col(j) << (hxplus-hxmin)*factor;
	}
	return H;
}

/* ********************************************************************************************* */
Vector Factors::DiffSlopes::evaluateError(const LieVector& a0, const LieVector& a1, 
	const LieVector& b0, const LieVector& b1, boost::optional <Matrix&> H1, 
	boost::optional <Matrix&> H2, boost::optional <Matrix&> H3, boost::optional <Matrix&> H4) const {

	// Get the error vector
	Vector error = errorProxy(a0, a1, b0, b1);

	// Set the derivatives - get them using the numerical derivatives
	if(H1) *H1 = numerDeriv(a0, a1, b0, b1, 0);
	if(H2) *H2 = numerDeriv(a0, a1, b0, b1, 1);
	if(H3) *H3 = numerDeriv(a0, a1, b0, b1, 2);
	if(H4) *H4 = numerDeriv(a0, a1, b0, b1, 3);

	return error;
}

/* ********************************************************************************************* */
Vector Factors::RestPlate::errorProxy(const LieVector& a0, const LieVector& a1, 
		const LieVector& b0, const LieVector& b1) const {

	// Get the two lines
	Eigen::VectorXd a0v = a0.vector(), a1v = a1.vector(), b0v = b0.vector(), b1v = b1.vector();
	Eigen::VectorXd aLine = a1v - a0v, bLine = b1v - b0v;
	double aLength = aLine.norm(), bLength = bLine.norm();
	aLine = aLine / aLength, bLine = bLine / bLength;
	Eigen::Vector2d aPerp(-aLine(1), aLine(0)), bPerp(-bLine(1), bLine(0));

	// Get the two projections (perpendicular projection should be 0.0).
	Eigen::VectorXd va = b0v - a0v, vb = a1v - b0v;
	double aParaProj = aLine(0) * va(0) + aLine(1) * va(1);
	double bParaProj = bLine(0) * vb(0) + bLine(1) * vb(1);
	double aPerpError = aPerp(0) * va(0) + aPerp(1) * va(1);
	double bPerpError = bPerp(0) * vb(0) + bPerp(1) * vb(1);

	// Get the parallel errors
	double aParaError = 0.0, bParaError = 0.0;
	if(aParaProj > aLength) aParaError = aParaProj - aLength;
	else if(aParaProj < 0.0) aParaError = aParaProj;
	if(bParaProj > bLength / 2) bParaError = bParaProj - bLength / 2;
	else if(bParaProj < 0.0) bParaError = bParaProj;

	// Send the minimal error
	if(fabs(aPerpError) < fabs(bPerpError)) return Vector_(2, aPerpError, aParaError);
	else return Vector_(2, bPerpError, bParaError);
}

/* ********************************************************************************************* */
Matrix Factors::RestPlate::numerDeriv(const LieVector& a0, const LieVector& a1, 
		const LieVector& b0, const LieVector& b1, size_t vid) const {

	// Compute initial error
	LieVector hx = errorProxy(a0, a1, b0, b1);
	
	// Prepare the matrix
	static const double delta = 1e-5;
	size_t m = hx.dim(), n;
	if(vid == 0) n = a0.dim();
	else if(vid == 1) n = a1.dim();
	else if(vid == 2) n = b0.dim();
	else if(vid == 3) n = b1.dim();
	Matrix H = zeros(m,n);

	// Fill the matrix by retracting a delta ammount for each coordinate
	double factor = 1.0/(2.0*delta);
	Vector d = zero(n), hxplus, hxmin;
	for (size_t j=0;j<n;j++) {
		d(j) +=   delta; 
		if(vid == 0) hxplus = hx.localCoordinates(errorProxy(a0.retract(d), a1, b0, b1));
		else if(vid == 1) hxplus = hx.localCoordinates(errorProxy(a0, a1.retract(d), b0, b1));
		else if(vid == 2) hxplus = hx.localCoordinates(errorProxy(a0, a1, b0.retract(d), b1));
		else if(vid == 3) hxplus = hx.localCoordinates(errorProxy(a0, a1, b0, b1.retract(d)));
		d(j) -= 2*delta; 
		if(vid == 0) hxmin = hx.localCoordinates(errorProxy(a0.retract(d), a1, b0, b1));
		else if(vid == 1) hxmin = hx.localCoordinates(errorProxy(a0, a1.retract(d), b0, b1));
		else if(vid == 2) hxmin = hx.localCoordinates(errorProxy(a0, a1, b0.retract(d), b1));
		else if(vid == 3) hxmin = hx.localCoordinates(errorProxy(a0, a1, b0, b1.retract(d)));
		d(j) +=   delta;
		H.col(j) << (hxplus-hxmin)*factor;
	}
	return H;
}

/* ********************************************************************************************* */
Vector Factors::RestPlate::evaluateError(const LieVector& a0, const LieVector& a1, 
	const LieVector& b0, const LieVector& b1, boost::optional <Matrix&> H1, 
	boost::optional <Matrix&> H2, boost::optional <Matrix&> H3, boost::optional <Matrix&> H4) const {

	// Get the error vector
	Vector error = errorProxy(a0, a1, b0, b1);

	// Set the derivatives - get them using the numerical derivatives
	if(H1) *H1 = numerDeriv(a0, a1, b0, b1, 0);
	if(H2) *H2 = numerDeriv(a0, a1, b0, b1, 1);
	if(H3) *H3 = numerDeriv(a0, a1, b0, b1, 2);
	if(H4) *H4 = numerDeriv(a0, a1, b0, b1, 3);

	return error;
}

/* ********************************************************************************************* */
Vector Factors::Side::errorProxy(const LieVector& p1, const LieVector& p2, const LieVector& p3) 
		const {

	// Compute the projection
	Eigen::VectorXd p1v = p1.vector(), p2v = p2.vector(), p3v = p3.vector();
	Eigen::VectorXd vLine = (p2v - p1v).normalized();
	Eigen::Vector2d vPerp(-vLine(1), vLine(0));
	Eigen::VectorXd v3 = p3v - p1v;
	double proj = vPerp(0) * v3(0) + vPerp(1) * v3(1);  

	// Compute the error
	double error = 0.0;
	if(side && (proj < 0.0)) error = proj;
	else if(!side && (proj > 0.0)) error = proj;
	return Vector_(1, error);
}

/* ********************************************************************************************* */
Matrix Factors::Side::numerDeriv(const LieVector& p1, const LieVector& p2, 
		const LieVector& p3, size_t vid) const {

	// Compute initial error
	LieVector hx = errorProxy(p1, p2, p3);
	
	// Prepare the matrix
	static const double delta = 1e-5;
	size_t m = hx.dim(), n;
	if(vid == 0) n = p1.dim();
	else if(vid == 1) n = p2.dim();
	else if(vid == 2) n = p3.dim();
	Matrix H = zeros(m,n);

	// Fill the matrix by retracting a delta ammount for each coordinate
	double factor = 1.0/(2.0*delta);
	Vector d = zero(n), hxplus, hxmin;
	for (size_t j=0;j<n;j++) {
		d(j) +=   delta; 
		if(vid == 0) hxplus = hx.localCoordinates(errorProxy(p1.retract(d), p2, p3));
		else if(vid == 1) hxplus = hx.localCoordinates(errorProxy(p1, p2.retract(d), p3));
		else if(vid == 2) hxplus = hx.localCoordinates(errorProxy(p1, p2, p3.retract(d)));
		d(j) -= 2*delta; 
		if(vid == 0) hxmin = hx.localCoordinates(errorProxy(p1.retract(d), p2, p3));
		else if(vid == 1) hxmin = hx.localCoordinates(errorProxy(p1, p2.retract(d), p3));
		else if(vid == 2) hxmin = hx.localCoordinates(errorProxy(p1, p2, p3.retract(d)));
		d(j) +=   delta;
		H.col(j) << (hxplus-hxmin)*factor;
	}
	return H;
}

/* ********************************************************************************************* */
Vector Factors::Side::evaluateError(const LieVector& p1, const LieVector& p2, const LieVector& p3,
		boost::optional <Matrix&> H1, boost::optional <Matrix&> H2, boost::optional <Matrix&> H3)const {

	// Get the error vector
	Vector error = errorProxy(p1, p2, p3);

	// Set the derivatives - get them using the numerical derivatives
	if(H1) *H1 = numerDeriv(p1, p2, p3, 0);
	if(H2) *H2 = numerDeriv(p1, p2, p3, 1);
	if(H3) *H3 = numerDeriv(p1, p2, p3, 2);

	return error;
}

/* ********************************************************************************************* */
Vector Factors::Distance::evaluateError(const LieVector& p1, 
		const LieVector& p2, boost::optional <Matrix&> H1, boost::optional <Matrix&> H2) const {

	Eigen::VectorXd p1v = p1.vector(), p2v = p2.vector();
	double dx = p1v(0) - p2v(0), dy = p1v(1) - p2v(1);
	double predicted = sqrt(dx * dx + dy * dy);
	assert(predicted != 0);
	double error = predicted - distance_;
	if(H1) (*H1) = Matrix_(1, 2, dx / predicted, dy / predicted);
	if(H2) (*H2) = Matrix_(1, 2, -dx / predicted, -dy / predicted);
	return Vector_(1, error);
}

/* ********************************************************************************************* */
Matrix Factors::Alignment::numerDeriv(const LieVector& p1, const LieVector& p2, 
		const LieVector& p3, size_t vid) const {

	// Compute initial error
	LieVector hx = errorProxy(p1, p2, p3);
	
	// Prepare the matrix
	static const double delta = 1e-5;
	size_t m = hx.dim(), n;
	if(vid == 0) n = p1.dim();
	else if(vid == 1) n = p2.dim();
	else if(vid == 2) n = p3.dim();
	Matrix H = zeros(m,n);

	// Fill the matrix by retracting a delta ammount for each coordinate
	double factor = 1.0/(2.0*delta);
	Vector d = zero(n), hxplus, hxmin;
	for (size_t j=0;j<n;j++) {
		d(j) +=   delta; 
		if(vid == 0) hxplus = hx.localCoordinates(errorProxy(p1.retract(d), p2, p3));
		else if(vid == 1) hxplus = hx.localCoordinates(errorProxy(p1, p2.retract(d), p3));
		else if(vid == 2) hxplus = hx.localCoordinates(errorProxy(p1, p2, p3.retract(d)));
		d(j) -= 2*delta; 
		if(vid == 0) hxmin = hx.localCoordinates(errorProxy(p1.retract(d), p2, p3));
		else if(vid == 1) hxmin = hx.localCoordinates(errorProxy(p1, p2.retract(d), p3));
		else if(vid == 2) hxmin = hx.localCoordinates(errorProxy(p1, p2, p3.retract(d)));
		d(j) +=   delta;
		H.col(j) << (hxplus-hxmin)*factor;
	}
	return H;
}

/* ********************************************************************************************* */
Vector Factors::Alignment::errorProxy(const LieVector& p1, const LieVector& p2, const LieVector& p3) 
		const {

	static const double offset = 0.0;

	// 1- Compute the error along the line - the projection of the point to the [p1,p2] line
	// should be between p1 and p2.

	Eigen::VectorXd p1v = p1.vector();
	Eigen::VectorXd p2v = p2.vector();
	Eigen::VectorXd p3v = p3.vector();
	p3v(0) += vert(0);
	p3v(1) += vert(1);

	// Get the vector from p1 to p2
	Eigen::VectorXd vLine = p2v - p1v;
	double length = vLine.norm();
	assert(length != 0.0);
	vLine = vLine / length;

	// Get the distance between p1 and projected p3
	// p3Offset here
	Eigen::VectorXd v3 = (p3v) - p1v;
	double p3Distance = vLine(0) * v3(0) + vLine(1) * v3(1);

	// Decide which case the first error is in: (1) p3 is after p2, (2) p3 is between [p1,p2] or
	// (3) p3 is before p1 and set the error accordingly.
	size_t caseID;
	double error1;
	if(p3Distance > (length - offset)) {
		caseID = 0;
		error1 = (length - offset) - p3Distance;
	}
	else if(p3Distance < offset) {
		caseID = 2;
		error1 = p3Distance - offset;
	}
	else {
		caseID = 1;
		error1 = 0.0;
	}

	// 2- Compute the error associated with the perpendicular distance of p3 from line [p1,p2]

	// Get the perpendicular vector
	Eigen::Vector2d vPerp(-vLine(1), vLine(0));

	// The error is the projection of p3 to the perpendicular vector
	double error2 = vPerp(0) * v3(0) + vPerp(1) * v3(1) + 0.01;

	// 3- Return the error
	return Vector_(2, error1, error2);
}

/* ********************************************************************************************* */
Vector Factors::Alignment::evaluateError(const LieVector& p1, const LieVector& p2, const LieVector& p3,
		boost::optional <Matrix&> H1, boost::optional <Matrix&> H2, boost::optional <Matrix&> H3)const {

	// Get the error vector
	Vector error = errorProxy(p1, p2, p3);

	// Set the derivatives - get them using the numerical derivatives
	if(H1) *H1 = numerDeriv(p1, p2, p3, 0);
	if(H2) *H2 = numerDeriv(p1, p2, p3, 1);
	if(H3) *H3 = numerDeriv(p1, p2, p3, 2);

	return error;
}
