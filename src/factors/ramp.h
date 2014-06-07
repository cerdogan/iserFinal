/**
 * @file ramp.h
 * @author Can Erdogan
 * @date Apr 18, 2012
 * @brief The collection of factors for the ramp example.
 */

#pragma once

#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/numericalDerivative.h>

namespace Factors {

	using namespace gtsam;

	/* ******************************************************************************************** */
	/// Limits the angle
	struct LimitAngle : public NoiseModelFactor2 <LieVector, LieVector> {

		LimitAngle(Key key1, Key key2, const SharedNoiseModel& model) :
			NoiseModelFactor2 <LieVector, LieVector>(model, key1, key2) { }

		static Vector errorProxy(const LieVector& p1, const LieVector& p2);

		Vector evaluateError(const LieVector& p1, const LieVector& p2,
			 boost::optional <Matrix&> H1= boost::none, boost::optional <Matrix&> H2 = boost::none) const;
	};

	/* ******************************************************************************************** */
	/// Ensures a point is on the surface of a box
	struct OnSurface : public NoiseModelFactor2 <LieVector, LieVector> {

		OnSurface(Key key1, Key key2, const SharedNoiseModel& model) :
			NoiseModelFactor2 <LieVector, LieVector>(model, key1, key2) { }

		static Vector errorProxy(const LieVector& p1, const LieVector& p2);

		Vector evaluateError(const LieVector& p1, const LieVector& p2,
			 boost::optional <Matrix&> H1= boost::none, boost::optional <Matrix&> H2 = boost::none) const;
	};

	/* ******************************************************************************************** */
	/// Vertex 1 should be on one of the two sides of a line defined by the vertices [2,3]
	struct Side : public NoiseModelFactor3 <LieVector, LieVector, LieVector> {

		bool side;

		/// The constructor
		Side(Key k1, Key k2, Key k3, bool s, const SharedNoiseModel& model) : 
			NoiseModelFactor3 <LieVector, LieVector, LieVector>(model, k1, k2, k3) { side = s; }

		Matrix numerDeriv(const LieVector& p1, const LieVector& p2,const LieVector& p3, size_t v)const;

		Vector errorProxy(const LieVector& p1, const LieVector& p2, const LieVector& p3) const;

		Vector evaluateError(const LieVector& p1, const LieVector& p2, const LieVector& p3, 
			boost::optional <Matrix&> H1 = boost::none, boost::optional <Matrix&> H2 = boost::none, 
			boost::optional <Matrix&> H3 = boost::none) const;
	};

	/* ******************************************************************************************** */
	/// Different slopes 
	struct DiffSlopes: public NoiseModelFactor4 <LieVector, LieVector, LieVector, LieVector> {

		DiffSlopes(Key k1, Key k2, Key k3, Key k4, const SharedNoiseModel& model) : 
				NoiseModelFactor4 <LieVector, LieVector, LieVector, LieVector>(model, k1, k2, k3, k4) { }

		Matrix numerDeriv(const LieVector& p1, const LieVector& p2, const LieVector& p3, 
				const LieVector& p4, size_t v)const;

		Vector errorProxy(const LieVector& a0, const LieVector& a1, const LieVector& b0,
				const LieVector& b1) const;

		Vector evaluateError(const LieVector& a0, const LieVector& a1, const LieVector& b0, 
			const LieVector& b1, 
			boost::optional <Matrix&> H1 = boost::none, boost::optional <Matrix&> H2 = boost::none, 
			boost::optional <Matrix&> H3 = boost::none, boost::optional <Matrix&> H4 = boost::none) const;
	};

	/* ******************************************************************************************** */
	/// Rests a plate on another (alternative to alignment)
	struct RestPlate: public NoiseModelFactor4 <LieVector, LieVector, LieVector, LieVector> {

		RestPlate(Key k1, Key k2, Key k3, Key k4, const SharedNoiseModel& model) : 
				NoiseModelFactor4 <LieVector, LieVector, LieVector, LieVector>(model, k1, k2, k3, k4) { }

		Matrix numerDeriv(const LieVector& p1, const LieVector& p2, const LieVector& p3, 
				const LieVector& p4, size_t v)const;

		Vector errorProxy(const LieVector& a0, const LieVector& a1, const LieVector& b0,
				const LieVector& b1) const;

		Vector evaluateError(const LieVector& a0, const LieVector& a1, const LieVector& b0, 
			const LieVector& b1, 
			boost::optional <Matrix&> H1 = boost::none, boost::optional <Matrix&> H2 = boost::none, 
			boost::optional <Matrix&> H3 = boost::none, boost::optional <Matrix&> H4 = boost::none) const;
	};

	/* ******************************************************************************************** */
	/// Aligns the third point in the middle of the 1st and 2nd points
	struct Alignment : public NoiseModelFactor3 <LieVector, LieVector, LieVector> {

		Eigen::Vector2d vert;

		/// The constructor
		Alignment(Key k1, Key k2, Key k3, const Eigen::Vector2d& v, const SharedNoiseModel& model) : 
			NoiseModelFactor3 <LieVector, LieVector, LieVector>(model, k1, k2, k3), vert(v) { }

		Matrix numerDeriv(const LieVector& p1, const LieVector& p2,const LieVector& p3, size_t v) const;

		Vector errorProxy(const LieVector& p1, const LieVector& p2, const LieVector& p3) const;

		Vector evaluateError(const LieVector& p1, const LieVector& p2, const LieVector& p3, 
			boost::optional <Matrix&> H1 = boost::none, boost::optional <Matrix&> H2 = boost::none, 
			boost::optional <Matrix&> H3 = boost::none) const;
	};

	/* ******************************************************************************************** */
	/// Regulates the distance between two 2D points to the given distance
	struct Distance : public NoiseModelFactor2 <LieVector, LieVector> {

		double distance_;  ///< The distance we want the two 2D points to be at.

		/// The constructor
		Distance(Key key1, Key key2, double distance, const SharedNoiseModel& model) :
			NoiseModelFactor2 <LieVector, LieVector>(model, key1, key2), distance_(distance) {
		}

		/// Error function that defines the manifold
		Vector evaluateError(const LieVector& p1, const LieVector& p2,
			 boost::optional <Matrix&> H1= boost::none, boost::optional <Matrix&> H2 = boost::none) const;
	};
};
