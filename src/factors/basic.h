/**
 * @file basic.h
 * @author Can Erdogan
 * @date Apr 18, 2012
 * @brief The collection of factors necessary for the recursive approach.
 */

#pragma once

#include <gtsam/base/LieScalar.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/BoundingConstraint.h>
#include <gtsam/slam/PriorFactor.h>

/* ********************************************************************************************* */
/// The definition of factors
namespace Factors {

	using namespace gtsam;

	/* ******************************************************************************************** */
	/// Creates a bounding factor on a liescalar using another one
	struct Ordering1D : public BoundingConstraint2 <LieScalar, LieScalar> {

		/// The constructor
		Ordering1D(Key key1, Key key2, double threshold, bool isGreaterThan, double mu = 1000.0) :
				BoundingConstraint2(key1, key2, threshold, isGreaterThan, mu) {
		}

		/// The value function that should be implemented
		double value(const LieScalar& s1, const LieScalar& s2, 
				boost::optional <Matrix&> H1=boost::none, boost::optional <Matrix&> H2 = boost::none) const{
			if(H1) *H1 = Matrix_(1, 1, -1.0);
			if(H2) *H2 = Matrix_(1, 1, 1.0);
			return (s2.value() - s1.value());
		}
	};

	/* ******************************************************************************************** */
	/// Creates a bounding factor on one of the fields of a 2D point using the second 2D point
	struct Ordering : public BoundingConstraint2 <LieVector, LieVector> {

		bool x_;  ///< The indicator whether this is for the "x" field or "y" field

		/// The constructor
		Ordering(Key key1, Key key2, bool x, double threshold, bool isGreaterThan, double mu = 1000.0) :
				BoundingConstraint2(key1, key2, threshold, isGreaterThan, mu), x_(x) {
		}

		/// The value function that should be implemented
		double value(const LieVector& p1, const LieVector& p2, boost::optional <Matrix&> H1 = boost::none,
				boost::optional <Matrix&> H2 = boost::none) const	{
			if(H1) *H1 = x_ ? Matrix_(1, 2, -1.0, 0.0) : Matrix_(1, 2, 0.0, -1.0);
			if(H2) *H2 = x_ ? Matrix_(1, 2, 1.0, 0.0) : Matrix_(1, 2, 0.0, 1.0);
			Eigen::VectorXd p1v = p1.vector(), p2v = p2.vector();
			return x_ ? (p2v(0) - p1v(0)) : (p2v(1) - p1v(1));
		}
	};

	/* ******************************************************************************************** */
	/// Creates a Gaussian on one of the fields of a 2D point
	struct Prior1D : public NoiseModelFactor1 <LieVector> {

		bool x_;  ///< The indicator whether this is for the "x" field or "y" field
		double value_;  ///< The value which we want "x" or "y" to be

		/// The constructor
		Prior1D(Key key, bool x, double value, const SharedNoiseModel& model) :
				NoiseModelFactor1 <LieVector>(model, key), value_(value), x_(x) {
		}

		/// Error function that defines the manifold
		Vector evaluateError(const LieVector& p, boost::optional <Matrix&> H = boost::none) const {
			Eigen::VectorXd pv = p.vector();
			double error = x_ ? pv(0) - value_ : pv(1) - value_;
			if(H) (*H) = x_ ? Matrix_(1, 2, 1.0, 0.0) : Matrix_(1, 2, 0.0, 1.0);
			return Vector_(1, error);
		}
	};

/* ******************************************************************************************** */
	/// Creates a bounding factor on a LieVector 
	struct PriorVector : public NoiseModelFactor1 <LieVector> {

		size_t index;
		double prior;

		PriorVector (Key k, size_t index_, double prior_, const SharedNoiseModel& model):
			 NoiseModelFactor1 <LieVector> (model, k), index(index_), prior(prior_) {}

		Vector evaluateError(const LieVector& v, boost::optional <Matrix&> H = boost::none) const {
			if(H) {
				*H = zeros(1, v.dim());
				(*H)(index) = 1.0;
			}
			Eigen::VectorXd val = v.vector();
			return Vector_(1, val(index) - prior);
		}
	};

	/* ******************************************************************************************** */
	/// Creates a bounding factor on a LieVector with another LieVector
	struct BoundingVector2 : public BoundingConstraint2 <LieVector, LieVector> {

		size_t index;

		BoundingVector2(Key k1, Key k2, size_t index_, double threshold, bool isGreaterThan, 
			double mu = 1000.0)	
			: BoundingConstraint2(k1, k2, threshold, isGreaterThan, mu), index(index_) {}

		double value(const LieVector& v1, const LieVector& v2, 
				boost::optional <Matrix&> H1=boost::none, boost::optional <Matrix&> H2 = boost::none) const{
			if(H1) {
				*H1 = zeros(1, v1.dim());
				(*H1)(index) = 1.0;
			}
			if(H2) {
				*H2 = zeros(1, v1.dim());
				(*H2)(index) = -1.0;
			}

			Eigen::VectorXd val1 = v1.vector();
			Eigen::VectorXd val2 = v2.vector();
			return val1(index) - val2(index);
		}
	};

	/* ******************************************************************************************** */
	/// Creates a bounding factor on a LieVector 
	struct BoundingVector : public BoundingConstraint1 <LieVector> {

		size_t index;

		BoundingVector(Key key, size_t index_, double threshold, bool isGreaterThan, double mu = 1000.0) 				
			: BoundingConstraint1(key, threshold, isGreaterThan, mu), index(index_) {}

		double value(const LieVector& v, boost::optional <Matrix&> H = boost::none) const {
			if(H) {
				*H = zeros(1, v.dim());
				(*H)(index) = 1.0;
			}
			Eigen::VectorXd val = v.vector();
			return val(index);
		}
	};

	/* ******************************************************************************************** */
	/// Creates a bounding factor on a LieScalar 
	struct Bounding1D : public BoundingConstraint1 <LieScalar> {

		/// The constructor -  the key "isGreaterThan" threshold
		Bounding1D(Key key, double threshold, bool isGreaterThan, double mu = 1000.0) :
				BoundingConstraint1(key, threshold, isGreaterThan, mu) {
		}

		/// The value function that should be implemented
		double value(const LieScalar& v, boost::optional <Matrix&> H = boost::none) const {
			if(H) *H = Matrix_(1, 1, 1.0);
			return v.value();
		}
	};

	/* ******************************************************************************************** */
	/// Creates a bounding factor on one of the fields of a 2D point with the fixed "threshold"
	struct Bounding2D : public BoundingConstraint1 <LieVector> {

		bool x_;  ///< The indicator whether this is for the "x" field or "y" field

		/// The constructor - x/y of the key "isGreaterThan" threshold
		Bounding2D(Key key, bool x, double threshold, bool isGreaterThan, double mu = 1000.0) :
				BoundingConstraint1(key, threshold, isGreaterThan, mu), x_(x) {
		}

		/// The value function that should be implemented
		double value(const LieVector& p, boost::optional <Matrix&> H = boost::none) const {
			if(H) *H = x_ ? Matrix_(1, 2, 1.0, 0.0) : Matrix_(1, 2, 0.0, 1.0);
			return x_ ? p(0) : p(1);
		}
	};

};

