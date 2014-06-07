/**
 * @file lever.h
 * @author Can Erdogan
 * @date Oct 03, 2013
 * @brief The collection of factors necessary to design a lever structure.
 */

#pragma once

#include <gtsam/base/LieScalar.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BoundingConstraint.h>

/* ********************************************************************************************* */
/// The definition of factors
namespace Factors {

	using namespace gtsam;

	/// Creates a Gaussian on one of the fields of the vertex of a mesh transformed by the LieVector
	struct PriorVertex : public NoiseModelFactor1 <LieVector> {

		Eigen::Vector3d v;		///< Vertex location in local frame
		size_t axis;					///< 0 for x, 1 for y and 2 for z

		PriorVertex(Key key, const Eigen::Vector3d& v_, size_t axis_, const SharedNoiseModel& model) :
				NoiseModelFactor1 <LieVector>(model, key), v(v_), axis(axis_) {}

		Vector evaluateError(const LieVector& t, boost::optional <Matrix&> H = boost::none) const;
	};

	/// Ensures that the contact point is on the given plane defined in local coordinates of the 
	/// part. Variables: point coord. & the transformation of the part
	struct VertexOnTriPlane : public NoiseModelFactor2 <LieVector, LieVector> {

		Eigen::Vector4d plane;		///< Plane in the local frame

		VertexOnTriPlane(Key k1, Key k2, const Eigen::Vector4d& p_, const SharedNoiseModel& m) :
			NoiseModelFactor2 <LieVector, LieVector>(m, k1, k2), plane(p_) { }

		Vector evaluateError(const LieVector& t, const LieVector& p, 
			boost::optional <Matrix&> H1 = boost::none, boost::optional <Matrix&> H2 = boost::none) const;
	};

	/// Ensures the point is at the given side of the plane transformed by the 6d vector
	struct VertexInTri : public BoundingConstraint2 <LieVector, LieVector> {

		Eigen::Vector4d plane;		///< Plane in the local frame

		VertexInTri(Key k1, Key k2, const Eigen::Vector4d& p_, bool s, double mu = 1000.0) :
			BoundingConstraint2<LieVector, LieVector>(k1, k2, 0.0, s, mu), plane(p_) { }

		double value (const LieVector& t, const LieVector& p, 
			boost::optional <Matrix&> H1 = boost::none, boost::optional <Matrix&> H2 = boost::none) const;
	};

	/// Ensures that the point is on a line defined by the edge of a mesh
	struct VertexOnLine : public NoiseModelFactor2 <LieVector, LieVector> {

		Eigen::Vector3d v1, v21;	///< the edge point and the unit vector to the second point

		VertexOnLine(Key k1, Key k2, const Eigen::Vector3d& v1_, const Eigen::Vector3d& v21_, 
			const SharedNoiseModel& m):NoiseModelFactor2<LieVector,LieVector>(m,k1,k2),v1(v1_),v21(v21_){}

		Vector evaluateError(const LieVector& t, const LieVector& p, 
			boost::optional <Matrix&> H1 = boost::none, boost::optional <Matrix&> H2 = boost::none) const;
	};

	/// Ensures that two points are at some minimum distance
	struct MinDistance: public BoundingConstraint2 <LieVector, LieVector> {

		MinDistance(Key k1, Key k2, double d, bool s, double mu = 1000.0) :
			BoundingConstraint2<LieVector, LieVector>(k1, k2, d, s, mu) { }

		double value (const LieVector& v1, const LieVector& v2, 
			boost::optional <Matrix&> H1 = boost::none, boost::optional <Matrix&> H2 = boost::none) const;
	};

	/// Equates the torque t that is applied to the edge of the fulcrum mesh that is transformed by T 
	/// and the contact point v with force f which are transformed by T2. v21p and vfnp are unit.
	struct Torque: public NoiseModelFactor4 <LieVector, LieVector, LieVector, LieVector> {

		Eigen::Vector3d v1p, v2p;	///< the edge point and the unit vector to the second point
		Eigen::Vector3d vfnp;			///< the direction of the force as dictated by the normal of the mesh
		double force;

		Torque(Key kTor, Key kTf, Key kTl, Key kV, const Eigen::Vector3d& v1p_, 
			const Eigen::Vector3d& v2p_, const Eigen::Vector3d& vfnp_, double force_, 
			const SharedNoiseModel& m) :
			NoiseModelFactor4 <LieVector, LieVector, LieVector, LieVector>(m, kTor, kTf, kTl, kV), 
			v1p(v1p_), v2p(v2p_), vfnp(vfnp_), force(force_) {}

		Matrix numerDeriv(const LieVector& torque, const LieVector& tf, const LieVector& tl, 
			const LieVector& v, size_t vid) const;

		Vector errorProxy(const LieVector& torque, const LieVector& tf, const LieVector& tl, 
			const LieVector& v) const;

		Vector evaluateError(const LieVector& torque, const LieVector& tf, const LieVector& tl, 
			const LieVector& v, 
			boost::optional <Matrix&> H1 = boost::none, boost::optional <Matrix&> H2 = boost::none, 
			boost::optional <Matrix&> H3 = boost::none, boost::optional <Matrix&> H4 =boost::none) const;
	};
};
