/**
 * @file Common.h
 * @author Can Erdogan
 * @date Sep 11, 2013
 * @brief This file contains common functions and classes.
 */

#pragma once

#include <vector>
#include <cstdarg>
#include <stdio.h>

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/LieScalar.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/BetweenFactor.h>

/* ********************************************************************************************* */
static gtsam::SharedNoiseModel model = 
		gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(1, 1e-3));  ///< 1D noise model

/* ******************************************************************************************** */
#define ADD(x) graph->add(x)
typedef gtsam::NonlinearFactorGraph Graph;  

/* ******************************************************************************************** */
/// Creates a vector from the given argument list
std::vector <size_t> vec (int num, ... ) {
	va_list args;
	va_start (args, num);
	std::vector <size_t> v;
	for(int i = 0; i < num; i++)
		v.push_back(va_arg(args, size_t));
	va_end (args);
	return v;
}

/* ******************************************************************************************** */
/// Creates a random vector within the given limits
gtsam::Vector randomVector(const gtsam::Vector& minLimits, const gtsam::Vector& maxLimits) {

	// Get the number of dimensions and create the return vector
	size_t numDims = gtsam::dim(minLimits);
	gtsam::Vector vector = gtsam::zero(numDims);

	// Create the random vector
	for(size_t i = 0; i < numDims; i++) {
		double range = maxLimits(i) - minLimits(i);
		vector(i) = (((double) rand()) / RAND_MAX) * range + minLimits(i);
	}
	return vector;
}

/* ******************************************************************************************** */
/// Optimizes for the given graph with random restarts where the initial values are sampled
/// from the given bounding box limit.
gtsam::Values solveGraph(Graph& graph, double maxValue, size_t numIterations = 1000) {

	// Try different instantiations
	size_t attempt;
	size_t numKeys = graph.keys().size();
	for(attempt = 0; attempt < numIterations; attempt++) {

		// if(attempt % 100 == 0) printf("Attempt %lu...\n", attempt);

		// Set random values for initialization
		gtsam::Values values;
		gtsam::Vector minLimits = gtsam::zeros(numKeys, 1), maxLimits = maxValue * gtsam::ones(numKeys);
//		minLimits = -maxLimits;
		gtsam::Vector random = randomVector(minLimits, maxLimits);
		for(size_t i = 0; i < numKeys; i++)
			values.insert(i, gtsam::LieScalar((double) random(i)));

		// Optimize
		gtsam::LevenbergMarquardtParams params;
		params.absoluteErrorTol = 1e-15;
		params.relativeErrorTol = 1e-15;
		gtsam::Values result;
		double error;
		try {
			gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, params);
			result = optimizer.optimize();
			error = graph.error(result);
		}
		catch (std::exception& e) {
		//	throw e;
//			printf("WARNING: GTSAM exception\n");
			continue;
		}

		// Stop if a zero error is attained.
		if(!result.empty() && error < 1e-2) {
			// result.print("\n\n\nRESULT::\n");
			printf("attempt: %lu, error: %lf\n", attempt, error);
			return result;
		}
//		else std::cout << "err: " << error << std::endl;
	}

	printf(".\n");
	return gtsam::Values();
}

