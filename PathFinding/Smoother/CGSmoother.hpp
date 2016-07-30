#ifndef CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP
#define CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP

#include <vector>

#include "../../VehicleModel/VehicleModel.hpp"
#include "../../GridMap/InternalGridMap.hpp"
#include "../../Entities/State2D.hpp"

namespace astar {

// define the minimizer status
enum CGStatus {CGIddle, CGContinue, CGStuck, CGSuccess, CGFailure};

class CGSmoother {

    private:

        // PRIVATE ATTRIBUTES

		// the obstacle weight
		double wo;

		// the smooth curvature weight
		double ws;

		// the voronoi potential field weight
		double wp;

		// the curvature weight
		double wk;

		// the max obstacle distance
		double dmax;

		// the inverse dmax squared distance
		double inverse_dmax2;

		// the max voronoi distance
		double vorodmax;

		// the alpha voronoi parameter
		double alpha;

		// the internal grid map reference
		astar::InternalGridMapRef grid;

		// the vehicle model reference
		astar::VehicleModelRef vehicle;

		// the maximum allowed curvature
		double kmax;

		// THE MINIMIZER CONTEXT ATTRIBUTES

		// the minizer state
		CGStatus cg_status;

		// the solution vector
		astar::StateArrayPtr x;

		// the cost function evaluated at x
		double fx;

		// the solution vector at the next position
		astar::StateArrayPtr x1;

		// the cost function evaluated at x1
		double fx1;

		// the steepest direction vector
		std::vector<astar::Vector2D<double>> gradient;

		// the gradient norm - Evaluated at the first point x
		double gx_norm;

		// the gradient norm - Evaluated at the second point x1
		double gx1_norm;

		// the direction vector (Krylov Space)
		std::vector<astar::Vector2D<double>> s;

		// the direction vector norm
		double s_norm;

		// store the info about the free and locked positions
		std::vector<bool> locked_positions;

		// how many iterations?
		unsigned int max_iterations;

		// the problem dimension
		unsigned int dim;

		// the next step size
		double step;

		// the tolerance
		double tol;

		// PRIVATE METHODS

		// the main function to be minimized
		double CostFunction(astar::StateArrayPtr);

		// get the obstacle and voronoi contribution
		astar::Vector2D<double> GetObstacleAndVoronoiDerivatives(const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&);

		// get the curvature contribution
		astar::Vector2D<double> GetCurvatureDerivative(const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&);

		// get the smootheness contribution
		astar::Vector2D<double> GetSmoothPathDerivative(
				const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&);

		// build the cost function gradient evaluated at a given input path and returns the gradient's norm
		double ComputeGradient(astar::StateArrayPtr const path);

		// reset all values in a given position array
		void SetZero(std::vector<astar::Vector2D<double>> &v);

		// custom dot product
		double DotProduct(const std::vector<astar::Vector2D<double>> &a, const std::vector<astar::Vector2D<double>> &b);

		// scale a given Vector2D array
		void ScaleVector(double value, std::vector<astar::Vector2D<double>> &v);

		// add the first vector2D array to the second one
		void AddVectors(const std::vector<astar::Vector2D<double>> &a, std::vector<astar::Vector2D<double>> &b);

		// compute a norm 2 for a Vector2D array
		double VectorNorm2(const std::vector<astar::Vector2D<double>> &v);

		// take a fixed step at the current direction vector (s)
		void TakeStep(double factor);

		// compare the two last solution vectors, if they are equal we are stuck
		bool isStuck(const std::vector<astar::State2D> &current, const std::vector<astar::State2D> &next);

		// the vanilla line search approach
		// do a line minimization in the region (xa, fa) (xc, fc) to find an intermediate (xb, fb)
		// satisfying fa > fb < fc. Using parabolic interpolation
		double LineSearch(double fa, double fc, double lambda, double sg);

		// Starting at (x0, f0) move along the direction p to find a minimum
		// f(x0 - lambda * p), returning the new point x1 = x0-lambda*p,
		// f1=f(x1) and g1 = grad(f) at x1.
		void WalkDownhill(double fa, double fc, double lambda, double sg);

		// setup the first iteration
		bool Setup(astar::StateArrayPtr raw_path);

		// minimize the cost function based on a given state array path
		void Minimize(astar::StateArrayPtr path);

		// interpolate a given path
		astar::StateArrayPtr Interpolate(astar::StateArrayPtr);

		bool Iterate();

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

		// the basic constructor
		CGSmoother(astar::InternalGridMapRef, astar::VehicleModelRef);

		// basic destructor
		~CGSmoother();

        // smooth a given path
        astar::StateArrayPtr Smooth(astar::InternalGridMapRef, astar::VehicleModelRef, astar::StateArrayPtr);

};

}

#endif


