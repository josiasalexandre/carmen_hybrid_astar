#ifndef CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP
#define CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP

#include <vector>

#include "../../VehicleModel/VehicleModel.hpp"
#include "../../GridMap/InternalGridMap.hpp"
#include "../../Entities/State2D.hpp"

namespace astar {

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

		// the max voronoi distance
		double vorodmax;

		// the inverse dmax squared distance
		double inverse_dmax2;

		// the alpha voronoi parameter
		double alpha;

		// the internal grid map reference
		astar::InternalGridMapRef grid;

		// the vehicle model reference
		astar::VehicleModelRef vehicle;

		// some Vector2D helpers
		astar::Vector2D<double> tmp;

		// the maximum allowed curvature
		double kmax;

		// THE MINIMIZER CONTEXT ATTRIBUTES

		// PRIVATE METHODS

		// the main function to be minimized
		double CostFunction(astar::StateArrayPtr);

		// get the obstacle and voronoi contribution
		astar::Vector2D<double> GetObstacleAndVoronoiDerivatives(const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&);

		// get the curvature contribution
		astar::Vector2D<double> GetCurvatureDerivatives(const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&);

		// get the smootheness contribution
		astar::Vector2D<double> GetSmoothPathDerivatives(
				const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&);

		// build the cost function gradient
		astar::Vector2DArrayPtr<double> Gradient(astar::StateArrayPtr);

		bool Iterate();

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

		// the basic constructor
		CGSmoother(astar::InternalGridMapRef, astar::VehicleModelRef);

        // smooth a given path
        void Smooth(astar::InternalGridMapRef, astar::VehicleModelRef, astar::StateArrayPtr);

};

}

#endif
