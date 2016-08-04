#ifndef CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP
#define CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP

#include <vector>

#include "../../VehicleModel/VehicleModel.hpp"
#include "../../GridMap/InternalGridMap.hpp"
#include "../../Entities/State2D.hpp"
#include "../../Entities/Vector2DArray.hpp"

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
        double inverse_vorodmax2;

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

        // the gradient evaluated at x
        astar::Vector2DArrayPtr<double> gx;

        // the gradient norm - Evaluated at the first point x
        double gx_norm;

        // the solution vector at the next position
        astar::StateArrayPtr x1;

        // the cost function evaluated at x1
        double fx1;

        // the gradient evaluated at x1
        astar::Vector2DArrayPtr<double> gx1;

        // the gradient norm - Evaluated at the second point x1
        double gx1_norm;

        // the best solution so far
        astar::StateArrayPtr bestx;

        // the best solution function value
        double bestfx;

        // the best solution gradient
        astar::Vector2DArrayPtr<double> bestgx;

        // the best solution gradient norm
        double bestgx_norm;

        // the displacement between the next and previous gradient
        std::vector<astar::Vector2D<double>> gx1mgx;

        // the displacement between the next and the previous position
        std::vector<astar::Vector2D<double>> x1mx;

        // the difference between the old and the new path
        double x1mx_norm;

        // the direction vector (Krylov Space)
        std::vector<astar::Vector2D<double>> s;

        // the direction vector norm
        double s_norm;

        // the directional derivative value
        double sg;

        // store the info about the free and locked positions
        std::vector<bool> locked_positions;

        // how many iterations?
        unsigned int max_iterations;

        // the problem dimension
        unsigned int dim;

        // the next step size
        double step;

        // the default step length
        double default_step_length;

        // the max step size limit
        double stepmax;

        // the min step size limit
        double stepmin;

        // the function tolerance
        double ftol;

        // the gradient tolerance
        double gtol;

        // the solution progress tolerance
        double xtol;

        // PRIVATE METHODS

        // get the greater number considering the absolute values
        double ABSMax(double a, double b, double c);

        // the main function to be minimized
        double CostFunction(astar::StateArrayPtr);

        // get the obstacle and voronoi contribution
        astar::Vector2D<double> GetObstacleAndVoronoiDerivatives(const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&);

        // get the obstacle and voronoi contribution
        // overloaded version
        astar::Vector2D<double> GetObstacleAndVoronoiDerivatives(
                const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&, double nearest_obstacle_distance, double nearest_voro_distance);

        // get the curvature contribution
        astar::Vector2D<double> GetCurvatureDerivative(const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&);

        // get the smootheness contribution
        astar::Vector2D<double> GetSmoothPathDerivative(
                const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&, const astar::Vector2D<double>&);

        // build the cost function gradient evaluated at a given input path and returns the gradient's norm
        double ComputeGradient(astar::StateArrayPtr const path, std::vector<astar::Vector2D<double>> &gradient);

        // take a fixed step at the current direction vector (s)
        void TakeStep(double factor);

        // custom function to evaluate the cost function and the update the gradient at the same time
        // it uses the x1 as the input array
        // the resulting cost value is saved in the internal fx1 variable
        // the gradient vector gx1 is updated and the gradient norm is saved in gx1_norm
        void EvaluateFunctionAndGradient();

        // The purpose of cstep is to compute a safeguarded step for
        // a linesearch and to update an interval of uncertainty for
        // a minimizer of the function.
        // It's the cstep function provided by the minpack library and it is adapted to our context here
        //  Argonne National Laboratory. MINPACK Project. June 1983
        // Jorge J. More', David J. Thuente
        int CStep(
                    double& stl, double& fx, double& sgl,
                    double& stu, double& fu, double& sgu,
                    double& stp, double& fp, double& sgp,
                    bool& bracket, double stmin, double stmax);


        // the More-Thuente line serch
        // based on the minpack and derivates codes
        int MTLineSearch(double lambda);

        // setup the first iteration
        bool Setup(astar::StateArrayPtr raw_path, bool locked);

        // update the conjugate direction -> s(i+1) = -gradient + gamma * s(i)
        void UpdateConjugateDirection(std::vector<astar::Vector2D<double>> &s, const std::vector<astar::Vector2D<double>> &gradient, double gamma);

        // the Polak-Ribiere Conjugate Gradient Method With Moré-Thuente Line Search
        void ConjugateGradientPR(astar::StateArrayPtr path, bool locked = false);

        // interpolate a given path
        astar::StateArrayPtr Interpolate(astar::StateArrayPtr);

        // show the current path in the map
        void ShowPath(astar::StateArrayPtr, bool plot_locked = true);

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


