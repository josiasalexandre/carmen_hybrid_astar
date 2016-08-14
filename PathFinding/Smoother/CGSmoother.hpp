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

        // the input path
        astar::StateArrayPtr input_path;

        // the solution vector
        astar::Vector2DArrayPtr<double> x;

        // the cost function evaluated at x
        double fx;

        // the gradient evaluated at x
        astar::Vector2DArrayPtr<double> gx;

        // the gradient norm - Evaluated at the first point x
        double gx_norm;

        // the solution vector at the next position
        astar::Vector2DArrayPtr<double> x1;

        // the cost function evaluated at x1
        double fx1;

        // the gradient evaluated at x1
        astar::Vector2DArrayPtr<double> gx1;

        // the gradient norm - Evaluated at the second point x1
        double gx1_norm;

        // the nest trial solution
        astar::Vector2DArrayPtr<double> trialx;

        // the best solution function value
        double ftrialx;

        // the best solution gradient
        astar::Vector2DArrayPtr<double> gtrialx;

        // the best solution gradient norm
        double gtrialx_norm;

        // the displacement between the next and previous gradient
        std::vector<astar::Vector2D<double>> gx1mgx;

        // the norm of the displacement vector between the old and the best new path
        double x1mx_norm;

        // the norm of the displacement vector between the old and the the trial path
        double trialxmx_norm;

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

        // the Interpolation context attributes

        // register the stopping points
        std::vector<unsigned int> stopping_points;

        // PRIVATE METHODS

        // verify if a given path is unsafe
        bool UnsafePath(astar::Vector2DArrayPtr<double>);

        // get the greater number considering the absolute values
        double ABSMax(double a, double b, double c);

        // the main function to be minimized
        void CostFunction();

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
        void ComputeGradient();

        // take a fixed step at the current direction vector (s)
        void TakeStep(double factor);

        // Evaluate the function at the given point
        inline void EvaluateF(
                const astar::Vector2D<double> &xim1,
                const astar::Vector2D<double> &xi,
                const astar::Vector2D<double> &xip1,
                double &obstacle,
                double &potential,
                double &smooth,
                double &curvature
                );

        // Evaluate the obstacle, potential field and curvature gradient contributions
        inline void EvaluateG(
                const astar::Vector2D<double> &xim1,
                const astar::Vector2D<double> &xi,
                const astar::Vector2D<double> &xip1,
                astar::Vector2D<double> &gradient
                );

        // Evaluate the function and the obstacle/potential field gradients
        inline void EvaluateFG(
                const astar::Vector2D<double> &xim1,
                const astar::Vector2D<double> &xi,
                const astar::Vector2D<double> &xip1,
                double &obstacle,
                double &potential,
                double &smooth,
                double &curvature,
                astar::Vector2D<double> &gradient
                );


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

        // configure the stopping points and break the path into subpaths
        void BreakPath(astar::StateArrayPtr raw_path);

        // setup the first iteration
        bool Setup(astar::StateArrayPtr raw_path, bool locked);

        // update the conjugate direction -> s(i+1) = -gradient + gamma * s(i)
        void UpdateConjugateDirection(std::vector<astar::Vector2D<double>> &s, const std::vector<astar::Vector2D<double>> &gradient, double gamma);

        // the Polak-Ribiere Conjugate Gradient Method With Moré-Thuente Line Search
        void ConjugateGradientPR(astar::StateArrayPtr path, bool locked = false);

        // copy the current solution to the input path
        void InputPathUpdate(astar::Vector2DArrayPtr<double>, astar::StateArrayPtr);

        // show the current path in the map
        void ShowPath(astar::StateArrayPtr, bool plot_locked = true);

        // get a bezier point given four points and the time
        inline astar::Vector2D<double> GetBezierPoint(std::vector<astar::Vector2D<double>> &points, double t);

        // build a set of control points between the states
        void BuildBezierControlPoints(
                const std::vector<astar::State2D>&,
                std::vector<astar::Vector2D<double>> &p1,
                std::vector<astar::Vector2D<double>> &p2,
                unsigned int, unsigned int);

        // build a bezier curve passing through a set of states
        void DrawBezierCurve(
                const std::vector<astar::State2D>&,
                std::vector<astar::State2D> &,
                const std::vector<astar::Vector2D<double>> &p1,
                const std::vector<astar::Vector2D<double>> &p2,
                unsigned int, unsigned int);

        // interpolate a given path
        astar::StateArrayPtr Interpolate(astar::StateArrayPtr);

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


