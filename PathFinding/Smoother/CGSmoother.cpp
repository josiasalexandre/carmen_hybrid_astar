#include "CGSmoother.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

astar::CGSmoother::CGSmoother(astar::InternalGridMapRef map, astar::VehicleModelRef vehicle_) :
    wo(0.002), ws(4.0), wp(0.2), wk(4.0), dmax(5.0), vorodmax(20),
    alpha(0.2), grid(map), vehicle(vehicle_), kmax(0.22), input_path(nullptr),
    cg_status(astar::CGIddle), fx(), gx_norm(), fx1(), gx1_norm(), ftrialx(), x1mx_norm(), gtrialx_norm(), trialxmx_norm(), s(), s_norm(), sg(),
    locked_positions(), max_iterations(400), dim(0), start(0), end(0), step(0.01), default_step_length(1.0), stepmax(1e06), stepmin(1e-12),
    ftol(1e-04), gtol(0.99999), xtol(1e-06), stopping_points(0)
{

    // update the inverse dmax
    inverse_vorodmax2 = 1.0/ (vorodmax * vorodmax);

    // start the x and x1 vectors
    x = new astar::Vector2DArray<double>();
    x1 = new astar::Vector2DArray<double>();
    trialx = new astar::Vector2DArray<double>();

    // the gradient vectors
    gx = new astar::Vector2DArray<double>();
    gx1 = new astar::Vector2DArray<double>();
    gtrialx = new astar::Vector2DArray<double>();


}

// basic destructor
astar::CGSmoother::~CGSmoother() {

    // remove the x and x1 vectors
    delete x;
    delete x1;

    // remove the gx and gx1 gradient vectors
    delete gx;
    delete gx1;

}

// verify if a given path is safe
bool astar::CGSmoother::UnsafePath(astar::Vector2DArrayPtr<double> path) {

    // the boolean result
    bool unsafe = false;

    // direct access
    std::vector<astar::Vector2D<double>> &positions(path->vs);
    std::vector<astar::State2D> &poses(input_path->states);

    // upper index limit
    unsigned int limit = dim - 1;

    double safety = vehicle.safety_factor;

    for (unsigned int i = 1, j = start + 1; i < limit; ++i, ++j) {

        if (!locked_positions[i]) {

            if (!grid.isSafePlace(vehicle.GetVehicleBodyCircles(positions[i], poses[j].orientation), safety)) {

                // lock the current point
                locked_positions[i] = true;

                // reset the point to the A* result
                positions[i] = poses[j].position;

                // set the unsafe flag
                unsafe = false;

            }
        }

    }

    return unsafe;
}

// get the greater number considering the absolute values
double astar::CGSmoother::ABSMax(double a, double b, double c) {

    a = std::fabs(a);
    b = std::fabs(b);
    c = std::fabs(c);

    if (a > b) {

        return (a > c) ? a : c;

    } else {

        return (b > c) ? b : c;

    }

}

// the cost function to be minimized
void astar::CGSmoother::CostFunction() {

    // the partial values
    double obstacle = 0.0, potential = 0.0, curvature = 0.0, smooth = 0.0;

    // the tmp solution direct access
    std::vector<astar::Vector2D<double>> &trialxs(trialx->vs);

    unsigned int limit = dim - 1;

    // iterate from the second element till the second last
    // and get the individual derivatives
    // the first and last points should no be modified
    for (unsigned int i = 1; i < limit; ++i) {

        astar::Vector2D<double> &xim1(trialxs[i-1]), &xi(trialxs[i]), &xip1(trialxs[i+1]);

        // evaluate the current point
        EvaluateF(xim1, xi, xip1, obstacle, potential, smooth, curvature);

    }

    //fx1 = ws*smooth + wo*obstacle + wp*potential + ;
    ftrialx = wo*obstacle + wp*potential + wk*curvature + ws*smooth;

}

// get the obstacle and voronoi contribution
astar::Vector2D<double>
astar::CGSmoother::GetObstacleAndVoronoiDerivatives(const astar::Vector2D<double> &xim1, const astar::Vector2D<double> &xi, const astar::Vector2D<double> &xip1) {

    // the resultin value
    astar::Vector2D<double> res;

    // get the nearest obstacle distance
    double obst_distance = grid.GetObstacleDistance(xi);

    // the obstacle and voronoi field terms are valid when dmax is greater or equal to the nearest obstacle distance
    if (dmax > obst_distance) {

        // get the nearest obstacle position
        astar::Vector2D<double> nearest(grid.GetObstaclePosition(xi));

        // get the current vector Xi - Oi
        astar::Vector2D<double> ximoi(xi - nearest);

        // normalize the vector
        ximoi.Multiply(obst_distance);

        // the current obstacle derivative contribution
        res = ximoi;
        res.Multiply(wo * (obst_distance - dmax));

        if (obst_distance < vorodmax) {

            // get the nearest voronoi position
            nearest = grid.GetVoronoiPosition(xi);

            // get the current vector Xi - Vi
            astar::Vector2D<double> ximvi(xi - nearest);

            // get the current voro distance
            double voro_distance = ximvi.Norm();

            if (0 < voro_distance) {
                // normalize the ximvi vector
                ximvi.Multiply(voro_distance);

                // avoid a lot of divisions and repetitive floating point operations
                double alpha_over_obstacle = alpha / (alpha + obst_distance);
                double opv = obst_distance + voro_distance;
                double omd = obst_distance - dmax;

                // get the potential field derivative of the nearest voronoi edge with respect the current position
                double pvdv = alpha_over_obstacle * ((omd*omd)*inverse_vorodmax2) * (obst_distance / (opv*opv));

                // get the potential field derivative of the nearest obstacle with respect to the current position
                double pvdo = alpha_over_obstacle * (voro_distance / opv) * ((omd) * inverse_vorodmax2) *
                        (-(omd)/(alpha + obst_distance) - (omd)/opv + 2.0);

                // update the Xi -Oi and Xi - Vi vectors
                ximvi.Multiply(pvdv);
                ximoi.Multiply(pvdo);

                // add the voronoi potential field contribution
                res.x += wp * (ximvi.x + ximoi.x);
                res.y += wp * (ximvi.y + ximoi.y);

            }

        }

    }

    return res;

}

// get the obstacle and voronoi contribution
// overloaded version
astar::Vector2D<double>
astar::CGSmoother::GetObstacleAndVoronoiDerivatives(
    const astar::Vector2D<double> &xim1, const astar::Vector2D<double> &xi, const astar::Vector2D<double> &xip1,
    double nearest_obstacle_distance, double nearest_voro_distance) {

    // the resultin value
    astar::Vector2D<double> res;

    // get the nearest obstacle position
    astar::Vector2D<double> nearest(grid.GetObstaclePosition(xi));

    // get the current vector Xi - Oi
    astar::Vector2D<double> ximoi(xi - nearest);

    // normalize the vector
    ximoi.Multiply(1.0/nearest_obstacle_distance);

    // the current obstacle derivative contribution
    res = ximoi;
    res.Multiply(wo * 2.0 * (nearest_obstacle_distance - dmax));

    if (0 < nearest_voro_distance) {

        // get the nearest voronoi position
        nearest = grid.GetVoronoiPosition(xi);

        // get the current vector Xi - Vi
        astar::Vector2D<double> ximvi(xi - nearest);

        // normalize the ximvi vector
        ximvi.Multiply(1.0/nearest_voro_distance);

        // avoid a lot of divisions and repetitive floating point operations
        double alpha_over_obstacle = alpha / (alpha + nearest_obstacle_distance);
        double opv = nearest_obstacle_distance + nearest_voro_distance;
        double omd = nearest_obstacle_distance - vorodmax;

        // get the potential field derivative of the nearest voronoi edge with respect the current position
        double pvdv = alpha_over_obstacle * ((omd*omd)*inverse_vorodmax2) * (nearest_obstacle_distance / (opv*opv));

        // get the potential field derivative of the nearest obstacle with respect to the current position
        double pvdo = alpha_over_obstacle * (nearest_voro_distance / opv) * ((omd) * inverse_vorodmax2) *
                (-(omd)/(alpha + nearest_obstacle_distance) - (omd)/opv + 2.0);

        // update the Xi -Oi and Xi - Vi vectors
        ximvi.Multiply(pvdv);
        ximoi.Multiply(pvdo);

        // add the voronoi potential field contribution
        res.x += wp * (ximvi.x + ximoi.x);
        res.y += wp * (ximvi.y + ximoi.y);

    }

    return res;

}

// get the curvature contribution
astar::Vector2D<double>
astar::CGSmoother::GetCurvatureDerivative(const astar::Vector2D<double> &xim1, const astar::Vector2D<double> &xi, const astar::Vector2D<double> &xip1) {

    // the resulting vector
    astar::Vector2D<double> res;

    // get the dxi
    astar::Vector2D<double> dxi(xi.x - xim1.x, xi.y - xim1.y);
    astar::Vector2D<double> dxip1(xip1.x - xi.x, xip1.y - xi.y);

    // get the delta phi value
    double dphi = std::acos(std::max(-1.0, std::min((dxi.x * dxip1.x + dxi.y * dxip1.y) / (dxi.Norm() * dxip1.Norm()), 1.0)));

    // get the curvature
    double k = dphi / dxi.Norm();

    // if the curuvature k is greater than kmax then we need to add the curvature contribution
    // otherwise we set the curvature term equals to zero
    if (kmax < k) {

        // get the derivative of delta phi with respect the cosine of delta phi
        double ddphi = -1.0/std::sqrt(1.0 - std::pow(std::cos(dphi), 2));

        // the common denominator
        double inverse_denom = 1.0/(xi.Norm() * xip1.Norm());

        // the normalized orthogonal complements
        astar::Vector2D<double> p1, p2;

        // some helpers
        double inverse_norm2 = 1.0/xip1.Norm2();
        double nx = -xip1.x;
        double ny = -xip1.y;
        double tmp = (xi.x * nx + xi.y * ny);
        double tmp1 = tmp * inverse_norm2;

        // set the first othogonal complement
        p1.x = (xi.x - nx * tmp1) * inverse_denom;
        p1.y = (xi.y - ny * tmp1) * inverse_denom;

        // reset the helpers
        inverse_norm2 = 1.0/xi.Norm();

        tmp1 = tmp * inverse_norm2;

        // set the second othogonal complement
        p2.x = (nx - xi.x * tmp1) * inverse_denom;
        p2.y = (ny - xi.y * tmp1) * inverse_denom;

        // get common term in all three points
        double coeff1 = (-1.0/dxi.Norm()) * ddphi;
        double coeff2 = dphi / dxi.Norm2();

        // reuse the k variable to get the first part of the derivative
        k = 2 * (k - kmax);

        astar::Vector2D<double> ki, kim1, kip1;
        ki = (p1 + p2);
        ki.Multiply(-coeff1);
        ki.Subtract(coeff2);
        // apply the factor
        ki.Multiply(0.5 * k);

        kim1 = p2;
        kim1.Multiply(coeff1);
        kim1.Add(coeff2);
        // aply the factor
        kim1.Multiply(0.25 * k);

        kip1 = p1;
        // aply the factor
        kip1.Multiply(coeff1 * 0.25 * k);

        // add the curvature contribution
        res.x += wk*(kim1.x + ki.x + kip1.x);
        res.y += wk*(kim1.y + ki.y + kip1.y);

    }

    return res;

}

// get the smootheness contribution
astar::Vector2D<double> astar::CGSmoother::GetSmoothPathDerivative(
        const astar::Vector2D<double> &xim2,
        const astar::Vector2D<double> &xim1,
        const astar::Vector2D<double> &xi,
        const astar::Vector2D<double> &xip1,
        const astar::Vector2D<double> &xip2)
{

    // the resulting vector
    astar::Vector2D<double> res;

    res.x = xim2.x - 4.0 * xim1.x + 6.0 * xi.x - 4.0 * xip1.x + xip2.x;
    res.y = xim2.y - 4.0 * xim1.y + 6.0 * xi.y - 4.0 * xip1.y + xip2.y;

    return res;

}

// build the cost function gradient evaluated at a given input path and returns the norm of the gradient
void astar::CGSmoother::ComputeGradient() {

    // get the third last limit
    unsigned int limit = dim - 2;

    // reset the gx1 gradient norm
    gtrialx_norm = 0.0;

    // the tmp solution direct access
    std::vector<astar::Vector2D<double>> &trialxs(trialx->vs);

    // the gradient direct access
    std::vector<astar::Vector2D<double>> &gradient(gtrialx->vs);

    // reset the first gradient value
    gradient[1].x = 0.0;
    gradient[1].y = 0.0;

    if (!locked_positions[1]) {

        astar::Vector2D<double> &xim1(trialxs[0]), &xi(trialxs[1]), &xip1(trialxs[2]), &xip2(trialxs[3]);

        // evaluate the second point
        // the
        EvaluateG(xim1, xi, xip1, gradient[1]);

        // the boundary
        // set up the smooth path derivatives contribution
        gradient[1].x += ws * (-2.0 * xim1.x + 5.0 * xi.x - 4.0 * xip1.x + xip2.x);
        gradient[1].y += ws * (-2.0 * xim1.y + 5.0 * xi.y - 4.0 * xip1.y + xip2.y);

        // update the gradient norm
        gtrialx_norm += gradient[1].Norm2();

    }

    unsigned int i;

    // iterate from the third element till the third last
    // and get the individual derivatives
    // the first and last points should no be modified
    for (i = 2; i < limit; ++i) {

        // reset the current gradient value
        gradient[i].x = 0.0;
        gradient[i].y = 0.0;

        if (!locked_positions[i]) {

            // get the position references
            astar::Vector2D<double> &xim2(trialxs[i-2]), &xim1(trialxs[i-1]),
                    &xi(trialxs[i]), &xip1(trialxs[i+1]), &xip2(trialxs[i+2]);

            // evaluate the current point/
            EvaluateG(xim1, xi, xip1, gradient[i]);

            // the custom smooth formula
            // set up the smooth path derivatives contribution
            gradient[i].x += ws * (xim2.x - 4.0 * xim1.x + 6.0 * xi.x - 4.0 * xip1.x + xip2.x);
            gradient[i].y += ws * (xim2.y - 4.0 * xim1.y + 6.0 * xi.y - 4.0 * xip1.y + xip2.y);

            // update the gradient norm
            gtrialx_norm += gradient[i].Norm2();

        }

    }

    // reset the current gradient value
    gradient[i].x = 0.0;
    gradient[i].y = 0.0;

    if (!locked_positions[i]) {

        astar::Vector2D<double> &xim2(trialxs[i-2]), &xim1(trialxs[i-1]), &xi(trialxs[i]), &xip1(trialxs[i+1]);

        // evaluate the second point
        // the
        EvaluateG(xim1, xi, xip1, gradient[i]);

        // set up the smooth path derivatives contribution
        gradient[i].x += ws * (xim2.x - 4.0 * xim1.x + 5.0 * xi.x - 2.0 * xip1.x);
        gradient[i].y += ws * (xim2.y - 4.0 * xim1.y + 5.0 * xi.y - 2.0 * xip1.y);

        // update the gradient norm
        gtrialx_norm += gradient[i].Norm2();

    }

    // euclidean norm
    gtrialx_norm = std::sqrt(gtrialx_norm);

}

// take a step at the current direction vector (s)
void astar::CGSmoother::TakeStep(double factor) {

    std::vector<astar::Vector2D<double>> &current(x->vs);
    std::vector<astar::Vector2D<double>> &next(trialx->vs);

    // get the limit
    unsigned int limit = dim - 2;

    // reset the dx norm
    trialxmx_norm = 0.0;

    // update the current position
    for (unsigned int i = 2; i < limit; ++i) {

        if (!locked_positions[i]) {

            // the factor value is equal to step/ s_norm
            // the s vector is not normalized, so ...
            next[i].x = current[i].x + factor * s[i].x;
            next[i].y = current[i].y + factor * s[i].y;

            trialxmx_norm += std::pow(next[i].x - current[i].x, 2) + std::pow(next[i].y - current[i].y, 2);

        }

    }

    // euclidean norm
    trialxmx_norm = std::sqrt(trialxmx_norm);

}

// process the current points
void astar::CGSmoother::EvaluateF(
    const astar::Vector2D<double> &xim1,
    const astar::Vector2D<double> &xi,
    const astar::Vector2D<double> &xip1,
    double &obstacle,
    double &potential,
    double &smooth,
    double &curvature) {

    //
    astar::Vector2D<double> dxi, dxip1, tmp;

    // get the nearest obstacle distance
    double obst_distance = grid.GetObstacleDistance(xi);

    if (dmax >= obst_distance) {

        double d = (dmax - obst_distance)*(dmax - obst_distance);

        // updatate the obstacle term
        obstacle += d;

        // get the nearest voronoi edge distance
        double voro_distance = grid.GetVoronoiDistance(xi);

        // update the Voronoi potential field term
        // can it become a Nan?
        //potential += grid.GetPathCost(xi);
        potential += (alpha / (alpha  + obst_distance)) * (voro_distance / (obst_distance  + voro_distance)) * ((d - vorodmax*vorodmax)* inverse_vorodmax2);

    }

    // get the current displacement
    dxi.x = xi.x - xim1.x;
    dxi.y = xi.y - xim1.y;

    // get the next displacement
    dxip1.x = xip1.x - xi.x;
    dxip1.y = xip1.y - xi.y;

    // update the curvature term
    curvature += std::pow((std::fabs(std::atan2(dxip1.y, dxip1.x) - std::atan2(dxi.y, dxi.x)) / dxi.Norm() - kmax), 2);

    // update the smooth term
    smooth += std::pow(xip1.x - 2.0 * xi.x + xim1.x, 2) + std::pow(xip1.y - 2.0 * xi.y + xim1.y, 2);

}

// process the current points
void astar::CGSmoother::EvaluateG(
    const astar::Vector2D<double> &xim1,
    const astar::Vector2D<double> &xi,
    const astar::Vector2D<double> &xip1,
    astar::Vector2D<double> &gradient
    ) {

    //
    astar::Vector2D<double> dxi, dxip1;

    // get the nearest obstacle distance
    double obst_distance = grid.GetObstacleDistance(xi);

    if (dmax >= obst_distance) {

        // get the nearest voronoi edge distance
        double voro_distance = grid.GetVoronoiDistance(xi);

        // add the obstacle and the voronoi contributions
        // overloaded method
        // is the current pose a stop one?
        gradient.Add(GetObstacleAndVoronoiDerivatives(xim1, xi, xip1, obst_distance, voro_distance));

    }

    // add the curvature term contribution
    gradient.Add(GetCurvatureDerivative(xim1, xi, xip1));

}

// process the current points
void astar::CGSmoother::EvaluateFG(
    const astar::Vector2D<double> &xim1,
    const astar::Vector2D<double> &xi,
    const astar::Vector2D<double> &xip1,
    double &obstacle,
    double &potential,
    double &smooth,
    double &curvature,
    astar::Vector2D<double> &gradient) {

    //
    astar::Vector2D<double> dxi, dxip1;

    // get the nearest obstacle distance
    double obst_distance = grid.GetObstacleDistance(xi);

    if (dmax >= obst_distance) {

        double d = (dmax - obst_distance)*(dmax - obst_distance);

        // updatate the obstacle term
        obstacle += d;

        // get the nearest voronoi edge distance
        double voro_distance = grid.GetVoronoiDistance(xi);

        // update the Voronoi potential field term
        // can it become a Nan?
        //potential += grid.GetPathCost(xi);
        potential += (alpha / (alpha  + obst_distance)) * (voro_distance / (obst_distance  + voro_distance)) * ((d - vorodmax*vorodmax)* inverse_vorodmax2);

        // add the obstacle and the voronoi contributions
        // overloaded method
        // is the current pose a stop one?
        gradient.Add(GetObstacleAndVoronoiDerivatives(xim1, xi, xip1, obst_distance, voro_distance));

    }

    // get the current displacement
    dxi.x = xi.x - xim1.x;
    dxi.y = xi.y - xim1.y;

    // get the next displacement
    dxip1.x = xip1.x - xi.x;
    dxip1.y = xip1.y - xi.y;

    // update the curvature term
    curvature += std::pow((std::fabs(std::atan2(dxip1.y, dxip1.x) - std::atan2(dxi.y, dxi.x)) / dxi.Norm() - kmax), 2);

    // update the smooth term
    smooth += std::pow(xip1.x - 2.0 * xi.x + xim1.x, 2) + std::pow(xip1.y - 2.0 * xi.y + xim1.y, 2);

    // add the curvature term contribution
    gradient.Add(GetCurvatureDerivative(xim1, xi, xip1));

}

// custom method to evaluate the cost function and the gradient at the same time
// it uses the trialx as the input array
// the resulting cost value is saved in the internal ftrialx variable
// the gradient vector gtrialx is updated and the it's norm is saved in the gtrialx_norm variable
void astar::CGSmoother::EvaluateFunctionAndGradient() {

    // the partial values
    double obstacle = 0.0, potential = 0.0, curvature = 0.0, smooth = 0.0;

    // get the third last limit
    unsigned int limit = dim - 2;

    // reset the gx1 gradient norm
    gtrialx_norm = 0.0;

    // the tmp solution direct access
    std::vector<astar::Vector2D<double>> &trialxs(trialx->vs);

    // the gradient direct access
    std::vector<astar::Vector2D<double>> &gradient(gtrialx->vs);

    unsigned int i;

    // iterate from the third element till the third last
    // and get the individual derivatives
    // the first and last points should no be modified
    for (i = 2; i < limit; ++i) {

        // reset the current gradient value
        gradient[i].x = 0.0;
        gradient[i].y = 0.0;

        // get the position references
        astar::Vector2D<double> &xim2(trialxs[i-2]), &xim1(trialxs[i-1]), &xi(trialxs[i]), &xip1(trialxs[i+1]), &xip2(trialxs[i+2]);

        if (locked_positions[i]) {

            // evaluate the current point
            EvaluateF(xim1, xi, xip1, obstacle, potential, smooth, curvature);

        } else {

            // evaluate the current point
            EvaluateFG(xim1, xi, xip1, obstacle, potential, smooth, curvature, gradient[i]);

            // the custom smooth formula
            // set up the smooth path derivatives contribution
            gradient[i].x += ws * (xim2.x - 4.0 * xim1.x + 6.0 * xi.x - 4.0 * xip1.x + xip2.x);
            gradient[i].y += ws * (xim2.y - 4.0 * xim1.y + 6.0 * xi.y - 4.0 * xip1.y + xip2.y);

            // update the gradient norm
            gtrialx_norm += gradient[i].Norm2();

        }

    }

    // set the euclidean norm final touch
    gtrialx_norm = std::sqrt(gtrialx_norm);

    //fx1 = ws*smooth + wo*obstacle + wp*potential + ;
    ftrialx = wo*obstacle + wp*potential + wk*curvature + ws*smooth;

}

// the Moré-Thuente step trial
// The purpose of cstep is to compute a safeguarded step for
// a linesearch and to update an interval of uncertainty for
// a minimizer of the function.
// It's the cstep function provided by the minpack library and it is adapted to our context here
//  Argonne National Laboratory. MINPACK Project. June 1983
// Jorge J. More', David J. Thuente
int astar::CGSmoother::CStep(
        double& stl, double& fx, double& sgl,
        double& stu, double& fu, double& sgu,
        double& stp, double& fp, double& sgp,
        bool& bracket, double stmin, double stmax) {


    int info = 0;

    // Check the input parameters for errors.
    if ((bracket && ((stp <= std::min(stl, stu)) || (stp >= std::max(stl, stu)))) || (sgl * (stp - stl) >= 0.0) || (stmax < stmin))
        return info;

    // Determine if the derivatives have opposite sign.
    double sgnd = sgp * (sgl / fabs(sgl));

    // First case. A higher function value.  The minimum is
    // bracketed. If the cubic step is closer to stl than the quadratic
    // step, the cubic step is taken, else the average of the cubic and
    // quadratic steps is taken.

    bool bound;
    double theta;
    double s;
    double gamma;
    double p,q,r;
    double stpc, stpq, stpf;

    if (fp > fx) {

        info = 1;
        bound = 1;
        theta = 3 * (fx - fp) / (stp - stl) + sgl + sgp;
        s = ABSMax(theta, sgl, sgp);
        gamma = s * sqrt(((theta / s) * (theta / s)) - (sgl / s) * (sgp / s));

        if (stp < stl) {

            gamma = -gamma;

        }

        p = (gamma - sgl) + theta;
        q = ((gamma - sgl) + gamma) + sgp;
        r = p / q;
        stpc = stl + r * (stp - stl);
        stpq = stl + ((sgl / ((fx - fp) / (stp - stl) + sgl)) / 2) * (stp - stl);

        if (fabs(stpc - stl) < fabs(stpq - stl)) {

            stpf = stpc;

        } else {

            stpf = stpc + (stpq - stpc) / 2;

        }

        bracket = true;

    } else if (sgnd < 0.0) {

        // Second case. A lower function value and derivatives of opposite
        // sign. The minimum is bracketed. If the cubic step is closer to
        // stl than the quadratic (secant) step, the cubic step is taken,
        // else the quadratic step is taken.

        info = 2;
        bound = false;
        theta = 3 * (fx - fp) / (stp - stl) + sgl + sgp;
        s = ABSMax(theta,sgl,sgp);
        gamma = s * sqrt(((theta/s) * (theta/s)) - (sgl / s) * (sgp / s));

        if (stp > stl) {

            gamma = -gamma;

        }

        p = (gamma - sgp) + theta;
        q = ((gamma - sgp) + gamma) + sgl;
        r = p / q;
        stpc = stp + r * (stl - stp);
        stpq = stp + (sgp / (sgp - sgl)) * (stl - stp);

        if (fabs(stpc - stp) > fabs(stpq - stp)) {

            stpf = stpc;

        } else {

            stpf = stpq;

        }

        bracket = true;

    } else if (fabs(sgp) < fabs(sgl)) {

        // Third case. A lower function value, derivatives of the same sign,
        // and the magnitude of the derivative decreases.  The cubic step is
        // only used if the cubic tends to infinity in the direction of the
        // step or if the minimum of the cubic is beyond stp. Otherwise the
        // cubic step is defined to be either stmin or stmax. The
        // quadratic (secant) step is also computed and if the minimum is
        // bracketed then the the step closest to stl is taken, else the
        // step farthest away is taken.

        info = 3;
        bound = true;
        theta = 3 * (fx - fp) / (stp - stl) + sgl + sgp;
        s = ABSMax(theta, sgl, sgp);

        // The case gamma = 0 only arises if the cubic does not tend
        // to infinity in the direction of the step.

        gamma = s * sqrt(std::max(0.0, (theta / s) * (theta / s) - (sgl / s) * (sgp / s)));
        if (stp > stl)
            gamma = -gamma;

        p = (gamma - sgp) + theta;
        q = (gamma + (sgl - sgp)) + gamma;
        r = p / q;

        if ((r < 0.0) && (gamma != 0.0)) {

            stpc = stp + r * (stl - stp);

        } else if (stp > stl) {

            stpc = stmax;

        } else {

            stpc = stmin;

        }

        stpq = stp + (sgp/ (sgp - sgl)) * (stl - stp);

        if (bracket) {

            if (fabs(stp - stpc) < fabs(stp - stpq)) {

                stpf = stpc;

            } else {

                stpf = stpq;

            }

        } else {

            if (fabs(stp - stpc) > fabs(stp - stpq)) {

                stpf = stpc;

            } else {

                stpf = stpq;

            }

        }

    } else {

        // Fourth case. A lower function value, derivatives of the same
        // sign, and the magnitude of the derivative does not decrease. If
        // the minimum is not bracketed, the step is either stmin or
        // stmax, else the cubic step is taken.

        info = 4;
        bound = false;

        if (bracket) {
            theta = 3 * (fp - fu) / (stu - stp) + sgu + sgp;
            s = ABSMax(theta, sgu, sgp);
            gamma = s * sqrt(((theta/s)*(theta/s)) - (sgu / s) * (sgp / s));

            if (stp > stu) {

                gamma = -gamma;

            }

            p = (gamma - sgp) + theta;
            q = ((gamma - sgp) + gamma) + sgu;
            r = p / q;
            stpc = stp + r * (stu - stp);
            stpf = stpc;

        }

        else if (stp > stl) {

            stpf = stmax;

        } else {

            stpf = stmin;

        }

    }

    // Update the interval of uncertainty. This update does not depend
    // on the new step or the case analysis above.

    if (fp > fx) {

        stu = stp;
        fu = fp;
        sgu = sgp;

    } else {

        if (sgnd < 0.0) {

            stu = stl;
            fu = fx;
            sgu = sgl;

        }

        stl = stp;
        fx = fp;
        sgl = sgp;

    }

    // Compute the new step and safeguard it.

    stpf = std::min(stmax, stpf);
    stpf = std::max(stmin, stpf);
    stp = stpf;

    if (bracket && bound) {

        if (stu > stl) {

            stp = std::min(stl + 0.66 * (stu - stl), stp);

        } else {

            stp = std::max(stl + 0.66 * (stu - stl), stp);

        }

    }

    return info;

}

// the Moré-Thuente line serch
// based on the minpack and derivates codes
int astar::CGSmoother::MTLineSearch(double lambda) {

    // verify the direction
    if (0 <= sg) {

        std::cout << "Not a descent direction!\n";

        return -2;

    }

    // Initialize local variables
    int info = 0;
    int infoc = 1;
    bool bracket = false;
    bool stage1 = true;
    double sgtest = ftol * sg;
    double width = stepmax - stepmin;
    double width1 = 2.0 * width;
    double maxfev = 20;
    int nfev = 0;

    // initial function value
    double finit = fx;

    // the lower step length limit
    double stl = 0.0;
    double fl = finit;
    double sgl = sg;

    // the upper step length limit
    double stu = 0.0;
    double fu = finit;
    double sgu = sg;

    // the current step
    double stp = default_step_length;
    double fp;
    double sgp;

    // get the linear solve tolerance
    // double eta = -1.0;
    double stmin, stmax;

    // the modified function, see PSI function in More-Thuente
    double fpm, flm, fum, sgpm, sglm, sgum;

    // start of iteration
    while (true) {

        if (bracket) {

            stmin = std::min(stl, stu);
            stmax = std::max(stl, stu);

        } else {

            stmin = stl;
            stmax = stp + 4.0 * (stp - stl);

        }

        // clamp the step
        if (stp < stepmin) {

            stp = stepmin;

        } else if (stp > stepmax) {

            stp = stepmax;

        }

        // If an unusual termination is to occur then let stp be the
        // lowest point obtained so far
        if ((bracket  && ((stp <= stmin) || (stp >= stmax))) || (nfev >= maxfev  - 1) || (infoc == 0) ||
                (bracket && (stmax - stmin <= xtol * stmax))) {

            stp = stl;

        }

        // Evaluate the function and the gradient at the current stp
        // move trialx to the new position
        TakeStep(stp * lambda);

        // evaluate the function at the new point
        // CostFunction();
        // ComputeGradient();
        EvaluateFunctionAndGradient();

        // get the directional derivative
        sgp = astar::Vector2DArray<double>::DotProduct(s, gtrialx->vs) * lambda;

        fp = ftrialx;

        if (fx1 > ftrialx) {

            // update the best solution so far
            // flip x1 and bestx
            astar::Vector2DArrayPtr<double> tmp = trialx;
            trialx = x1;
            x1 = tmp;

            // flip the gradient vectors
            tmp = gtrialx;
            gtrialx = gx1;
            gx1 = tmp;

            // copy the function value
            fx1 = ftrialx;

            // copy the norm of gradient
            gx1 = gtrialx;

            // set the norm of the displacement vector between the old and the new path
            x1mx_norm = trialxmx_norm;

        }

        // Armijo-Goldstein sufficient decrease
        double ftest = finit + stp * sgtest;

        // Ared/Pred suffiecient decrease (could use a user defined norm)
        // double ftest2 = gx_norm * (1.0 - ftol * (1.0 - eta));

        // Convergence tests

        // Rounding errors
        if ((bracket && ((stp <= stmin) || (stp >= stmax))) || (infoc == 0)) {

            info = 6;

        }

        // stp = stepmax
        if ((stp == stmax) && (fp <= ftest) && (sgp <= sgtest)) {

            info = 5;

        }

        // stp = stpmin
        if ((stp == stepmin) && ((fp > ftest) || (sgp >= sgtest))) {

            info = 4;

        }

        // max'd out on fevals
        if (nfev >= maxfev) {

            info = 3;

        }

        // bracketed soln
        if (bracket && (stmax - stmin <= xtol * stmax)) {

            info = 2;

        }

        // Armijo-Goldstein test
        if (fp <= ftest && std::fabs(sgp) <= gtol * (-sg)) {

            // success!
            info = 1;

        }

        if (0 != info) {

            // line search is done!

            if (1 != info) {

                // wrong!!!
                // set the default step
                stp = default_step_length;

            }

            // set the step
            step = stp;

            // exit
            return info;

        }

        // In the first stage we seek a step for which the modified
        // function has a nonpositive value and nonnegative derivative.
        if (stage1 && (fp <= ftest) && (sgp >= std::min(ftol, gtol) * sg)) {

            stage1 = false;

        }

        if (stage1 && (fp <= fl) && (fp > ftest))
        {

            // Define the modified function and derivative values.
            fpm = fp - stp * sgtest;
            sgpm = sgp - sgtest;

            flm = fl - stl * sgtest;
            sglm = sgl - sgtest;

            fum = fu - stu * sgtest;
            sgum = sgu - sgtest;

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.
            infoc = CStep(stl, flm, sglm, stu, fum, sgum, stp, fpm, sgpm, bracket, stmin, stmax);

            // Reset the function and gradient values for f.
            fl = flm + stl * sgtest;
            fu = fum + stu * sgtest;
            sgl = sglm + sgtest;
            sgu = sgum + sgtest;

        } else {

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.
            infoc = CStep(stl, fl, sgl, stu, fu, sgu, stp, fp, sgp, bracket, stmin, stmax);

        }

        // Force a sufficient decrease in the size of the
        // interval of uncertainty.
        if (bracket)
        {

            if (std::fabs(stu - stl) >= 0.66 * width1) {

                stp = stl + 0.5 * (stu - stl);

            }

            width1 = width;
            width = std::fabs(stu - stl);

        }

    }

}

// configure the stopping points and break the path into subpaths
void astar::CGSmoother::BreakPath(astar::StateArrayPtr raw_path) {

    // states direct access
    std::vector<astar::State2D> &states(raw_path->states);

    // get the path dimension
    unsigned int size = states.size();

    // reset the stoppig points vector
    stopping_points.clear();

    unsigned int i = 1;

    // the first part is the A* simple expanded nodes
    while (i < size) {

        if (states[i-1].gear != states[i].gear) {

            // set the coming to stop flag
            states[i-1].coming_to_stop = true;

            // set the stopping point index
            stopping_points.push_back(i);

        }

        // update the i counter
        i += 1;

    }

    // add the last stop point
    stopping_points.push_back(size - 1);

}

// setup the first iteration and configure the containers
bool astar::CGSmoother::Setup(astar::StateArrayPtr path, bool locked) {

    // get the reference to the input path
    input_path = path;

    // set the kmax value
    kmax = 1.0/vehicle.min_turn_radius;

    // get the input path size and store the problem's dimension
    // dim = path->states.size();
    dim = (end - start) + 1;

    // tmp vector
    astar::Vector2D<double> tmp(0.0, 0.0);

    // resize the solution vectors
    x->vs.resize(dim, tmp);
    x1->vs.resize(dim, tmp);
    trialx->vs.resize(dim, tmp);

    // direct access
    std::vector<astar::State2D> &states(path->states);
    std::vector<astar::Vector2D<double>> &xs(x->vs);
    std::vector<astar::Vector2D<double>> &x1s(x1->vs);
    std::vector<astar::Vector2D<double>> &trialxs(trialx->vs);

    // copy the input positions
    // and lock the stopping points
    for (unsigned int i = 0, j = start; i < dim; ++i, ++j) {

        xs[i] = states[j].position;
        x1s[i] = xs[i];
        trialxs[i] = xs[i];

    }

    if (!locked) {

        // resize the locked positions vector
        locked_positions.resize(dim, false);

        // the last valid index
        unsigned int limit = dim - 1;

        for (unsigned int i = 1, j = start + 1; i < limit; ++i, ++j) {

            locked_positions[i] = states[j - 1].gear != states[j].gear;

        }

        // lock the first and last positions
        locked_positions[0] = locked_positions[limit] = true;

    }

    // update the dx norm
    trialxmx_norm = x1mx_norm = std::numeric_limits<double>::max();

    // reset the gradient at x
    gx->vs.resize(dim, tmp);

    // reset the gradient at x1
    gx1->vs.resize(dim, tmp);

    // reset the gradient displacement
    gx1mgx.resize(dim, tmp);

    // resize the trial position gradient
    gtrialx->vs.resize(dim, tmp);

    // evaluate the function and the gradient at the same time
    EvaluateFunctionAndGradient();

    // minima?
    if (0.0 == gtrialx_norm) {

        // set the status to success
        cg_status = astar::CGSuccess;

        // already a solution, local minima
        return false;

    }

    // reset the direction vector
    s.resize(dim, tmp);
    for (unsigned int i = 0; i < dim; ++i) {

        s[i].x = -gtrialx->vs[i].x;
        s[i].y = -gtrialx->vs[i].y;

    }

    // multiply the current direction with the inverse gradient
    // it's the directional derivative along s
    sg = astar::Vector2DArray<double>::DotProduct(s, gtrialx->vs) / gtrialx_norm;

    fx = fx1 = ftrialx;
    gx_norm = gx1_norm = s_norm = gtrialx_norm;

    // flip the gradients
    astar::Vector2DArrayPtr<double> vs = gtrialx;
    gtrialx = gx;
    gx = vs;

    // reset the tolerances
    ftol = xtol = 1e-04;
    gtol = 0.001;

    // set the status to continue
    cg_status = astar::CGContinue;

    return true;

}

// update the conjugate direction -> s(i+1) = -gradient + gamma * s(i)
void astar::CGSmoother::UpdateConjugateDirection(std::vector<astar::Vector2D<double>> &s, const std::vector<astar::Vector2D<double>> &gradient, double gamma) {

    // reset the current s_norm
    s_norm = 0.0;

    // reset the current slope
    sg = 0.0;

    // the limit index
    unsigned int limit = dim - 1;

    // assuming the dimension is valid
    for (unsigned int i = 1; i < limit; ++i) {

        s[i].x = gamma*s[i].x - gradient[i].x;
        s[i].y = gamma*s[i].y - gradient[i].y;

        s_norm += s[i].Norm2();

        sg += s[i].x * gradient[i].x + s[i].y * gradient[i].y;

    }

    // euclidean norm
    s_norm = std::sqrt(s_norm);

    sg /= s_norm;
}

// the main iteration loop
void astar::CGSmoother::Iterate() {

    // optimize
    // the main conjugate gradient process
    do {

        // the direction is up or down?
        double direction;
        double inverse_snorm;
        double betha = 0;

        unsigned int iter = 0;

        // the main CG Loop
        while (astar::CGContinue == cg_status && iter < max_iterations) {

            if (xtol > x1mx_norm) {

                // test the stuck case

                // set the stuck case!
                cg_status = astar::CGStuck;

                // set
                break;

            } else if (gtol > gx_norm) {

                // success!
                cg_status = astar::CGSuccess;

                // exit
                break;

            } else if (ftol > fx) {

                // success
                cg_status = astar::CGSuccess;

                // exit
                break;

            } else {

                // update the iterator counter
                iter += 1;

                // get the next step
                int info = MTLineSearch(1.0/s_norm);

                // SUCCESS!
                // verify the restart case
                if (0 != iter % dim) {

                    // get the displacement between the two gradients
                    astar::Vector2DArray<double>::SubtractCAB(gx1mgx, gx1->vs, gx->vs);

                    // get the betha value
                    // based on Powell 1983
                    betha = std::max(astar::Vector2DArray<double>::DotProduct(gx1->vs, gx1mgx)/astar::Vector2DArray<double>::DotProduct(gx->vs, gx->vs), 0.0);

                } else {

                    // set gamma to zero, it restarts the conjugate gradient
                    betha = 0.0;

                }

                // the new x1 point is the new position
                // update the conjugate direction at the new position
                // it also updates the the curent conjugate direction norm
                UpdateConjugateDirection(s, gx1->vs, betha);

                // the new position is a better one
                astar::Vector2DArrayPtr<double> tmp = x;
                x = x1;
                x1 = tmp;

                // flip the gradient vectors
                astar::Vector2DArrayPtr<double> gradient = gx1;
                gx1 = gx;
                gx = gradient;

                // copy the norm
                gx_norm = gx1_norm;

            }

        }

    } while (UnsafePath(x));

    // copy the resulting path back
    InputPathUpdate(x, input_path);

}

// the Polak-Ribiere Conjugate Gradient Method With More-Thuente Line Search
void astar::CGSmoother::ConjugateGradientPR(astar::StateArrayPtr path, bool locked) {

    // break the subpaths
    BreakPath(path);

    // how many pieces
    unsigned int pieces = stopping_points.size();

    // reset the start limit
    start = 0;

    // iterate over the subpaths
    for (unsigned int i = 0; i < pieces; ++i) {

        // set the upper limit index
        end = stopping_points[i];

        // first step, setup the optimization process
        // the gradient is the first direction
        if (!Setup(path, locked)) {

            // could not start the minimizer
            // we should make shure the old process is finished
            std::cout << "Could not start the optimization process, Maybe it's a local minima!!\n";

            return;

        }

        // the main conjugate gradient
        Iterate();

        // update the start
        start = end;

    }

    // unset the status
    cg_status = astar::CGIddle;

}

// copy the current solution to the input path
void astar::CGSmoother::InputPathUpdate(astar::Vector2DArrayPtr<double> solution, astar::StateArrayPtr output) {

    // direct access
    std::vector<astar::State2D> &states(output->states);
    std::vector<astar::Vector2D<double>> &xs(solution->vs);

    for (unsigned int i = 0, j = start; i < dim; ++i, ++j) {

        // set the new position
        states[j].position.x = xs[i].x;
        states[j].position.y = xs[i].y;

    }

}

// show the current path in the map
void astar::CGSmoother::ShowPath(astar::StateArrayPtr path, bool plot_locked) {

    // get the map
    unsigned int h = grid.GetHeight();
    unsigned int w = grid.GetWidth();

    unsigned char *map = grid.GetGridMap();

    // create a image
    cv::Mat image(w, h, CV_8UC1, map);

    // draw a new window
    cv::namedWindow("Smooth", cv::WINDOW_AUTOSIZE);

    // draw each point
    for (unsigned int i = 0; i < path->states.size(); ++i) {

        if (!plot_locked && locked_positions[i]) {

            continue;

        }

        // get the current point
        astar::GridCellIndex index(grid.PoseToIndex(path->states[i].position));

        // convert to opencv point
        cv::Point p1(index.col - 1, h - index.row - 1);
        cv::Point p2(index.col + 1, h - index.row + 1);

        cv::rectangle(image, p1, p2, cv::Scalar(0, 0, 0), 1);

        // show the image
        cv::imshow("Smooth", image);

        // draw in the windows
        cv::waitKey(30);

    }

    delete [] map;

    // destroy
    cv::destroyWindow("Smooth");

}

// get a bezier point given four points and the time
// using cubic bezier interpolation
astar::Vector2D<double> astar::CGSmoother::GetBezierPoint(std::vector<astar::Vector2D<double>> &points, double t) {

    for (unsigned int index = 3; 0 < index; --index) {

        for (unsigned int i = 0; i < index; ++i) {

            points[i].x = (1-t) * points[i].x + t * points[i+1].x;
            points[i].y = (1-t) * points[i].y + t * points[i+1].y;

        }

    }

    return points[0];

}

// build a bezier curve passing through a set of states
void astar::CGSmoother::BuildBezierControlPoints(
    const std::vector<astar::State2D> &input,
    std::vector<astar::Vector2D<double>> &p1,
    std::vector<astar::Vector2D<double>> &p2,
    unsigned int start,
    unsigned int end) {

    // the dimension
    unsigned int n = (end - start);

    // the internal limit index
    unsigned int internal_limit = n - 1;

    // helpers
    unsigned int im1, ip1;

    astar::Vector2D<double> tmp(0.0, 0.0);

    // resize the control points
    p1.resize(n, tmp);
    p2.resize(n, tmp);

    // the matriz diagonals
    std::vector<astar::Vector2D<double>> a(n);
    std::vector<astar::Vector2D<double>> b(n);
    std::vector<astar::Vector2D<double>> c(n);

    // the right hand side vector
    std::vector<astar::Vector2D<double>> rhs(n);

    // left most segment
    a[0].x = 0.0;
    a[0].y = 0.0;

    b[0].x = 2.0;
    b[0].y = 2.0;

    c[0].x = 1.0;
    c[0].y = 1.0;

    rhs[0].x = input[start].position.x  + 2.0 * input[start + 1].position.x;
    rhs[0].y = input[start].position.y  + 2.0 * input[start + 1].position.y;

    // internal segments
    for (unsigned int i = 1, j = start + 1; i < internal_limit; ++i, ++j) {

        a[i].x = 1.0;
        a[i].y = 1.0;

        b[i].x = 4.0;
        b[i].y = 4.0;

        c[i].x = 1.0;
        c[i].y = 1.0;

        rhs[i].x = 4.0 * input[j].position.x + 2.0 * input[j+1].position.x;
        rhs[i].y = 4.0 * input[j].position.y + 2.0 * input[j+1].position.y;

    }

    // right most segment
    a[internal_limit].x = 2.0;
    a[internal_limit].y = 2.0;

    b[internal_limit].x = 7.0;
    b[internal_limit].y = 7.0;

    c[internal_limit].x = 0.0;
    c[internal_limit].y = 0.0;

    rhs[internal_limit].x = 8.0 * input[end - 1].position.x + input[end].position.x;
    rhs[internal_limit].y = 8.0 * input[end - 1].position.y + input[end].position.y;

    // now we have a Ax = b system
    // It's a tridiagonal system, let's use the Thomas algorithm (from Wikipedia)
    for (unsigned int i = 1; i < n; ++i) {

        im1 = i - 1;

        double mx = a[i].x/b[im1].x;
        double my = a[i].y/b[im1].y;

        b[i].x = b[i].x - mx * c[im1].x;
        b[i].y = b[i].y - my * c[im1].y;

        rhs[i].x = rhs[i].x - mx*rhs[im1].x;
        rhs[i].y = rhs[i].y - my*rhs[im1].y;
    }

    // set the last p1 point value
    p1[internal_limit].x = rhs[internal_limit].x/b[internal_limit].x;
    p1[internal_limit].y = rhs[internal_limit].y/b[internal_limit].y;

    // backward substitution
    for (int i = ((int) n) - 2; i >= 0; --i) {

        ip1 = i+1;

        p1[i].x = (rhs[i].x - c[i].x * p1[ip1].x) / b[i].x;
        p1[i].y = (rhs[i].y - c[i].y * p1[ip1].y) / b[i].y;

    }

    // we have the left control points, now compute the right ones
    for (unsigned int i = 0, j = start + 1; i < internal_limit; ++i, ++j) {

        ip1 = i+1;

        p2[i].x = 2.0 * input[j].position.x - p1[ip1].x;
        p2[i].y = 2.0 * input[j].position.y - p1[ip1].y;

    }

    // update the last one
    p2[internal_limit].x = 0.5 * (input[end].position.x + p1[internal_limit].x);
    p2[internal_limit].y = 0.5 * (input[end].position.y + p1[internal_limit].y);

}

// build a bezier curve passing through a set of states
void astar::CGSmoother::DrawBezierCurve(
                const std::vector<astar::State2D> &input,
                std::vector<astar::State2D> &output,
                const std::vector<astar::Vector2D<double>> &p1,
                const std::vector<astar::Vector2D<double>> &p2,
                unsigned int start, unsigned int end) {

    // get the grid resoluiton
    double resolution = grid.GetResolution();
    double inverse_resolution = grid.GetInverseResolution();
    double resolution_factor = resolution;
    double res2 = std::pow(resolution, 2);

    // the four points to tbe interpolated
    std::vector<astar::Vector2D<double>> piece(4);

    for (unsigned int i = start, j = 0; i < end; ++i, ++j) {

        // direct access
        const astar::Vector2D<double> &left(input[i].position), &right(input[i+1].position);

        // a tmp helper
        astar::State2D tmp(input[i]);

        // set the coming to stop flag
        tmp.coming_to_stop = false;

        // push the current point to the output path
        output.push_back(tmp);

        // get the distance betweeng the left and right points
        double d = left.Distance(right);

        if (resolution_factor < d) {

            // copy the points
            piece[0].x = left.x;
            piece[0].y = left.y;

            piece[1].x = p1[j].x;
            piece[1].y = p1[j].y;

            piece[2].x = p2[j].x;
            piece[2].y = p2[j].y;

            piece[3].x = right.x;
            piece[3].y = right.y;

            // draw the curve between the points
            double time_step = resolution/d;
            double t = time_step;

            // iterate over time [0, 1] and compute the points in the interval
            while (t < 1.0) {

                // draw the points
                tmp.position = GetBezierPoint(piece, t);

                if (res2 < tmp.position.Distance2(output.back().position) && res2 < tmp.position.Distance2(right)) {

                    // unset the coming to stop flag to false
                    tmp.coming_to_stop = false;

                    // push to the output vector
                    output.push_back(tmp);

                }

                // update the time value
                t += time_step;

            }

            // update the last inserted state
            output.back().coming_to_stop = input[i].coming_to_stop;

        }

    }

}

// interpolate a given path
astar::StateArrayPtr astar::CGSmoother::Interpolate(astar::StateArrayPtr path) {

    // the resulting path
    astar::StateArrayPtr interpolated_path = new StateArray();

    if (3 <= path->states.size()) {

        // break the path into subpieces
        BreakPath(path);

        // direct access
        std::vector<astar::State2D> &input(path->states);
        std::vector<astar::State2D> &output(interpolated_path->states);

        // how many stops?
        unsigned int stops = stopping_points.size();

        // the start index
        start = 0;

        for (unsigned int i = 0; i < stops; ++i) {

            // get the upper limit
            end = stopping_points[i];

            // the control points
            std::vector<astar::Vector2D<double>> p1, p2;

            // compute and build the bezier control points
            BuildBezierControlPoints(input, p1, p2, start, end);

            // interpolate the subpath
            // add the interpolated states to the output vector
            DrawBezierCurve(input, output, p1, p2, start, end);

            // update the start index to the next iteration
            start = end;

        }

        // add the last element
        output.push_back(input.back());

    } else {

        // copy the input states
        interpolated_path->states = path->states;

    }

    return interpolated_path;

}

// the main smooth function
astar::StateArrayPtr astar::CGSmoother::Smooth(astar::InternalGridMapRef grid_, astar::VehicleModelRef vehicle_, astar::StateArrayPtr raw_path) {

    // update the grid and vehicle references
    grid = grid_;
    vehicle = vehicle_;

    // show the map
    // ShowPath(raw_path);

    // conjugate gradient based on the Polak-Ribiere formula
    ConjugateGradientPR(raw_path);

    // show the map
    // ShowPath(raw_path);

    // now, interpolate the entire path
    // astar::StateArrayPtr interpolated_path = new astar::StateArray();
    // interpolated_path->states = raw_path->states;
    // return interpolated_path;
    astar::StateArrayPtr interpolated_path = Interpolate(raw_path);

    // show the map
    // ShowPath(interpolated_path);

    // minimize again the interpolated path
    // conjugate gradient based on the Polak-Ribiere formula
    ConjugateGradientPR(interpolated_path);

    // show the map
    // ShowPath(interpolated_path, false);

    // return the new interpolated path
    return interpolated_path;

}
