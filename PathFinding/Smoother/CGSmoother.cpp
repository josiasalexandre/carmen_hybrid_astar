#include "CGSmoother.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

astar::CGSmoother::CGSmoother(astar::InternalGridMapRef map, astar::VehicleModelRef vehicle_) :
    wo(0.02), ws(4.0), wp(0.02), wk(6.0), dmax(5.0), vorodmax(20),
    alpha(0.2), grid(map), vehicle(vehicle_), kmax(0.22),
    cg_status(astar::CGIddle), fx(), gx(nullptr), gx_norm(), fx1(), gx1(nullptr), gx1_norm(), bestfx(), bestgx_norm(), x1mx(), x1mx_norm(), s(), s_norm(), sg(),
    locked_positions(), max_iterations(300), dim(0), step(0.01), default_step_length(1.0), stepmax(1e06), stepmin(1e-12),
    ftol(1e-04), gtol(0.99999), xtol(1e-06)
{

    // update the inverse dmax
    inverse_vorodmax2 = 1.0/ (vorodmax * vorodmax);

    // start the x and x1 vectors
    x = new astar::StateArray();
    x1 = new astar::StateArray();

    // the best solution so far
    bestx = new astar::StateArray();

    // start the gx and gx1 gradient vectors
    gx = new astar::Vector2DArray<double>();
    gx1 = new astar::Vector2DArray<double>();

    // the best solution gradient
    bestgx = new astar::Vector2DArray<double>();

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
double astar::CGSmoother::CostFunction(astar::StateArrayPtr input) {

    // the partial values
    double obstacle = 0.0, potential = 0.0, curvature = 0.0, smooth = 0.0;

    // some helpers
    astar::Vector2D<double> dxi, dxip1;

    // nearest obstacle distance
    double obst_distance;

    // nearest voronoi distance
    double voro_distance;

    // get the third last limit
    unsigned int limit = dim - 2;

    astar::Vector2D<double> tmp(0.0, 0.0);

    // the tmp solution direct access
    std::vector<astar::State2D> &xs(input->states);

    double d;

    // iterate from the third element till the third last
    // and get the individual derivatives
    for (unsigned int i = 2; i < limit; ++i) {

        // is the current pose a stop one?
        if (!locked_positions[i]) {

            // get the position references
            astar::Vector2D<double> &xim1(xs[i-1].position), &xi(xs[i].position), &xip1(xs[i+1].position);

            // THE FUNCTION VALUE
            // update the obstacle and potential field terms
            // get the nearest obstacle distance
            obst_distance = grid.GetObstacleDistance(xi);

            // the obstacle and potential field terms are updated only when dmax >= obst_distance
            // otherwise we consider the term += 0.0;
            if (dmax >= obst_distance) {

                d = (obst_distance - dmax) * (obst_distance - dmax);

                // update the obstacle term
                obstacle += d;

                // get the nearest voronoi edge distance
                voro_distance = grid.GetVoronoiDistance(xi);

                // update the Voronoi potential field term
                // can it become a Nan?
                potential += (alpha / (alpha  + obst_distance)) * (voro_distance / (obst_distance  + voro_distance)) * (d * inverse_vorodmax2);

            }

            // get the current displacement
            dxi.x = xi.x - xim1.x;
            dxi.y = xi.y - xim1.y;

            // get the next displacement
            dxip1.x = xip1.x - xi.x;
            dxip1.y = xip1.y - xi.y;

            // update the curvature term
            double dxip1_ = std::atan2(dxip1.y, dxip1.x);
            double dxi_ = std::atan2(dxi.y, dxi.x);
            double n = dxi.Norm();

            curvature += std::pow((std::fabs(std::atan2(dxip1.y, dxip1.x) - std::atan2(dxi.y, dxi.x)) / dxi.Norm() - kmax), 2);

            // update the smooth term
            tmp.x = dxip1.x - dxi.x;
            tmp.y = dxip1.y - dxi.y;
            smooth += tmp.Norm2();

        }

    }

    // set the euclidean norm final touch
    // gx_norm = std::sqrt(gx_norm); It's no the euclidean version
    return wo*obstacle + wp*potential + wk*curvature + ws*smooth;

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
        double inverse_norm2 = 1.0/xip1.Norm2(); // TODO ERRADO
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
        res.x += kim1.x + ki.x + kip1.x;
        res.y += kim1.y + ki.y + kip1.y;

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
double astar::CGSmoother::ComputeGradient(astar::StateArrayPtr const path, std::vector<astar::Vector2D<double>> &gradient) {

    // get the third last limit
    unsigned int limit = dim - 2;

    astar::Vector2D<double> tmp(0.0, 0.0);

    // reset the gradient norm
    double norm2 = 0.0;

    // the current solution direct access
    std::vector<astar::State2D> &xs(path->states);

    // iterate from the third element till the third last
    // and get the individual derivatives
    for (unsigned int i = 2; i < limit; ++i) {

        // is the current pose a stop one?
        if (!locked_positions[i]) {

            // get the position references
            astar::Vector2D<double> &xim2(xs[i-2].position), &xim1(xs[i-1].position),
                    &xi(xs[i].position), &xip1(xs[i+1].position), &xip2(xs[i+2].position);

            // add the obstacle and the voronoi contributions
            tmp.Add(GetObstacleAndVoronoiDerivatives(xim1, xi, xip1));

            // add the curvature term contribution
            tmp.Add(GetCurvatureDerivative(xim1, xi, xip1));

            // set up the smooth path derivatives contribution
            tmp.x += ws * (xim2.x - 4.0 * xim1.x + 6.0 * xi.x - 4.0 * xip1.x + xip2.x);
            tmp.y += ws * (xim2.y - 4.0 * xim1.y + 6.0 * xi.y - 4.0 * xip1.y + xip2.y);

        }

        // save the current derivative
        gradient[i] = tmp;

        // update the gradient norm
        norm2 += tmp.Norm2();

        // reset the tmp vector
        tmp.x = 0.0;
        tmp.y = 0.0;

    }

    // set the euclidean norm final touch
    // gx_norm = std::sqrt(gx_norm); It's no the euclidean version

    // return the norm 2
    return norm2;

}

// take a step at the current direction vector (s)
void astar::CGSmoother::TakeStep(double factor) {

    // get the limit
    unsigned int limit = dim - 2;

    std::vector<astar::State2D> &current(x->states);
    std::vector<astar::State2D> &next(x1->states);

    // reset the dx norm
    x1mx_norm = 0.0;

    // update the current position
    for (unsigned int i = 2; i < limit; i++) {

        if (!locked_positions[i]) {

            // see the plus signal: the factor should be negative
            // the factor value is equal to -step/ s_norm
            // the s vector is not normalized, so ...
            next[i].position.x = current[i].position.x + factor * s[i].x;
            next[i].position.y = current[i].position.y + factor * s[i].y;

            //x1mx[i].x = next[i].position.x - current[i].position.x;
            //x1mx[i].y = next[i].position.y - current[i].position.y;

            x1mx_norm += (next[i].position.x - current[i].position.x)*(next[i].position.x - current[i].position.x);
            x1mx_norm += (next[i].position.y - current[i].position.y)*(next[i].position.y - current[i].position.y);

            // verify the safe condition
            if (!grid.isSafePlace(vehicle.GetVehicleBodyCircles(next[i]), vehicle.safety_factor)) {

                // lock the current position
                locked_positions[i] = true;

                // the old position is the valid one
                next[i].position = current[i].position;

            }

        }

    }

}

// custom function to evaluate the cost function and the update the gradient at the same time
// it uses the x1 as the input array
// the resulting cost value is saved in the internal fx1 variable
// the gradient vector gx1 is updated and the gradient norm is saved in gx1_norm
void astar::CGSmoother::EvaluateFunctionAndGradient() {

    // the partial values
    double obstacle = 0.0, potential = 0.0, curvature = 0.0, smooth = 0.0;

    // some helpers
    astar::Vector2D<double> dxi, dxip1;

    // nearest obstacle distance
    double obst_distance;

    // nearest voronoi distance
    double voro_distance;

    // get the third last limit
    unsigned int limit = dim - 2;

    astar::Vector2D<double> tmp(0.0, 0.0), tmp1(0.0, 0.0);

    // reset the gx1 gradient norm
    gx1_norm = 0.0;

    // the tmp solution direct access
    std::vector<astar::State2D> &xs(x1->states);

    // the gradient direct access
    std::vector<astar::Vector2D<double>> &gradient(gx1->vs);

    double d;
    unsigned int i;

    // it's a boundary value problem
    // the first and last points should no be modified

    // iterate from the third element till the third last
    // and get the individual derivatives
    for (i = 2; i < limit; ++i) {

        // is the current pose a stop one?
        if (!locked_positions[i]) {

            // get the position references
            astar::Vector2D<double> &xim2(xs[i-2].position), &xim1(xs[i-1].position),
                    &xi(xs[i].position), &xip1(xs[i+1].position), &xip2(xs[i+2].position);

            // THE FUNCTION VALUE
            // update the obstacle and potential field terms
            // get the nearest obstacle distance
            obst_distance = grid.GetObstacleDistance(xi);

            // the obstacle and potential field terms are updated only when dmax >= obst_distance
            // otherwise we consider the term += 0.0;
            if (dmax >= obst_distance) {

                d = (dmax - obst_distance) * (dmax - obst_distance);

                // update the obstacle term
                obstacle += d;

                // get the nearest voronoi edge distance
                voro_distance = grid.GetVoronoiDistance(xi);

                // update the Voronoi potential field term
                // can it become a Nan?
                //potential += grid.GetPathCost(xi);
                potential += (alpha / (alpha  + obst_distance)) * (voro_distance / (obst_distance  + voro_distance)) * ((d - vorodmax*vorodmax)* inverse_vorodmax2);

                // add the obstacle and the voronoi contributions
                // overloaded method
                tmp1.Add(GetObstacleAndVoronoiDerivatives(xim1, xi, xip1, obst_distance, voro_distance));

            }

            // get the current displacement
            dxi.x = xi.x - xim1.x;
            dxi.y = xi.y - xim1.y;

            // get the next displacement
            dxip1.x = xip1.x - xi.x;
            dxip1.y = xip1.y - xim1.y;

            // update the curvature term
            curvature += std::pow((std::fabs(std::atan2(dxip1.y, dxip1.x) - std::atan2(dxi.y, dxi.x)) / dxi.Norm() - kmax), 2);

            // update the smooth term
            tmp.x = xip1.x - 2.0 * xi.x + xim1.x;
            tmp.y = xip1.y - 2.0 * xi.y + xim1.y;


            smooth += tmp.Norm2();

            // add the curvature term contribution
            //tmp1.Add(GetCurvatureDerivative(xim1, xi, xip1));

            // set up the smooth path derivatives contribution
            tmp1.x += 2.0*(xim2.x - 4.0 * xim1.x + 6.0 * xi.x - 4.0 * xip1.x + xip2.x);
            tmp1.y += 2.0*(xim2.y - 4.0 * xim1.y + 6.0 * xi.y - 4.0 * xip1.y + xip2.y);

        }

        // save the current derivative
        gradient[i] = tmp1;

        // update the gradient norm
        gx1_norm += tmp1.Norm2();

        // reset the tmp and tmp1 vectors
        tmp.x = 0.0;
        tmp.y = 0.0;
        tmp1 = tmp;

    }


    // set the euclidean norm final touch
    // gx_norm = std::sqrt(gx_norm); It's no the euclidean version
    //fx1 = wo*obstacle + wp*potential + wk*curvature + ws*smooth;
    fx1 = ws*smooth + wo*obstacle + wp*potential;

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
        // move x1 to the new position
        TakeStep(stp * lambda);

        // evaluate the function and the gradient
        // fp = CostFunction(x1);
        // gx1_norm = ComputeGradient(x1, gx1->vs);
        EvaluateFunctionAndGradient();

        fp = fx1;

        if (bestfx > fx1) {

            // update the best solution so far
            bestx->states = x1->states;
            bestfx = fx1;
            bestgx->vs = gx1->vs;
            bestgx_norm = gx1_norm;

        }

        // get the directional derivative
        sgp = astar::Vector2DArray<double>::DotProduct(s, gx1->vs) * lambda;

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

// setup the first iteration and configure the containers
bool astar::CGSmoother::Setup(astar::StateArrayPtr path, bool locked) {

    if (astar::CGIddle == cg_status) {

        // set the kmax value
        kmax = 1.0/(vehicle.min_turn_radius * 1.1);

        // get the input path size and store the problem's dimension
        dim = path->states.size();

        // set the first value to the solution vector
        x->states = path->states;

        // set the second point values
        x1->states = path->states;

        // set the best point values
        bestx->states = path->states;

        // update the dx norm
        x1mx_norm = std::numeric_limits<double>::max();

        // tmp vector
        astar::Vector2D<double> tmp(0.0, 0.0);

        // reset the gradient at x
        gx->vs.resize(dim, tmp);

        // reset the gradient at x1
        gx1->vs.resize(dim, tmp);

        // reset the gradient displacement
        gx1mgx.resize(dim, tmp);

        // reset the x1mx size
        x1mx.resize(dim, tmp);

        // resize the best position gradient
        bestgx->vs.resize(dim, tmp);

        if (!locked) {

            // reset the lock vector size
            locked_positions.resize(dim, false);

            // direct access
            std::vector<astar::State2D> &states(path->states);

            // set the index acess limit
            unsigned int limit = dim - 2;

            // lock the stoping points
            for (unsigned int i = 2; i < limit; ++i) {

                locked_positions[i] = states[i].gear != states[i-1].gear;

            }

        }

        // evaluate the cost function at the given position
        // fx = CostFunction(x);

        // evaluate the first gradient
        // gx_norm = ComputeGradient(x, gx->vs);

        EvaluateFunctionAndGradient();

        bestfx = fx = fx1;
        bestgx_norm = gx_norm = gx1_norm;

        // flip the gradients
        astar::Vector2DArrayPtr<double> vs = gx;
        gx = gx1;
        gx1 = vs;

        // reset the direction vector
        s.resize(dim, tmp);
        for (unsigned int i = 0; i < dim; ++i) {

            s[i].x = -gx->vs[i].x;
            s[i].y = -gx->vs[i].y;

        }

        // the s norm is equal to the gradient's norm
        gx1_norm = s_norm = gx_norm;

        // multiply the current direction with the inverse gradient
        // it's the directional derivative along s
        sg = astar::Vector2DArray<double>::DotProduct(s, gx->vs) / s_norm;

        /*// get the first step
        if (1 == MTLineSearch(1.0/s_norm)) {

            // we got the first step
            // SUCCESS!

            // get the displacement between the two gradients
            astar::Vector2DArray<double>::SubtractCAB(gx1mgx, gx1->vs, gx->vs);

            // get the gamma value
            // based on Powell 1983
            double gamma = std::max(astar::Vector2DArray<double>::DotProduct(gx1mgx, gx1->vs)/astar::Vector2DArray<double>::DotProduct(gx->vs, gx->vs), 0.0);

            // the new x1 point is the new position
            // update the conjugate direction at the new position
            // it also updates the the curent conjugate direction norm
            UpdateConjugateDirection(s, gx1->vs, gamma);

            // the new position is a better one
            astar::StateArrayPtr tmp = x;
            x = x1;
            x1 = tmp;

            // flip the gradient vectors
            astar::Vector2DArrayPtr<double> gradient = gx1;
            gx1 = gx;
            gx = gradient;

            // copy the norm
            gx_norm = gx1_norm;

        }*/

        // reset the tolerances
        ftol = gtol = xtol = 1e-04;

        // start the minimizer
        cg_status = astar::CGContinue;

        return true;

    }

    return false;

}

// update the conjugate direction -> s(i+1) = -gradient + gamma * s(i)
void astar::CGSmoother::UpdateConjugateDirection(std::vector<astar::Vector2D<double>> &s, const std::vector<astar::Vector2D<double>> &gradient, double gamma) {

    // reset the current s_norm
    s_norm = 0.0;

    // reset the current slope
    sg = 0.0;

    // the limit index
    unsigned int limit = dim - 2;

    // assuming the dimension is valid
    for (unsigned int i = 2; i < limit; ++i) {

        s[i].x = gamma*s[i].x - gradient[i].x;
        s[i].y = gamma*s[i].y - gradient[i].y;

        s_norm += s[i].Norm2();

        sg += s[i].x * gradient[i].x + s[i].y * gradient[i].y;

    }

    sg /= s_norm;
}

// the Polak-Ribiere Conjugate Gradient Method With More-Thuente Line Search
void astar::CGSmoother::ConjugateGradientPR(astar::StateArrayPtr path, bool locked) {

    // first step, setup the optmization process
    // the gradient is the first direction
    if (!Setup(path, locked)) {

        // could not start the minimizer
        // we should make shure the old process is finished
        std::cout << "Could not start the optimization process!!\n";

        return;

    }

    unsigned int iter = 0;

    // the direction is up or down?
    double direction;
    double inverse_snorm;
    double betha = 0;

    // the main CG Loop
    while (astar::CGContinue == cg_status && iter < max_iterations) {

        // update the iterator counter
        iter += 1;

        int info = MTLineSearch(1.0/s_norm);

        if (xtol > x1mx_norm) {

            // test the stuck case

            // set the stuck case!
            cg_status = astar::CGStuck;

            // set
            break;

        } else if (gtol > gx1_norm) {

            // success!
            cg_status = astar::CGSuccess;

            // exit
            break;

        } else if (ftol > fx1) {

            // success
            cg_status = astar::CGSuccess;

            // exit
            break;

        } else {

            if (1 != info) {

                // bestx is the best solution so far

                // flip the function value
                double v = bestfx;
                bestfx = fx1;
                fx1 = v;

                // flip the solution vectors
                astar::StateArrayPtr tmp = bestx;
                bestx = x1;
                x1 = tmp;

                // flip the gradients
                astar::Vector2DArrayPtr<double> gradient = bestgx;
                bestgx = gx1;
                gx1 = gradient;

                // flip the norms
                v = bestgx_norm;
                bestgx_norm = gx1_norm;
                gx1_norm = v;

            }

            // SUCCESS!
            // verify the restart case
            if (0 != iter % dim) {

                // get the displacement between the two gradients
                astar::Vector2DArray<double>::SubtractCAB(gx1mgx, gx1->vs, gx->vs);

                // get the betha value
                // based on Powell 1983
                betha = std::max(astar::Vector2DArray<double>::DotProduct(gx1mgx, gx1->vs)/astar::Vector2DArray<double>::DotProduct(gx->vs, gx->vs), 0.0);

            } else {

                // set gamma to zero, it restarts the conjugate gradient
                betha = 0.0;

            }

            // the new x1 point is the new position
            // update the conjugate direction at the new position
            // it also updates the the curent conjugate direction norm
            UpdateConjugateDirection(s, gx1->vs, betha);

            // the new position is a better one
            astar::StateArrayPtr tmp = x;
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

    // set the cg status to iddle
    cg_status = astar::CGIddle;

    // copy the resulting path back
    path->states = x1->states;

}

// interpolate a given path
astar::StateArrayPtr astar::CGSmoother::Interpolate(astar::StateArrayPtr path) {

    // the resulting path
    astar::StateArrayPtr interpolated_path = new StateArray();

    // direct access
    std::vector<astar::State2D> &input(path->states);
    std::vector<astar::State2D> &output(interpolated_path->states);

    // get the iterators
    std::vector<astar::State2D>::iterator end = input.end();
    std::vector<astar::State2D>::iterator current = input.begin();
    std::vector<astar::State2D>::iterator next = current;
    ++next;

    // the grid resolution
    double resolution = grid.GetResolution();
    double resolution_factor =  resolution * 2.0;

    // invert the resolution, avoiding a lot o divisions
    double inverse_resolution = 1.0/resolution;

    // helpers
    unsigned int n;
    double d, dx, dy;

    // the tmp state
    astar::State2D tmp;

    // clear the locked positions vector
    locked_positions.clear();

    bool cst;

    // iterate over almost the entire input path and interpolate between the given positions
    // the reeds shepp action set is already interpolated
    while (next != end) {

        // push back the current one
        // copy the current state
        tmp.position = current->position;
        tmp.gear = current->gear;

        // set the coming to stop flag to false
        tmp.coming_to_stop = false;

        // add the current state to the output path
        output.push_back(tmp);

        // lock this point
        locked_positions.push_back(true);

        // get the distance between the current and the next poses
        d = current->position.Distance(next->position);

        // verify the distance between the poses
        if (resolution_factor < d) {

            // interpolate the current and the next states
            n = std::floor(d * inverse_resolution);

            // get the x and y step lengths
            dx = (next->position.x - current->position.x) / ((double) n);
            dy = (next->position.y - current->position.y) / ((double) n);

            for (unsigned int i = 0; i < n-2; ++i) {

                // update the tmp position
                tmp.position.x += dx;
                tmp.position.y += dy;

                // add the interpolated point to the output path
                output.push_back(tmp);

                // unlock this point
                locked_positions.push_back(false);

            }

            // get the coming to stop flag
            tmp.coming_to_stop = current->coming_to_stop;

            // get the next position
            tmp.position.x += dx;
            tmp.position.y += dy;

            // add the last interpolated element
            output.push_back(tmp);

            // unlock this point
            locked_positions.push_back(false);

        }

        // update the iterators
        current = next;
        ++next;
    }

    // add the last state
    output.push_back(*current);

    // lock this position
    locked_positions.push_back(true);

    return interpolated_path;

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
        // get the currnt point
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

// the main smooth function
astar::StateArrayPtr astar::CGSmoother::Smooth(astar::InternalGridMapRef grid_, astar::VehicleModelRef vehicle_, astar::StateArrayPtr raw_path) {

    // update the grid and vehicle references
    grid = grid_;
    vehicle = vehicle_;

    // minimize the current state array
    // conjugate gradient based on the Polak-Ribiere formula
    ConjugateGradientPR(raw_path);

    // get the map
    ShowPath(raw_path);

    // now, interpolate the entire path
    // astar::StateArrayPtr interpolated_path = new astar::StateArray();
    // interpolated_path->states = raw_path->states;
    astar::StateArrayPtr interpolated_path = Interpolate(raw_path);

    // get the map
    // ShowPath(interpolated_path);

    // minimize again the interpolated path
    // conjugate gradient based on the Polak-Ribiere formula
    // ConjugateGradientPR(interpolated_path, true);

    // get the map
    ShowPath(interpolated_path, false);

    // return the new interpolated path
    return interpolated_path;

}
