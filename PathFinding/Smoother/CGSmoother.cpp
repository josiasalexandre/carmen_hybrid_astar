#include "CGSmoother.hpp"

astar::CGSmoother::CGSmoother(astar::InternalGridMapRef map, astar::VehicleModelRef vehicle_) :
	wo(0.002), ws(4.0), wp(0.2), wk(4.0), dmax(5.0), vorodmax(20),
	alpha(0.2), grid(map), vehicle(vehicle_), kmax(vehicle_.max_curvature * 1.2), cg_status(astar::CGIddle),
	fx(), fx1(), gradient(), gx_norm(), gx1_norm(), s(), s_norm(),
	locked_positions(), max_iterations(300), dim(0), step(), tol(1e-04)
{
	// update the inverse dmax
	inverse_dmax2 = 1.0/ (dmax * dmax);

	// start the x and x1 vectors
	x = new astar::StateArray();
	x1 = new astar::StateArray();

}

// basic destructor
astar::CGSmoother::~CGSmoother() {

	// remove the x and x1 vectors
	delete x;
	delete x1;

}

// the cost function to be minimized
double astar::CGSmoother::CostFunction(astar::StateArrayPtr input) {

	// the partial values
	double obstacle = 0.0, potential = 0.0, curvature = 0.0, smooth = 0.0;

	// some helpers
	astar::Vector2D<double> dxi, dxip1;

	// the vector iterators
	std::vector<astar::State2D>::iterator current, next, prev, end;
	current = next = prev = input->states.begin();

	// advance the current the and the next pointers
	std::advance(current, 1);
	std::advance(next, 2);

	// get the end iterator pointer
	end = input->states.end();

	// nearest obstacle distance
	double obst_distance;

	// nearest voronoi distance
	double voro_distance;

	// tmp
	astar::Vector2D<double> tmp;

	double d;

	// iterate from the second element till the last but one
	while (next != end) {

		// set the references
		// direct access
		astar::Vector2D<double> &xim1(prev->position), &xi(current->position), &xip1(next->position);

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
			voro_distance = grid.GetVoronoiDistance(current->position);

			// update the Voronoi potential field term
			// can it become a Nan?
			potential += (alpha / (alpha  + obst_distance)) * (voro_distance / (obst_distance  + voro_distance)) * (d * inverse_dmax2);

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
		tmp.x = dxip1.x - dxi.x;
		tmp.y = dxip1.y - dxi.y;
		smooth += tmp.Norm();

		// update all pointers
		prev = current;
		current = next;
		++next;

	}

	// return the cost function scalar value
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
				double pvdv = alpha_over_obstacle * ((omd*omd)*inverse_dmax2) * (obst_distance / (opv*opv));

				// get the potential field derivative of the nearest obstacle with respect to the current position
				double pvdo = alpha_over_obstacle * (voro_distance / opv) * ((omd) * inverse_dmax2) *
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
		double denom = xi.Norm() * xip1.Norm();

		// the normalized orthogonal complements
		astar::Vector2D<double> p1, p2;

		// some helpers
		double inverse_norm2 = 1.0/xip1.Norm2();
		double nx = -xip1.x;
		double ny = -xip1.y;
		double tmp1 = (xi.x * nx + xi.y * ny);
		double tmp2 = tmp1 * inverse_norm2;

		// set the first othogonal complement
		p1.x = xi.x - nx * tmp2;
		p1.y = xi.y - ny * tmp2;

		// reset the helpers
		inverse_norm2 = 1.0/xi.Norm();
		tmp2 = tmp1 * inverse_norm2;

		// set the second othogonal complement
		p2.x = nx - xi.x * tmp2;
		p2.y = ny - xi.y * tmp2;

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
		kim1.Subtract(coeff2);
		// aply the factor
		kim1.Multiply(0.25 * k);

		kip1 = p1;
		kip1.Multiply(coeff1);
		// aply the factor
		kip1.Multiply(0.25 * k);

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
double astar::CGSmoother::ComputeGradient(astar::StateArrayPtr const path) {

	// get the third last limit
	unsigned int limit = dim - 3;

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

// reset all values in a given position array
void astar::CGSmoother::SetZero(std::vector<astar::Vector2D<double>> &v) {

	// get the vector size
	unsigned int v_size = v.size();

	for (unsigned int i = 0; i < v_size; ++i) {

		v[i].x = 0.0;
		v[i].y = 0.0;

	}

}

// custom dot product
double astar::CGSmoother::DotProduct(
		const std::vector<astar::Vector2D<double>> &a, const std::vector<astar::Vector2D<double>> &b)
{
	// the resulting dot product
	double dot = 0.0;

	if (a.size() == b.size()) {

		// the vector size
		unsigned int size = a.size();

		for (unsigned int i = 0; i < size; ++i) {

			dot += a[i].x * b[i].x;
			dot += a[i].y * b[i].y;

		}

		return dot;

	} else {

		// exception
		throw std::exception();

	}

}

// scale a given Vector2D array
void astar::CGSmoother::ScaleVector(double value, std::vector<astar::Vector2D<double>> &v) {

	for (unsigned int i = 0; i < dim; ++i) {

		v[i].x *= value;
		v[i].y *= value;

	}

}

// add the first vector2D array to the second one
void astar::CGSmoother::AddVectors(const std::vector<astar::Vector2D<double>> &a, std::vector<astar::Vector2D<double>> &b) {

	// get the size
	unsigned int size = a.size();

	if (size == b.size()) {

		for (unsigned int i = 0; i < size; ++i) {

			b[i].x += a[i].x;
			b[i].y += a[i].y;

		}

	}

}

// compute a norm 2 for a Vector2D array
double astar::CGSmoother::VectorNorm2(const std::vector<astar::Vector2D<double>> &v) {

	// the resulting norm
	double norm = 0.0;

	// get the size
	unsigned int size = v.size();

	for (unsigned int i = 0; i < size; ++i) {

		norm += v[i].Norm2();

	}

	return norm;

}

// take a step at the current direction vector (s)
void astar::CGSmoother::TakeStep(double factor) {

	// get the limit
	unsigned int limit = dim - 2;

	std::vector<astar::State2D> &current(x->states);
	std::vector<astar::State2D> &next(x1->states);

	// update the current position
	for (unsigned int i = 2; i < limit; i++) {

		if (!locked_positions[i]) {

			// see the plus signal: the factor should be negative
			// the factor value is equal to -step/ s_norm
			// the s vector is not normalized, so ...
			next[i].position.x = current[i].position.x + factor * s[i].x;
			next[i].position.y = current[i].position.y + factor * s[i].y;

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

// vanilla line search approach
double astar::CGSmoother::LineSearch(double fa, double fc, double lambda, double sg) {

	double fb;

	// the initial step limit
	double stepc = step;

	// the desired step length
	double stepb;

	do {

		// get a new stepb value
		// parabolic interpolation
		double u = std::fabs(sg * lambda * stepc);
		stepb = 0.5 * stepc * u / ((fc - fa) + u);

		// take a new step based on the current stepb value
		TakeStep(-stepb * lambda);

		// verify the progress
		if (isStuck(x->states, x1->states)) {

			// could not improve the solution, maybe a local minimum or numerical instability
			step = 0.0;
			fx = fa;

			return 0.0;

		}

		// evaluate the cost function at the new position
		fc = fb = CostFunction(x1);

		// set the new step limit
		stepc = stepb;

	} while (fa >= fb && 0.0 < stepb);

	// success!
	step = stepb;
	fx = fb;

	// the new position at x1 is a better one, so let's flip then
	astar::StateArrayPtr tmp = x;
	x = x1;
	x1 = tmp;

	// compute the new gradient
	gx_norm = ComputeGradient(x);

	return a;
}

// Starting at (x0, f0) move along the direction p to find a minimum
// f(x0 - lambda * p), returning the new point x1 = x0-lambda*p,
// f1=f(x1) and g1 = grad(f) at x1.
void astar::CGSmoother::WalkDownhill(double fa, double fc, double lambda, double sg) { /* TODO */ }

// setup the first iteration and configure the containers
bool astar::CGSmoother::Setup(astar::StateArrayPtr path) {

	if (astar::CGIddle == cg_status) {

		// get the input path size and store the problem's dimension
		dim = path->states.size();

		// set the first value to the solution vector
		x->states = path->states;

		// evaluate the cost function at the given position
		fx = CostFunction(x);

		// reset the steepest descent direction vector
		gradient.resize(dim, astar::Vector2D<double>(0.0, 0.0));

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

		// evaluate the first gradient
		gx_norm = ComputeGradient(x);

		// reset the direction vector
		// WARNING!! we save the positive value, so we need to subtract this direction
		s = gradient;

		// the s norm is equal to the gradient's norm
		s_norm = gx_norm;

		// set the second point values
		x1->states = x->states;

		// the cost function is not evaluated in this second point
		fx1 = std::numeric_limits<double>::max();

		// the norm of the gradient in the second point
		gx1_norm = fx1;

		// set initial step size
		step = 0.01;

		// set the tolerance
		tol = 1e-4;

		// start the minimizer
		cg_status = astar::CGContinue;

		return true;

	}

	return false;

}

// minimize the cost function based on a given state array path
void astar::CGSmoother::Minimize(astar::StateArrayPtr path) {

	// first step, setup the optmization process
	// the gradient is the first direction
	if (!Setup(path)) {

		// could not start the minimizer
		// we should make shure the old process is finished
		std::cout << "Could not start the optimization process!!\n";

		return;

	}

	unsigned int iter = 0;

	// the main loop
	while (astar::CGContinue == cg_status && iter < max_iterations) {

		// update the counter
		iter += 1;

		// test the gradient norm and the direction vector norm
		if (0.0 == s_norm || 0.0 == gx_norm) {

			// reset the direction vector
			SetZero(s);

			// break
			cg_status = astar::CGStuck;

		} else {

			// determine wich direction is downhill
			double sg = DotProduct(s, gradient);
			double direction = (0.0 <= sg) ? 1.0 : -1.0;

			double lambda = direction / s_norm;

			// take the next step
			TakeStep(-step * lambda);

			// evaluate the cost function at the next point
			double fc = CostFunction(x1);

			// compare the costs
			if (fc < fx) {

				// success! The current iteration was a great move!

				// set the new cost function value
				fx = fc;

				// x1 is a better aproximatted solution, let's flip then
				astar::StateArrayPtr tmp = x;
				x = x1;
				x1 = tmp;

				// update the gradient
				gx_norm = ComputeGradient(x);

				// increase the step size
				step *= 2.0;

			} else {

				// not a good move, let's try a line search
				step = LineSearch(fx, fc, lambda, sg);

				if (0.0 == step) {

					// should not continue, we got a
					cg_status = astar::CGStuck;

				} else {

					// we found a better step
					// Let's walk downhill
					WalkDownhill(fx, fc, lambda, sg);

					// choose a new conjugate direction for the next step
					if (0 == iter % dim) {

						// restart the direction vector
						s = gradient;

						// copy the gradient norm
						s_norm = gx1_norm;

					} else {

						// update the direction vector
						// s(i+1) = gradient  - beta * s(i)
						// using beta from Fletcher-Reeves formula
						double beta = std::pow(gx1_norm / gx_norm, 2);

						// scale the s vector
						ScaleVector(-beta, s);

						// add the gradient Vector2D to the s direction vector
						AddVectors(gradient, s);

						// update the norm
						s_norm = VectorNorm2(s);

					}

					// copy the gradient norm
					gx_norm = gx1_norm;

				}

			}

		}

		// test the norm of the gradient against the tolerance
		if (tol > gx_norm) {

			cg_status = astar::CGSuccess;

		}

	}

	// we can use the cg_status here to obtain some interesting information

	// copy the solution vector to the input path
	path->states = x->states;

	// we have finished the current minimization
	// reset the conjugate gradient
	cg_status = astar::CGIddle;

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
	double resolution_factor =  resolution * 1.2;

	// invert the resolution, avoiding a lot o divisions
	double inverse_resolution = 1.0/resolution;

	// helpers
	unsigned int n;
	double d, dx, dy;

	// the tmp state
	astar::State2D tmp;

	// iterate over almost the entire input path and interpolate between the given positions
	// the reeds shepp action set is already interpolated
	while (next != end) {

		// get the distance between the current and the next poses
		d = current->position.Distance(next->position);

		// verify the distance between the poses
		if (resolution_factor < d) {

			// copy the current state
			tmp.position = current->position;
			tmp.gear = current->gear;

			// set the coming to stop flag to false
			tmp.coming_to_stop = false;

			// add the current state to the output path
			output.push_back(tmp);

			// interpolate the current and the next states
			n = std::floor(d * inverse_resolution);

			// get the x and y step lengths
			dx = (next->position.x - current->position.x) / ((double) n + 1);
			dy = (next->position.y - current->position.y) / ((double) n + 1);

			for (unsigned int i = 1; i < n - 1; ++i) {

				// update the tmp position
				tmp.position.x += dx;
				tmp.position.y += dy;

				// add the interpolated point to the output path
				output.push_back(tmp);

			}

			// get the coming to stop flag
			tmp.coming_to_stop = current->coming_to_stop;

			// get the next position
			tmp.position.x += dx;
			tmp.position.y += dy;

			// add the last interpolated element
			output.push_back(tmp);

		}

		// update the iterators
		current = next;
		++next;
	}

	// add the last state
	output.push_back(*current);

	return interpolated_path;

}

// the main smooth function
astar::StateArrayPtr astar::CGSmoother::Smooth(astar::InternalGridMapRef grid_, astar::VehicleModelRef vehicle_, astar::StateArrayPtr raw_path) {

	// update the grid and vehicle references
	grid = grid_;
	vehicle = vehicle_;

	// minimize the current state array
	Minimize(raw_path);

	// now, interpolate the entire path
	astar::StateArrayPtr interpolated_path = Interpolate(raw_path);

	// minimize again the interpolated path
	Minimize(interpolated_path);

	// return the new interpolated path
	return interpolated_path;

}
