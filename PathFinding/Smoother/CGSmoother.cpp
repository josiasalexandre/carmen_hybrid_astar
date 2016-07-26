#include "CGSmoother.hpp"

astar::CGSmoother::CGSmoother(astar::InternalGridMapRef map, astar::VehicleModelRef vehicle_) :
	wo(0.002), ws(4.0), wp(0.2), wk(4.0), dmax(5.0), vorodmax(20), alpha(0.2), grid(map), vehicle(vehicle_), kmax(vehicle_.max_curvature * 1.2), tmp()
{
	// update the inverse dmax
	inverse_dmax2 = 1.0/ (dmax * dmax);

}

// the cost function to be minimized
double astar::CGSmoother::CostFunction(astar::StateArrayPtr input) {

	// the partial values
	double obstacle = 0.0, potential = 0.0, curvature = 0.0, smooth = 0.0;

	// some helpers
	astar::Vector2D<double> dxi, dxip1;

	// the vector iterators
	std::vector<astar::State2D>::iterator current, next, prev;
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
	double d;

	// direct access
	astar::Vector2D<double> &xim1, &xi, &xip1;

	// iterate from the second element till the last but one
	while (next != end) {

		// set the references
		xim1 = prev->position;
		xi = current->position;
		xip1 = next->position;

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
astar::CGSmoother::GetCurvatureDerivatives(const astar::Vector2D<double> &xim1, const astar::Vector2D<double> &xi, const astar::Vector2D<double> &xip1) {

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

		astar::Vector2D ki, kim1, kip1;
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
astar::Vector2D<double> astar::CGSmoother::GetSmoothPathDerivatives(
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

// the cost function derivative, it builds the current gradient
astar::Vector2DArrayPtr<double> astar::CGSmoother::Gradient(astar::StateArrayPtr input) {

	// build the output gradient array
	astar::Vector2DArrayPtr<double> output = new astar::Vector2DArray<double>();

	// direct access
	std::vector<astar::Vector2D<double>> &gradient(output->vs);

	// the vector iterators
	std::vector<astar::State2D>::iterator itm2, itm1, it, itp1, itp2;

	// the vector iterators
	itm2 = itm1 = it = itp1 = itp2 = input->states.begin();
	// get the end iterator pointer
	std::vector<astar::State2D>::iterator end = input->states.end();

	// advance the iterators
	++itp2;
	itm1 = itp2;
	++itp2;
	it = itp2;
	++itp2;
	itp1 = itp2;
	++itp2;

	/*
		std::advance(itm1, 1);
		std::advance(it, 2);
		std::advance(itp1, 3);
		std::advance(itp2, 4);
	*/

	// some Vector2D references
	astar::Vector2D<double> &xim2, &xim1, &xi, &xip1, &xip2;

	// reset the tmp Vector2D
	tmp.x = 0.0;
	tmp.y = 0.0;

	// append the first and second points derivatives
	gradient.push_back(tmp);
	gradient.push_back(tmp);

	astar::State2D current;

	// iterate from the second element till the last but one
	while (itp2 != end) {

		// get the current positions references
		xim2 = itm2->position;
		xim1 = itm1->position;
		xi = it->position;
		xip1 = itp1->position;
		xip2 = itp2->position;

		// add the obstacle and the voronoi contributions
		tmp.Add(GetObstacleAndVoronoiDerivatives(xim1, xi, xip1));

		// add the curvature term contribution
		tmp.Add(GetCurvatureDerivatives(xim1, xi, xip1));

		// set up the current smooth derivatives
		tmp.x += ws * (xim2.x - 4.0 * xim1.x + 6.0 * xi.x - 4.0 * xip1.x + xip2.x);
		tmp.y += ws * (xim2.y - 4.0 * xim1.y + 6.0 * xi.y - 4.0 * xip1.y + xip2.y);

		// append the current derivative
		gradient.push_back(tmp);

		// update the iterators
		itm2 = itm1;
		itm1 = it;
		it = itp1;
		itp1 = itp2;
		++itp2;

		// reset the tmp Vector2D
		tmp.x = 0.0;
		tmp.y = 0.0;

	}

	// append the last two derivatives
	gradient.push_back(tmp);
	gradient.push_back(tmp);

	return output;
}

// the main smooth function
void astar::CGSmoother::Smooth(astar::InternalGridMapRef grid_, astar::VehicleModelRef vehicle_, astar::StateArrayPtr raw_path) {

	// update the grid and vehicle references
	grid = grid_;
	vehicle = vehicle_;

	(void) raw_path;

	return;

}
