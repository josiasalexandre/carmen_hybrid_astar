#include "InternalGridMap.hpp"
#include <cmath>
using namespace astar;

InternalGridMap::InternalGridMap() :
	width(0), width_2(0),
	height(0), height_2(0),
	size(0),
	resolution(0), inverse_resolution(0), diagonal_resolution(0),
	origin(),
	grid_map(nullptr),
	voronoi()
{

}

// basic destructor
InternalGridMap::~InternalGridMap() {

	if (nullptr != grid_map) {
		delete [] grid_map;
	}

}

void InternalGridMap::InitializeGridMap(unsigned int h, unsigned int w, double res, const astar::Vector2D<double> &_origin) {

	if (w != width || h != height || res != resolution || _origin != origin) {

		if (nullptr != grid_map) {

			for (unsigned int i = 0; i < height; ++i) {
				delete [] grid_map[i];
			}

			delete [] grid_map;

		}

		// copy the new parameters
		width = w;
		width_2 = w*0.5;
		height = h;
		height_2 = h*0.5;
		size = h*w;
		resolution = res;
		inverse_resolution = 1.0/res;
		diagonal_resolution = res * std::sqrt(2.0);
		origin = _origin;

		// allocate the row pointers
		grid_map = new GridMapCellPtr[height];
		for (unsigned int i = 0; i < height; ++i) {

			grid_map[i] = new GridMapCell[width];

		}

		// restart the voronoi diagram
		voronoi.Initialize(height, width);

	}

}

// verify if a given pose is a valid one
bool InternalGridMap::isSafePlace(const std::vector<astar::Circle> &body, double safety_factor) {

	// the row and column index
	int c, r;

	// the current circle
	astar::Circle &circle;

	// the obstacle distance
	double obstacle_distance;

	for (unsigned int i = 0; i < body.size(); ++i) {

		circle = body[i];

		c = std::floor((circle.position.x - origin.x) * inverse_resolution);
		r = std::floor((circle.position.y - origin.y) * inverse_resolution);

		if (0 > r || height <= r || 0 > c || width <= c) {
			return false;
		}

		// get the closest obstacle from the voronoi distance map
		obstacle_distance = voronoi.GetObstacleDistance(c, r);

		if (obstacle_distance < circle.r * safety_factor) {
			return false;
		}

	}

	return true;

}

// occupy a given cell
void InternalGridMap::OccupyCell(int row, int col) {

	// occupy the given cell
	grid_map[row][col].occupancy = 1.0;

	// set the given cell as an obstacle
	voronoi.SetObstacle(row, col);

}

// clear a given cell
void InternalGridMap::ClearCell(int row, int col) {

	// clear the given cell
	grid_map[row][col].occupancy = 0.0;

	// clear the given obstacle
	voronoi.RemoveObstacle(row, col);

}

// update the internal grid map
void InternalGridMap::UpdateGridMap() {

	voronoi.Update();

}
