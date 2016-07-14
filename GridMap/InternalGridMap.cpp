#include "InternalGridMap.hpp"
#include <cmath>
using namespace astar;

InternalGridMap::InternalGridMap() :
	width(0), width_2(0),
	height(0), height_2(0),
	size(0),
	diagonal_resolution(0),
	origin(),
	grid_map(nullptr),
	voronoi(), resolution(0.0), inverse_resolution(0.0)
{
}

// basic destructor
InternalGridMap::~InternalGridMap() {

	// Remove the current grid
	RemoveGridMap();

}

// remove the current grid map
void InternalGridMap::RemoveGridMap() {

	if (nullptr != grid_map) {

		for (unsigned int i = 0; i < height; ++i) {
			delete [] grid_map[i];
		}

		delete [] grid_map;

	}

}

// get the map parameters and allocate the grid map in the memmory
void InternalGridMap::InitializeGridMap(unsigned int h, unsigned int w, double res, const astar::Vector2D<double> &_origin) {

	if (w != width || h != height || res != resolution ||  origin != _origin) {

		// Remove the current grid map
		RemoveGridMap();

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
		voronoi.InitializeEmpty(height, width);

	}

}

// is a valid cell?
bool InternalGridMap::isValidPoint(const astar::Vector2D<double> &position) {

	unsigned int row = std::floor((position.y - origin.y) * inverse_resolution);
	unsigned int col = std::floor((position.x - origin.x) * inverse_resolution);

	return (height > row && width > col);

}

// verify if a given pose is a valid one
bool InternalGridMap::isSafePlace(const std::vector<astar::Circle> &body, double safety_factor) {

	// the row and column index
	unsigned int c, r;

	// the obstacle distance
	double obstacle_distance;

	for (unsigned int i = 0; i < body.size(); ++i) {

		// the current circle
		const astar::Circle &circle(body[i]);

		c = std::floor((circle.position.x - origin.x) * inverse_resolution);
		r = std::floor((circle.position.y - origin.y) * inverse_resolution);

		if (height <= r || width <= c) {
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

// return a cell given a pose
GridMapCellPtr InternalGridMap::PoseToCell(const astar::Pose2D &p) {

	unsigned int row = std::floor((p.position.y - origin.y) * inverse_resolution);
	unsigned int col = std::floor((p.position.x - origin.x) * inverse_resolution);

	if (height > row && width > col)
		return &grid_map[row][col];
	else
		return nullptr;

}

// occupy a given cell
void InternalGridMap::OccupyCell(int row, int col) {

	if (1.0 != grid_map[row][col].occupancy) {

		// occupy the given cell
		grid_map[row][col].occupancy = 1.0;

		// set the given cell as an obstacle
		voronoi.SetObstacle(row, col);

	}

}

// clear a given cell
void InternalGridMap::ClearCell(int row, int col) {

	if (0.0 != grid_map[row][col].occupancy) {

		// clear the given cell
		grid_map[row][col].occupancy = 0.0;

		// clear the given obstacle
		voronoi.RemoveObstacle(row, col);

	}

}

// update the internal grid map
void InternalGridMap::UpdateGridMap() {

	voronoi.Update();

}

// get the current grid map
astar::GVDLau* InternalGridMap::GetGVD() {

	return &voronoi;

}

// indirect obstacle distance
double InternalGridMap::GetObstacleDistance(const astar::Vector2D<double> &position) {

	unsigned int row = std::floor((position.y - origin.y) * inverse_resolution);
	unsigned int col = std::floor((position.x - origin.x) * inverse_resolution);

	return voronoi.GetObstacleDistance(row, col);
}

// indirect voronoi edge distance
double InternalGridMap::GetVoronoiDistance(const astar::Vector2D<double> &position) {

	unsigned int row = std::floor((position.y - origin.y) * inverse_resolution);
	unsigned int col = std::floor((position.x - origin.x) * inverse_resolution);

	return voronoi.GetVoronoiDistance(row, col);
}
