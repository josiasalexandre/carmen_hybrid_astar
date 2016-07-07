#include "InternalGridMap.hpp"

using namespace astar;

// basic constructor
InternalGridMap::InternalGridMap() :
    resolution(),
    inverse_resolution(),
    width(),
    width_2(),
    height(),
    height_2(),
    size(),
    origin(),
    orientation(0),
    grid_map(nullptr),
    voronoy_field(nullptr) {}


bool InternalGridMap::InitializeGridMap(
        unsigned int w,
        unsigned int h,
        double res,
        const astar::Vector2D<double> &_origin,
        double _orientation) {

    if (w != width || h != height || res != resolution || origin != _origin || orientation != _orientation) {

        // update the current parameters
        width = w;
        width_2 = 0.5*w;
        height = h;
        height_2 = 0.5*h;
        size = w*h;
        resolution = res;
        inverse_resolution = 1.0/res;
        origin = _origin;
        orientation = _orientation;

        if (nullptr != grid_map) {

            // remove the entire grid map
            delete [] grid_map;

        }

        // reallocate the entire grid map
        grid_map = new GridMapCell[size];

        return true;
    }

    return false;
}

// initialize the entire grid map
void InternalGridMap::Update() {



}

// occupy a given cell
void InternalGridMap::OccupyCell(astar::GridMapCellPtr c);

// clear a given cell
void InternalGridMap::ClearCell(astar::GridMapCellPtr c);

// find a cell which a given pose is localized
GridMapCellPtr InternalGridMap::PoseToCell(const astar::Pose2D&) {}

// get the distance to the nearest obstacle
double InternalGridMap::GetObstacleDistance(const astar::Vector2D<double>&);

// get the distance to the nearest voronoy edge
double InternalGridMap::GetVoronoiDistance(const astar::Vector2D<double>&);

// is a valid point?
bool InternalGridMap::isValidPoint(const astar::Vector2D<double>&);

// is a safe place?
bool InternalGridMap::isSafePlace(const astar::Pose2D&);

// is a safe place?
bool InternalGridMap::isSafePlace(const astar::Vector2D<double> &position, double orientation);
