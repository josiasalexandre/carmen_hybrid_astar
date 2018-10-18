#include <iostream>
#include <cmath>

#include "InternalGridMap.hpp"

using namespace astar;

InternalGridMap::InternalGridMap() :
    width(0), width_2(0),
    height(0), height_2(0),
    size(0),
    diagonal_resolution(0),
    origin(),
    grid_map(nullptr),
    has_changed(false),
    corridor(0),
    voronoi(), resolution(0.0), inverse_resolution(0.0)
{}

// basic destructor
InternalGridMap::~InternalGridMap()
{
    // Remove the current grid
    RemoveGridMap();
}

// return the has changed flag
bool InternalGridMap::HasChanged() const
{
    return has_changed;
}

// get the grid cell index from any position
astar::GridCellIndex InternalGridMap::PoseToIndex(const astar::Vector2D<double> &position) const
{
    GridCellIndex index(
        std::floor((position.y - origin.y) * inverse_resolution + 0.5),
        std::floor((position.x - origin.x) * inverse_resolution + 0.5)
    );

    return index;
}

// remove the current grid map
void InternalGridMap::RemoveGridMap()
{
    // the GridMapCell map
    if (nullptr != grid_map)
    {
        for (unsigned int i = 0; i < height; ++i)
        {
           delete [] grid_map[i];
        }
        delete [] grid_map;
    }
}

// process the voronoi diagram
void InternalGridMap::ProcessVoronoiDiagram()
{
    has_changed = voronoi.Update();
}

// get the map parameters and allocate the grid map in the memmory
void InternalGridMap::UpdateGridMap(unsigned int h, unsigned int w, double res, const astar::Vector2D<double> &_origin, double *map)
{
    has_changed = false;

    if (w != width || h != height || res != resolution)
    {
        has_changed = true;

        // Remove the current grid map
        RemoveGridMap();

        // copy the new parameters
        height = h;
        height_2 = h * 0.5;
        width = w;
        width_2 = w * 0.5;
        size = h * w;
        resolution = res;
        inverse_resolution = 1.0/res;
        diagonal_resolution = res * std::sqrt(2.0);
        origin = _origin;

        // allocate the row pointers
        grid_map = new GridMapCellPtr[height];
        for (unsigned int i = 0; i < height; ++i)
        {
            grid_map[i] = new GridMapCell[width];
        }

        // restart the voronoi diagram
        voronoi.InitializeEmpty(height, width);
    }
    else if (origin != _origin)
    {
        has_changed = true;

        // update the origin
        origin = _origin;
    }

    unsigned int k = 0;

    for (unsigned int col = 0; col < width; ++col)
    {
        for (unsigned int row = 0; row < height; ++row)
        {
            if (0.4 < map[k])
            {
                grid_map[row][col].occupancy = 1.0;
            }
            else if (0 > map[k])
            {
                grid_map[row][col].occupancy = -1.0;
            }
            else
            {
                grid_map[row][col].occupancy = 0.0;
            }

            grid_map[row][col].is_corridor = false;

            ++k;
        }
    }

    // restart the voronoi diagram
    // voronoi.RestartVoronoiDiagram();
}

// expand a circular region around the RDDF point
void InternalGridMap::ExpandRegion(GridCellIndex &index, unsigned int distance)
{
    // the initial vector
    std::vector<GridCellIndex> indexes;
    indexes.push_back(index);

    unsigned int row, nrow, ncol, col, drow2, dcol2;

    int drow, dcol;

    int index_row = (int) index.row;
    int index_col = (int) index.col;

    // helper
    GridCellIndex tmp;

    for (unsigned int i = 0; i < indexes.size(); ++i)
    {
        col = indexes[i].col;
        row = indexes[i].row;

        // top
        nrow = row + 1;
        if (height > nrow)
        {
            drow = ((int) nrow) - index_row;
            drow2 = (unsigned int) (drow * drow);
            dcol = ((int) col) - index_col;
            dcol2 = (unsigned int) (dcol*dcol);

            // top
            if (distance > (drow2 + dcol2) && !grid_map[nrow][col].is_corridor)
            {
                tmp.row = nrow;
                tmp.col = col;

                grid_map[nrow][col].is_corridor = true;

                indexes.push_back(tmp);

                // update the corridor indexes counter
                corridor += 1;
            }
        }

        // bottom
        nrow = row - 1;
        if (height > nrow)
        {
            drow = ((int) nrow) - index_row;
            drow2 = (unsigned int) (drow * drow);
            dcol = ((int) col) - index_col;
            dcol2 = (unsigned int) (dcol*dcol);

            // top
            if (distance > (drow2 + dcol2) && !grid_map[nrow][col].is_corridor)
            {
                tmp.row = nrow;
                tmp.col = col;

                grid_map[nrow][col].is_corridor = true;

                indexes.push_back(tmp);

                // update the corridor indexes counter
                corridor += 1;
            }
        }

        // left
        ncol = col - 1;
        if (width > ncol)
        {
            drow = ((int) row) - index_row;
            drow2 = (unsigned int) (drow * drow);
            dcol = ((int) ncol) - index_col;
            dcol2 = (unsigned int) (dcol*dcol);

            if (distance > (drow2 + dcol2) && !grid_map[row][ncol].is_corridor)
            {
                tmp.row = row;
                tmp.col = ncol;

                grid_map[row][ncol].is_corridor = true;

                indexes.push_back(tmp);

                // update the corridor indexes counter
                corridor += 1;
            }
        }

        // right
        ncol = col + 1;
        if (width > ncol)
        {
            drow = ((int) row) - index_row;
            drow2 = (unsigned int) (drow * drow);
            dcol = ((int) ncol) - index_col;
            dcol2 = (unsigned int) (dcol*dcol);

            if (distance > (drow2 + dcol2)  && !grid_map[row][ncol].is_corridor)
            {
                tmp.row = row;
                tmp.col = ncol;

                grid_map[row][ncol].is_corridor = true;

                indexes.push_back(tmp);

                // update the corridor indexes counter
                corridor += 1;
            }
        }
    }
}

// update the corridor map given a new rddf
void InternalGridMap::UpdateCorridor(const std::vector<astar::Vector2D<double>> &rddf, unsigned int distance)
{
    // update the current corridor
    // get the rddf size
    int r_size = rddf.size();

    corridor = 0;

    if (0 < r_size)
    {
        GridCellIndex index;

        for (int i = r_size - 1; i >= 0; --i)
        {
            // the index
            index = PoseToIndex(rddf[i]);

            // get the cell reference
            GridMapCellPtr c(IndexToCell(index));

            if (nullptr != c)
            {
                // set as a corridor cell
                c->is_corridor = true;

                // expand!
                ExpandRegion(index, distance);
            }
        }
    }
}

// get the current corridor
unsigned int InternalGridMap::GetCorridorIndexes()
{
    return corridor;
}

// is a valid cell?
bool InternalGridMap::isValidPoint(const astar::Vector2D<double> &position)
{
    // get the grid cell
    GridCellIndex index(PoseToIndex(position));

    return (height > index.row && width > index.col);
}

// verify if a given pose is a valid one
bool InternalGridMap::isSafePlace(const std::vector<astar::Circle> &body, double safety_factor)
{
    // the row and column index
    unsigned int c, r;

    // the obstacle distance
    double obstacle_distance;

    for (unsigned int i = 0; i < body.size(); ++i)
    {
        // the current circle
        const astar::Circle &circle(body[i]);

        GridCellIndex index(PoseToIndex(circle.position));

        if (height > index.row && width > index.col)
        {
            // get the closest obstacle from the voronoi distance map
            obstacle_distance = voronoi.GetObstacleDistance(index.row, index.col) * resolution;

            if (obstacle_distance < circle.r * safety_factor)
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    return true;
}

// return a cell given a pose
GridMapCellPtr InternalGridMap::PoseToCell(const astar::Pose2D &p)
{
    // get the grid Cell
    GridCellIndex index(PoseToIndex(p.position));

    if (height > index.row && width > index.col)
        return &grid_map[index.row][index.col];
    else
        return nullptr;
}

// get the cell given a cell index
GridMapCellPtr InternalGridMap::IndexToCell(const astar::GridCellIndex &index)
{
    if (height > index.row && width > index.col)
        return &grid_map[index.row][index.col];
    else
        return nullptr;
}

// occupy a given cell
void InternalGridMap::SetSimpleObstacle(int row, int col)
{
    // occupy the given cell
    // grid_map[row][col].occupancy = 1.0;

    // set the obstacle in the voronoi diagram
    voronoi.SetSimpleObstacle(row, col);
}

// occupy a given cell
void InternalGridMap::SetSimpleFreeSpace(int row, int col)
{
    // occupy the given cell
    // grid_map[row][col].occupancy = 0.0;

    // set the obstacle in the voronoi diagram
    voronoi.SetSimpleFreeSpace(row, col);
}

// occupy a given cell
void InternalGridMap::OccupyCell(int row, int col)
{
    // set the obstacle in the voronoi diagram
    voronoi.SetObstacle(row, col);

    if (1.0 != grid_map[row][col].occupancy)
    {
        // occupy the given cell
        grid_map[row][col].occupancy = 1.0;
    }
}

// clear a given cell
void InternalGridMap::ClearCell(int row, int col)
{
    // remove the obstacle in the voronoi diagram
    voronoi.RemoveObstacle(row, col);

    if (0.0 != grid_map[row][col].occupancy)
    {
        // clear the given cell
        grid_map[row][col].occupancy = 0.0;

    }
}

// get the current grid map
GVDLau* InternalGridMap::GetGVD()
{
    return &voronoi;
}

// get the nearest obstacle position
Vector2D<double> InternalGridMap::GetObstaclePosition(const astar::Vector2D<double> &position)
{
    // get the grid cell index
    GridCellIndex index(PoseToIndex(position));

    if (height > index.row && width > index.col)
    {
        // get the nearest obstacle index
        GridCellIndex obstacle(voronoi.GetObstacleIndex(index.row, index.col));

        // get the position
        double x = origin.x + ((double) obstacle.col) * resolution;
        double y = origin.y + ((double) obstacle.row) * resolution;

        return Vector2D<double>(x, y);
    }

    return Vector2D<double>(position);
}

// indirect obstacle distance
double InternalGridMap::GetObstacleDistance(const Vector2D<double> &position)
{
    // get the grid cell index
    GridCellIndex index(PoseToIndex(position));

    if (height > index.row && width > index.col)
    {
        // get the obstacle distance
        return voronoi.GetObstacleDistance(index.row, index.col) * resolution;
    }

    return 0.0;
}

// get the nearest voronoi edge position
astar::Vector2D<double> InternalGridMap::GetVoronoiPosition(const astar::Vector2D<double> &position)
{
    // get the grid cell index
    GridCellIndex index(PoseToIndex(position));

    if (height > index.row && width > index.col)
    {
        // get the voronoi cell index
        GridCellIndex voro(voronoi.GetVoronoiIndex(index.row, index. col));

        // get the current displacement
        double x = origin.x + ((double) voro.col) * resolution;
        double y = origin.y + ((double) voro.row) * resolution;

        return Vector2D<double>(x, y);
    }

    return Vector2D<double>(std::numeric_limits<double>::max());
}

// indirect voronoi edge distance
double InternalGridMap::GetVoronoiDistance(const astar::Vector2D<double> &position)
{
    // get the grid cell index
    GridCellIndex index(PoseToIndex(position));

    if (height > index.row && width > index.col)
    {
        // get the voronoi cell index
        GridCellIndex voro(voronoi.GetVoronoiIndex(index.row, index.col));

        // get the current displacement
        double dx = position.x - (origin.x + ((double) voro.col)*resolution);
        double dy = position.y - (origin.y + ((double) voro.row)*resolution);

        return std::sqrt(dx*dx + dy*dy);
    }

    return std::numeric_limits<double>::max();
}

// compute the current path cost
double InternalGridMap::GetPathCost(const astar::Vector2D<double> &position)
{
    // get the grid cell index
    GridCellIndex index(PoseToIndex(position));

    if (height > index.row && width > index.col)
    {
        // get the current path cost
        return voronoi.GetPathCost(index.row, index.col);
    }

    return 1.0;
}

// get the current map
unsigned char* InternalGridMap::GetGridMap()
{
    // build a new array of chars
    unsigned char *map = new unsigned char[height*width];

    unsigned int k = 0;

    for (int row = height - 1; row >= 0; --row)
    {
        for (unsigned int col = 0; col < width; ++col)
        {
            unsigned char c = (unsigned char) (grid_map[(unsigned int) row][col].occupancy == 0 ? 255 : 0);

            map[k] = c;

            ++k;
        }
    }

    return map;

    for (int i = 0; i < width; ++i)
    {
        for (unsigned int j = 0; j < height; ++j)
        {
            unsigned char c = (unsigned char) (grid_map[(unsigned int) i][j].occupancy == 0 ? 255 : 0);

            map[k] = c;

            ++k;
        }
    }

    return map;
}

// get the current path cost map
unsigned char* InternalGridMap::GetPathCostMap()
{
    // build a new array of chars
    return voronoi.GetPathCostMap();
}

// get the current path cost map
unsigned char* InternalGridMap::GetObstacleDistanceMap()
{
    // build a new array of chars
    return voronoi.GetObstacleDistanceMap();
}

// get the current map
unsigned char* InternalGridMap::GetCorridorMap()
{
    // build a new array of chars
    unsigned char *map = new unsigned char[height*width];

    unsigned int k = 0;

    for (int i = ((int) width) - 1; i > -1; --i)
    {
        for (unsigned int j = 0; j < height; ++j)
        {
            unsigned char c = (unsigned char) (grid_map[(unsigned int) i][j].is_corridor ? 255 : 0);

            map[k] = c;

            ++k;
        }
    }

    return map;
}
