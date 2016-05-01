#ifndef INTERNAL_GRID_MAP_HPP
#define INTERNAL_GRID_MAP_HPP

#include "../GVDLau.hpp"
#include "../../Entities/Pose2D.hpp"
#include "MapCell.hpp"

namespace astar {

class InternalGridMap {

    private:

        // PRIVATE ATTRIBUTES

        // the grid resolution
        double resolution;

        // the inverse grid resolution
        double inverse_resolution;

        // the grid width
        unsigned int width;

        // the grid height
        unsigned int height;

        // the map origin
        astar::Vector2D origin;


        // the voronoy fiedl
        astar::GVDLau voronoy_field;

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES


        // PUBLIC METHODS

        // basic constructor
        InternalGridMap();

        // update the GridMap
        void UpdateGridMap();

        // find a cell which a given pose is localized
        MapCellPtr PoseToCell(const astar::Pose2D&);

        // get the distance to the nearest obstacle
        double GetObstacleDistance(const astar::Vector2D&);

        // get the distance to the nearest voronoy edge
        double GetVoronoiDistance(const astar::Vector2D&);

        // is a valid point?
        bool isValidPoint(const astar::Vector2D&);

        // is a safe place?
        bool isSafePlace(const astar::Pose2D&);
};

// just another helper
typedef InternalGridMap* InternalGridMapPtr;

}

#endif
