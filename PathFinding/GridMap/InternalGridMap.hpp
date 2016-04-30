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
        double inverseResolution;

        // the grid width
        unsigned int width;

        // the grid height
        unsigned int height;

        // the map origin
        astar::Vector2D origin;


        // the voronoy fiedl
        astar::GVDLau voronoyField;

        // PRIVATE METHODS

    public:

        // PUBLIC ATTRIBUTES


        // PUBLIC METHODS

        // basic constructor
        InternalGridMap();

        // update the GridMap
        void updateGridMap();

        // find a cell which a given pose is localized
        MapCellPtr poseToCell(const astar::Pose2D&);

        // get the distance to the nearest obstacle
        double getObstacleDistance(const astar::Vector2D&);

        // get the distance to the nearest voronoy edge
        double getVoronoiDistance(const astar::Vector2D&);

        // is a valid point?
        bool isValidPoint(const astar::Vector2D&);

        // is a safe place?
        bool isSafePlace(const astar::Pose2D&);
};

// just another helper
typedef InternalGridMap* InternalGridMapPtr;

}

#endif
