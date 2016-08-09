#ifndef HYBRID_ASTAR_HOLOMIC_HEURISTIC_INFO_HPP
#define HYBRID_ASTAR_HOLOMIC_HEURISTIC_INFO_HPP

#include <queue>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../../../GridMap/InternalGridMap.hpp"
#include "../../../Entities/Pose2D.hpp"
#include "../../../Entities/Circle.hpp"
#include "../../../PriorityQueue/PriorityQueue.hpp"

namespace astar {

class HolonomicHeuristic {

    private:

        // the current grid map reference pointer
        astar::InternalGridMapRef grid;

        // the current start pose
        astar::Pose2D start;

        // the current goal pose
        astar::Pose2D goal;

        // the complete circle
        double twopi;

        // the step angle
        double piece_angle;

        // a helper circle node
        class CircleNode {

            public:

                // the current circle reference
                astar::Circle circle;

                // the g cost value
                double g;

                // the total cost, included heuristic value
                double f;

                // the circle radius
                double radius;

                // the parent node
                CircleNode *parent;

                bool explored;

                // basic constructor
                CircleNode(const astar::Circle c, double r, double g_, double f_, CircleNode *p) :
                    circle(c), g(g_), f(f_), radius(r), parent(p), explored(false) {}

                // the copy constructor
                CircleNode(const CircleNode& cn) :
                    circle(cn.circle), g(cn.g), f(cn.f), radius(cn.circle.r), parent(nullptr), explored(cn.explored) {}

                // basic destructor
                ~CircleNode() {

                    // reset the parent node to nullptr
                    parent = nullptr;

                }

                // update the node values
                void UpdateValues(const CircleNode &cn) {

                    // copy the node values

                    // the circle position
                    circle.position = cn.circle.position;

                    // the circle radius
                    circle.r = cn.circle.r;

                    // the cost
                    g = cn.g;

                    // the heuristic cost
                    f = cn.f;

                    // the parent pointer
                    parent = cn.parent;

                }

                // < operator overloading, for priority queue compare purpose
                bool operator<(const CircleNode &cn) const {

                    return f < cn.f;

                }

                // Assignment operator
                void operator=(const CircleNode &cn) {

                    // copy the values
                    UpdateValues(cn);

                }

        };

        // define the syntatic sugar types
        typedef CircleNode* CircleNodePtr;
        typedef CircleNode& CircleNodeRef;

        // define a comparator class
        class CircleNodePtrDistanceComparator {

            public:

                // operator overloading
                bool operator() (CircleNodePtr a, CircleNodePtr b) {

                    // the default c++ stl is a max heap, so wee need to invert here
                    return a->f > b->f;

                }

        };

        // define a comparator class
        class CircleNodePtrRadiusComparator {

            public:

                // operator overloading
                bool operator() (CircleNodePtr a, CircleNodePtr b) {

                    // the default c++ stl is a max heap
                    return a->radius < b->radius;

                }

        };

        // define the CircleNodePtrArray
        class CircleNodePtrArray {

            public:

                // the vector of circle nodes
                std::vector<CircleNodePtr> circles;

        };

        // define the syntatic sugar type
        typedef CircleNodePtrArray* CircleNodePtrArrayPtr;
        typedef CircleNodePtrArray& CircleNodePtrArrayRef;

        // the resulting circle path
        CircleNodePtrArray circle_path;

        // the nearest open queue
        std::priority_queue<CircleNodePtr, std::vector<CircleNodePtr>, CircleNodePtrDistanceComparator> nearest_open;

        // the largest open queue
        std::priority_queue<CircleNodePtr, std::vector<CircleNodePtr>, CircleNodePtrRadiusComparator> largest_open;

        // the closed set
        std::vector<CircleNodePtr> closed;

        // THE HEURISTIC PRIVATE METHODS

        // verify if two circles overlaps with each other
        bool Overlap(const astar::Circle&, const astar::Circle&, double factor = 0.5);

        // get the nearest circle from a given pose
        CircleNodePtr NearestCircleNode(const astar::Pose2D&);

        // get the circle children
        CircleNodePtrArrayPtr GetChildren(CircleNodePtr);

        // clear the current CircleNodePtr sets
        void RemoveAllCircleNodes();

        // rebuild the circle array
        void RebuildCirclePath(CircleNodePtr cn, astar::CircleRef c_goal);

        // try to find a circle node inside the closed set
        bool NotExist(CircleNodePtr cn);

        // explore a given circle node
        void ExploreCircleNode(CircleNodePtr cn);

        // the current search method
        bool SpaceExploration();

        // show
        void ShowCirclePath();


    public:

        // THE HEURISTIC OBJECT METHODS

        // basic constructor
        HolonomicHeuristic(astar::InternalGridMapRef map);

        // update the Heuristic circle path with a new grid map, start and goal poses
        void UpdateHeuristic(astar::InternalGridMapRef, const astar::Pose2D&, const astar::Pose2D&);

        // get the heuristic value given a pose
        double GetHeuristicValue(const astar::Pose2D&);

};

}

#endif
