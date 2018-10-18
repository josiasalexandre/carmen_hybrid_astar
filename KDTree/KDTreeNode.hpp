#ifndef KD_TREE_NODE_TEMPLATE_HPP
#define KD_TREE_NODE_TEMPLATE_HPP

#include "../Entities/PointT.hpp"

namespace astar {

    template<typename T, unsigned int D>
    class KDTreeNode {

        private:

        public:

            // public members

            // the point stored
            astar::PointT<T, D> point;

            // the left pointer
            KDTreeNode *left;

            // the left pointer
            KDTreeNode *right;

            // basic construtor
            KDTreeNode (const astar::PointT<T, D>& p) : point(p), left(nullptr), right(nullptr) {}

            // basic destructor
            ~KDTreeNode()
            {
                // update the pointers
                left = right = nullptr;
            }
    };
}

#endif
