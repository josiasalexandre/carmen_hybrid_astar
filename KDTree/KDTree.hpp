#ifndef KD_TREE_TEMPLATE_HPP
#define KD_TREE_TEMPLATE_HPP

#include <iostream>
#include <vector>
#include <limits>

#include "KDTreeNode.hpp"

namespace astar {

template<typename T, unsigned int D>
class KDTree {

private:

    // private members
    // the root pointer
    astar::KDTreeNode<T, D>* root;

    // the min distance
    double min_distance;

    // the infinity value, just a null value
    astar::PointT<T, D> null_value;

    // private methods
    // quick select approach
    unsigned int QuickSelect(std::vector<astar::PointT<T, D>>& input, unsigned int start, unsigned int end, unsigned int nth, unsigned int axis)
    {
        // special case
        if (end == start + 1)
            return start;

        T pivot;
        unsigned int store;

        // find the nth element
        while(true)
        {
            pivot = input[nth][axis];
            std::swap(input[nth], input[end - 1]);
            for (unsigned int i = store = start; i < end; i++)
            {
                if (input[i][axis] < pivot)
                {
                    if (i != store)
                        std::swap(input[i], input[store]);
                    store += 1;
                }
            }

            std::swap(input[store], input[end - 1]);
            if (input[store][axis] == input[nth][axis])
                return nth;

            if (store > nth)
                end = store;
            else
                start = store;
        }
    }

    // build the kdtree from a given array
    astar::KDTreeNode<T, D>* BuildKDTree(std::vector<astar::PointT<T, D>>& input, unsigned int start, unsigned int end, unsigned int axis)
    {
        if (start < end)
        {
            unsigned int mid = QuickSelect(input, start, end, (start + end)/2, axis);

            astar::KDTreeNode<T, D>* n = new astar::KDTreeNode<T, D>(input[mid]);

            axis += 1;
            if (D <= axis)
                axis = 0;

            // build the left subtree
            n->left = BuildKDTree(input, start, mid, axis);

            // build the right subtree
            n->right = BuildKDTree(input, mid + 1, end, axis);

            // return the valid node
            return n;


        }

        return nullptr;

    }

    // compare two points
    bool DiffPoints(astar::PointT<T, D>& a, astar::PointT<T, D>&b)
    {
        // compare
        for (unsigned int i = 0; i < D; i++)
        {
            if (a[i] != b[i])
                return true;
        }

        return false;
    }

    // destroy a given kdtree
    void RemoveSubTree(astar::KDTreeNode<T, D>* n)
    {
        if (nullptr != n->left)
            RemoveSubTree(n->left);

        if (nullptr != n->right)
            RemoveSubTree(n->right);

        // finally, remove the current node
        delete n;
    }

    // distance between two nodes
    inline double Distance(astar::KDTreeNode<T, D>* a, astar::KDTreeNode<T, D>* b)
    {
        double t, d = 0.0;

        for (unsigned int axis = 0; axis < D; axis++)
        {
            t = a->point[axis] - b->point[axis];
            d += t*t;
        }

        return d;
    }

    // distance betee two points
    inline double Distance(const astar::PointT<T, D> &a, const astar::PointT<T, D> &b)
    {
        double t, d = 0.0;

        for (unsigned int axis = 0; axis < D; axis++)
        {
            t = (double) a[axis] - (double) b[axis];
            d += t*t;
        }

        return d;
    }

    // find the nearest point inside the kdtree
    astar::KDTreeNode<T, D>* GetNearest(
    		astar::KDTreeNode<T, D>* n, const astar::PointT<T, D>& p,
			unsigned int axis, astar::KDTreeNode<T, D>* best_node, double &best_dist)
    {
        double dist, deltaAxis, deltaAxis2;

        dist = Distance(n->point, p);
        if (min_distance > dist) {

        	best_dist = dist;
        	best_node = n;
            return n;
        }

        deltaAxis = (double) n->point[axis] - (double) p[axis];
        deltaAxis2 = deltaAxis*deltaAxis;

        // is the current distance closer than the current bestDist?
        if (nullptr == best_node || dist < std::fabs(best_dist))
        {
            best_dist = dist;
            best_node = n;
        }

        axis += 1;
        if (D <= axis)
            axis = 0;

        // should we search the left or right subtree?
        // should we search the left or right subtree?
        if (0 < deltaAxis && nullptr != n->left)
            best_node = GetNearest(n->left, p, axis, best_node, best_dist);
        else if (nullptr != n->right)
            best_node = GetNearest(n->right, p, axis, best_node, best_dist);

        // are we done?
        if (deltaAxis2 >= best_dist)
            return best_node;

        // search the other side
        if (0 < deltaAxis && nullptr != n->right)
            best_node = GetNearest(n->right, p, axis, best_node, best_dist);
        else if (nullptr != n->left)
            best_node = GetNearest(n->left, p, axis, best_node, best_dist);

        // std::cout << "==============================Min: " << dist << "\n";
        return best_node;
    }

public:

    // empty constructor
    KDTree(T _null) : root(nullptr), min_distance(0.005), null_value() {

    	for (unsigned int i = 0; i < D; ++i) {
			null_value[i] = _null;
		}
    }

    // basic constructor
    KDTree(std::vector<astar::PointT<T, D>> &input, T _null) : root(nullptr), min_distance(0.005), null_value()
    {
        // build a KDTree from the a given array
        root = BuildKDTree(input, 0, input.size(), 0);

        for (unsigned int i = 0; i < D; ++i) {
        	null_value[i] = _null;
        }
    }

    // basic destructor
    ~KDTree()
    {
        if (nullptr != root)
            RemoveSubTree(root);
    }

    // is the current KDTree empty?
    bool isEmpty()
    {
    	return nullptr == root;
    }

    // add a point to the kdtree
    void Insert(astar::PointT<T, D> &p)
    {
        if (nullptr != root)
        {
            astar::KDTreeNode<T, D> *tmp = root;
            unsigned int axis = 0;

            while (nullptr != tmp)
            {
                if (tmp->point[axis] > p[axis])
                {
                    // verify if there is a left subtree
                    if (nullptr != tmp->left)
                    {
                        // update the tmp pointer
                        tmp = tmp->left;
                    }
                    else
                    {
                        // add the new point to the left
                        tmp->left = new astar::KDTreeNode<T, D>(p);

                        // exit the while loop
                        return;

                    }

                }
                else if (tmp->point[axis] < p[axis] || DiffPoints(tmp->point, p))
                {
                    // verify if there is a right subtree
                    if (nullptr != tmp->right)
                    {
                        // update the tmp pointer
                        tmp = tmp->right;

                    }
                    else
                    {
                        // add the new point to the right
                        tmp->right = new astar::KDTreeNode<T, D>(p);

                        // exit the while loop
                        return;
                    }

                }
                else
                {
                    // same point
                    return;

                }
            }
        }
        else
        {
            root = new astar::KDTreeNode<T, D>(p);
        }

    }

    // add a list of pointers
    void InsertPoints(std::vector<astar::PointT<T, D>> &input)
    {
        // is it an empty kdtree?
        if (nullptr == root)
        {
            root = BuildKDTree(input, 0, input.size(), 0);
        }

        unsigned int v_size = input.size();

        // append each element
        for (unsigned int i = 0; i < v_size; i++)
        {
            Insert(input[i]);
        }
    }

    // rebuild the entire tree with a given set of points
    void RebuildKDTree(std::vector<astar::PointT<T, D>> &input)
    {
        if (nullptr != root)
        {
            // clear the entire KDTree
            RemoveSubTree(root);

        }

        // build a KDTree from the given array
        root = BuildKDTree(input, 0, input.size(), 0);

    }

    // find the nearest neighbour
    astar::PointT<T, D> Nearest(const astar::PointT<T, D>& p)
    {
        // is it a valid kdtree?
        if (nullptr != root)
        {
            double best_dist = -std::numeric_limits<double>::max();

            astar::KDTreeNode<T, D> *best_node = GetNearest(root, p, 0, nullptr, best_dist);

            // return the point
            return best_node->point;

        }

        return null_value;
    }

    // remove a point from the kdtree
    void Remove(const astar::PointT<T, D>& p)
    {
        /* TODO */
    }

    // find the k nearest points
    std::vector<astar::PointT<T, D>> Nearests(const astar::PointT<T, D>& p, unsigned int k)
    {
        /* TODO */
    	return std::vector<astar::PointT<T, D>>();
    }

    // balance the KDTree
    void Balance()
    {
        /* TODO */
        return;
    }

    // clear the kdtree
    void Clear()
    {

        if (nullptr != root)
        {
            // clear the entire tree
            RemoveSubTree(root);

        }
    }

};


}

#endif
