#include <iostream>
#include <random>
#include <climits>

#include "KDTree.hpp"


int main(int argc, char **argv) {

    // random generator
    std::default_random_engine generator;

    const unsigned int dim = 3;
    // random distribution
    std::uniform_real_distribution<double> distribution(-10, 10);

    // build a list of points
    // std::vector<astar::PointT<double, 3>> list;
    astar::KDTree<double, dim> kdtree(std::numeric_limits<double>::max());

    std::cout << std::endl << "Building the arrays" << std::endl;
    for (unsigned int i = 0; i < 99999; i++) {

    	astar::PointT<double, dim> p({distribution(generator), distribution(generator), distribution(generator)});

    	kdtree.Insert(p);
    	kdtree.Insert(p);
    	kdtree.Insert(p);
    	kdtree.Insert(p);
        // list.push_back(astar::PointT<double, 3>());

    }


    // push a known point to the list
    astar::PointT<double, dim> known({185.89, 234.55, 134.2});

    // list.push_back(known);
    kdtree.Insert(known);

    // build a sample best array:
    astar::PointT<double, dim> find;

    find[0] = distribution(generator);
    find[1] = distribution(generator);
    find[2] = distribution(generator);

    std::cout << std::endl << "Built" << std::endl;
    // build a kdtree
    // astar::KDTree<double, 3> kdtree(list, std::numeric_limits<double>::max());

    std::cout << "The given array: [" << find[0] << ", " << find[1] << ", " << find[2] << "]" << std::endl;
    std::cout << std::endl << "Trying to find the given array...." << std::endl;

    //astar::PointT<double, 3> found = kdtree.Nearest(find);
    //std::cout << std::endl << "Found: [" << found[0] << ", " << found[1] << ", " << found[2] << "]\n" << std::endl;

    std::cout << "\nThe known array: [" << known[0] << ", " << known[1] << ", " << known[2] << "]" << std::endl;
    std::cout << std::endl << "Trying to find the given array...." << std::endl;

    astar::PointT<double, 3> found = kdtree.Nearest(known);
    std::cout << std::endl << "Found: [" << found[0] << ", " << found[1] << ", " << found[2] << "]" << std::endl;

    return 0;

}

