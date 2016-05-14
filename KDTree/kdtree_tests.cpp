#include <iostream>
#include <random>

#include "KDTree.hpp"

int main(int argc, char **argv) {

    // random generator
    std::default_random_engine generator;

    // random distribution
    std::uniform_real_distribution<double> distribution(-10, 10);


    // build a list of points
    std::vector<astar::Point<double, 3>> list;

    std::cout << std::endl << "Building the arrays" << std::endl;
    for (unsigned int i = 0; i < 100000; i++) {

        list.push_back(astar::Point<double, 3>({distribution(generator), distribution(generator), distribution(generator)}));

    }

    // build a sample best array:
    astar::Point<double, 3> find;

    find[0] = distribution(generator);
    find[1] = distribution(generator);
    find[2] = distribution(generator);

    std::cout << std::endl << "Built" << std::endl;
    // build a kdtree
    astar::KDTree<double, 3> kdtree(list);

    std::cout << "The given array: [" << find[0] << ", " << find[1] << ", " << find[2] << "]" << std::endl;
    std::cout << std::endl << "Trying to find the given array...." << std::endl;

    astar::Point<double, 3> found = kdtree.nearest(find);
    std::cout << std::endl << "Found: [" << found[0] << ", " << found[1] << ", " << found[2] << "]" << std::endl;

    return 0;

}

