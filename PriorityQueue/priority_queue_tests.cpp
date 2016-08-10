#include <iostream>

#include <vector>
#include <random>
#include <cmath>
#include <climits>

#include "PriorityQueue.hpp"

int main() {

    // the default vector
    std::vector<int> numbers;
    std::vector<astar::PriorityQueueNodePtr<int>> handles;

    // fill the vector
    std::cout << std::endl << "Filling the vector: [";
    for (unsigned int i = 0; i < 200000; i++) {

        unsigned int n = (unsigned int) abs(rand()%2000000);
        std::cout << " " << n;
        numbers.push_back(n);

    }
    std::cout << "]" << std::endl;

    // build a PriorityQueue object
    astar::PriorityQueue<int> priority_queue(0);

    std::cout << std::endl << "Pushing to the PriorityQueue..." << std::endl;
    for (unsigned int i = 0; i < 200000; i++) {

        priority_queue.Add(numbers[i], numbers[i]);

    }
    std::cout << std::endl << "Lets see the priority queue: " << priority_queue.GetN() << std::endl;
    std::cout << std::endl << "Lets remove each element from the Priority Queue:" << std::endl;
    int i = 1;
    while(!priority_queue.isEmpty()) {

        priority_queue.DeleteMin();

        i += 1;

        std::cout << " " << i;

    }

    // return 2;

    std::cout << std::endl << "Lets see the priority queue: " << priority_queue.GetN() << std::endl;
    std::cout << std::endl << "Let's back the numbers to the PriorityQueue..." << std::endl;
    for (unsigned int i = 0; i < 200000; i++) {

        handles.push_back(priority_queue.Add(numbers[i], numbers[i]));

    }

    std::cout << std::endl << "Lets see the priority queue: " << priority_queue.GetN() << std::endl;
    std::cout << std::endl << "The min element: " << priority_queue.Min() << std::endl;

    for (unsigned int i= 0 ; i < 2000; i++) {

        // get a random index
        unsigned int index = rand()%20000;
        if (numbers[index] != UINT_MAX) {
            std::cout << "Let's try to decrease the key to the " << index + 1  << "º element: " << numbers[index] << std::endl;
            priority_queue.DecreaseKey(handles[index], -1);
            numbers[index] = UINT_MAX;

            std::cout << "Let's see the new min element: " << priority_queue.DeleteMin() << std::endl;

        }


    }

    std::cout << std::endl << "Lets see the priority queue: " << priority_queue.GetN() << std::endl;
    std::cout << std::endl << "Let's destroy the entire heap: "<< std::endl;
    priority_queue.ClearHeap();
    std::cout << std::endl << "Destroyed!" << std::endl;
    std::cout << std::endl << "Lets see the priority queue: " << priority_queue.GetN() << std::endl;

    std::cout << "Hello, world!" << std::endl;
   return 0;

}
