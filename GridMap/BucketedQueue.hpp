/*
 * Based on http://www.first-mm.eu/files/lau10iros.pdf
 */

#ifndef BUCKETED_PRIORITY_QUEUE_HPP
#define BUCKETED_PRIORITY_QUEUE_HPP

#include <vector>
#include <set>
#include <queue>
#include <assert.h>
#include <map>

#include "../Entities/IntPoint2D.hpp"

namespace astar {

template <typename T>
class BucketedQueue {

    private:

        // PRIVATE ATTRIBUTES

        // how many elements
        int count;

        // define a bucket map
        typedef std::map<int, std::queue<T> > BucketType;

        // the internal buckets
        BucketType buckets;

        // define a custom iterator name
        typename BucketType::iterator nextPop;

    public:

        // PUBLIC ATTRIBUTES

        // PUBLIC METHODS

        // basic constructor
        BucketedQueue() {
            Clear();
        }

        // delete the internal buckets
        void Clear() {
            buckets.clear();
            count = 0;
            nextPop = buckets.end();
        }

        //! Checks whether the Queue is empty
        bool Empty() { return (0 == count); }

        //! push an element
        void Push(int prio, T t) {
            buckets[prio].push(t);
            if (nextPop == buckets.end() || prio < nextPop->first) nextPop = buckets.find(prio);
            count++;
        }
        //! return and pop the element with the lowest squared distance */
        T Pop() {

            while (nextPop!=buckets.end() && nextPop->second.empty()) ++nextPop;

            T p = nextPop->second.front();
            nextPop->second.pop();

            if (nextPop->second.empty()) {
                typename BucketType::iterator it = nextPop;
                nextPop++;
                buckets.erase(it);
            }

            count--;

            return p;

        }

        int Size() { return count; }
        int GetNumBuckets() { return buckets.size(); }

        int GetTopPriority(){
            return nextPop->first;
        }

};

}

#endif
