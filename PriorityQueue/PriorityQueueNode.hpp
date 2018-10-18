#ifndef FIBONACCI_HEAP_NODE_TEMPLATE_HPP
#define FIBONACCI_HEAP_NODE_TEMPLATE_HPP

namespace astar {

template<typename T>
class PriorityQueueNode {

    public:

        // it's all public
        // attributes

        // the parent node
        PriorityQueueNode<T>* parent;

        // the child node
        PriorityQueueNode<T>* child;

        // the left node
        PriorityQueueNode<T>* left;

        // the right node
        PriorityQueueNode<T>* right;

        // the degree
        unsigned int degree;

        // the mark
        bool mark;

        // the external object
        T element;

        // the key
        double Key;

        // basic constructor
        PriorityQueueNode(T e, double Key_) :
            parent(nullptr), child(nullptr), degree(0), mark(false), element(e), Key(Key_) {

            // set the left and right pointers
            left = right = this;
        }

        // copy constructor
        PriorityQueueNode(const PriorityQueueNode<T>& n) : parent(n.parent), child(n.child), left(n.left), right(n.right), degree(n.degree), mark(n.mark), element(n.element), Key(n.Key) {}
};

// syntatic sugar
template<typename T>
using PriorityQueueNodePtr = PriorityQueueNode<T>*;

}

#endif
