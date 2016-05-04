#ifndef FIBONACCI_HEAP_NODE_TEMPLATE_HPP
#define FIBONACCI_HEAP_NODE_TEMPLATE_HPP

namespace astar {

template<typename T>
class FibonacciHeapNode {

    public:

        // it's all public
        // attributes

        // the parent node
        FibonacciHeapNode<T>* parent;

        // the child node
        FibonacciHeapNode<T>* child;

        // the left node
        FibonacciHeapNode<T>* left;

        // the right node
        FibonacciHeapNode<T>* right;

        // the degree
        unsigned int degree;

        // the mark
        bool mark;

        // the external object
        T element;

        // the key
        double Key;

        // basic constructor
        FibonacciHeapNode(T e, double Key_) : parent(nullptr), child(nullptr), left(nullptr), right(nullptr), degree(0), mark(false), element(e), Key(Key_) {}

        // copy constructor
        FibonacciHeapNode(const FibonacciHeapNode<T>& n) : parent(n.parent), child(n.child), left(n.left), right(n.right), degree(n.degree), mark(n.mark), element(n.element), Key(n.Key) {}

        // basic destructor
        ~FibonacciHeapNode() {

            // set all pointers to nullptr
            parent = child = left = right = nullptr;

        }

};

}

#endif
