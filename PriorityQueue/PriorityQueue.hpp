#ifndef PRIORITY_QUEUE_TEMPLATE_HPP
#define PRIORITY_QUEUE_TEMPLATE_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <exception>

#include "PriorityQueueNode.hpp"

namespace astar {

#define ASTAR_CEIL_LOG_INT_BITS (sizeof(int) * 8)

template<typename T>
class PriorityQueue {

    private:

        // PRIVATE ATTRIBUTES

        // the main root pointer
        astar::PriorityQueueNodePtr<T> root;

        // the internal fibonacci min heap pointer
        astar::PriorityQueueNodePtr<T> min;

        // for reorganizing purpose
        std::vector<astar::PriorityQueueNodePtr<T>> A;

        // the max degree log
        int D;

        // how many nodes?
        unsigned int N;

        // the null value
        T null_value;

        // PRIVATE METHODS

        // get the heap max degree
        inline int GetMaxDegree() {

            int oa;
            int i;
            int b;

            int a = (int) (N + 1);

            oa = a;
            b = ASTAR_CEIL_LOG_INT_BITS / 2;
            i = 0;
            while (b) {
                i = (i << 1);
                if (a >= (1 << b)) {
                    a /= (1 << b);
                    i = i | 1;
                } else
                    a &= (1 << b) - 1;
                b /= 2;
            }
            if ((1 << i) == oa)
                return i + 1;
            else
                return i + 2;

        }

        // make sure we have enough memmory to reorganize the heap
        void CheckCons() {

            if (-1 == D || N > ( 1 << D)) {

                // the old degree value
                int old_degree = D;

                // update the degree value
                D = std::max(GetMaxDegree(), 8);

                if (D != old_degree) {

                    // resize it
                    A.clear();

                    A.resize(D + 1, nullptr);

                    return;
                }

            }

            for (unsigned int i = 0; i < A.size(); ++i) {

                A[i] = nullptr;

            }

        }

        // insert a node 'b' after node 'a'
        void InsertNodeAfter(astar::PriorityQueueNodePtr<T> a, astar::PriorityQueueNodePtr<T> b) {

            if (a != a->right) {

                b->right = a->right;
                a->right->left = b;
                a->right = b;
                b->left = a;

            } else {

                a->right = b;
                a->left = b;
                b->right = a;
                b->left = a;

            }

        }

        // insert a node 'b' before node 'a'
        void InsertNodeBefore(astar::PriorityQueueNodePtr<T> a, astar::PriorityQueueNodePtr<T> b) {

            // tricky
            InsertNodeAfter(a->left, b);

        }

        // insert a given node to the heap
        void InsertRootList(astar::PriorityQueueNodePtr<T> x) {

            if (nullptr != root) {

                InsertNodeAfter(root, x);

                return;
            }

            // set the current root node
            x->left = x;
            x->right = x;
            root = x;

        }

        // Remove a node from the list and returns the left one, if exists
        astar::PriorityQueueNodePtr<T> RemoveNode(astar::PriorityQueueNodePtr<T> x) {

            // the return value
            astar::PriorityQueueNodePtr<T> ret = (x->left != x) ? x->left : nullptr;

            // fix the parent pointer
            if (nullptr != x->parent && x->parent->child == x) {

                x->parent->child = ret;

            }

            // change the connections
            x->right->left = x->left;
            x->left->right = x->right;

            // clear out the hanging pointers
            x->parent = nullptr;
            x->left = x->right = x;

            return ret;

        }

        // Remove a node from the root list
        void RemoveNodeFromRootList(astar::PriorityQueueNodePtr<T> x) {

            if (x->left == x) {

                // we are empty
                root = nullptr;

            } else {

                root = RemoveNode(x);

            }

        }

        // set the null value
        T ExtractMin() {

            // get the min node
            astar::PriorityQueueNodePtr<T> z = min;

            // get the min child
            astar::PriorityQueueNodePtr<T> x = z->child;

            // another auxiliary node
            astar::PriorityQueueNodePtr<T> y;

            // remove all the children and move to the root list
            if (nullptr != x) {

                // set the child pointer to null
                z->child = nullptr;

                while(nullptr != x->parent) {

                    // get the right element
                    y = x->right;

                    // disconnect the current node from the parent
                    x->parent = nullptr;

                    // insert the current x node to the root list
                    InsertRootList(x);

                    // move the current pointer to the right
                    x = y;

                }

            }

            // remove the min from the root list
            RemoveNodeFromRootList(z);

            // set the min node to nullptr
            min = nullptr;

            // decrement the node counter
            N -= 1;

            if (0 != N) {

                // we are not empty
                Consolidate();

            }

            T element = z->element;

            // remove the z node
            delete z;

            // return the min value
            return element;

        }

        // swap two nodes
        void SwapNodes(astar::PriorityQueueNode<T> **a, astar::PriorityQueueNode<T> **b) {

            // swapping
            astar::PriorityQueueNode<T> *c = *a;
            *a = *b;
            *b = c;

            return;

        }

        // link nodes
        void LinkNodes(astar::PriorityQueueNode<T> *y, astar::PriorityQueueNode<T> *x) {

            // make y a child of x
            if (nullptr == x->child) {

                // the x node has no child, direct assignement
                x->child = y;

            } else {

                // insert the y node to x->child left
                InsertNodeBefore(x->child, y);

            }

            // make x the parent of y
            y->parent = x;

            // update the node x values
            y->mark = false;
            x->degree += 1;

        }

        // cut the nodes
        void Cut(astar::PriorityQueueNodePtr<T> x, astar::PriorityQueueNodePtr<T> y) {

            // remove the node from the heap
            RemoveNode(x);

            // the node y lost a child
            y->degree -= 1;

            // add it again
            InsertRootList(x);

            // update values
            x->mark = false;
            x->parent = nullptr;

        }

        // cascading cut
        void CascadingCut(astar::PriorityQueueNodePtr<T> y) {

            // a helper
            astar::PriorityQueueNodePtr<T> z;

            while ((nullptr != (z = y->parent))) {

                if (y->mark) {

                    // cut the node
                    Cut(y, z);

                    // update the y node
                    y = z;

                } else {

                    y->mark = true;

                    return;
                }

            }

        }

        // consolidate the heap after some min removal
        void Consolidate() {

            // verify the current max degree and resize the A vector
            CheckCons();

            // a helper node to iterate over the root list
            astar::PriorityQueueNodePtr<T> w = nullptr;

            // pointer to the last root node
            astar::PriorityQueueNodePtr<T> x = nullptr;

            // pointer to the node
            astar::PriorityQueueNodePtr<T> y = nullptr;

            // the degree from a given node
            unsigned int d;

            // iterate over the entire root list
            while(nullptr != (w = root)) {

                // save the current node
                x = w;

                // move to the right
                RemoveNodeFromRootList(w);

                // get the index degree
                d = x->degree;

                // comparing with the node array
                while(nullptr != A[d]) {

                    // get the node array pointer
                    y = A[d];

                    // compare the keys
                    if (x->Key > y->Key) {

                        // swap the nodes
                        SwapNodes(&x, &y);

                    }

                    // link the nodes
                    LinkNodes(y, x);

                    // set the current node array pointer as a nullptr
                    A[d] = nullptr;

                    // update the degree
                    d += 1;

                }

                // move the current x node to the array
                A[d] = x;

            }

            // move the array to the root list
            for (unsigned int i = 0; i < A.size(); i++) {

                if (nullptr != A[i]) {

                    // insert the node to the root list
                    InsertRootList(A[i]);

                    if (nullptr == min || A[i]->Key < min->Key) {

                        min = A[i];

                    }

                }

            }

        }

        // delete a given child node
        void DestroySubTree(astar::PriorityQueueNodePtr<T> x) {

            // some helpers
            astar::PriorityQueueNodePtr<T> tmp = x->right;

            while(tmp != x) {

                // disconect the tmp from the list
                tmp->left->right = tmp->right;
                tmp->right->left = tmp->left;

                // reset the left and right pointers
                tmp->left = tmp->right = tmp;

                // delete the subtree
                DestroySubTree(tmp);

                // get the next neighbour
                tmp = x->right;

            }

            // remove the child nodes
            if (nullptr != x->child) {

                // get the root child address
                tmp = x->child;

                // reset the root's children pointer
                x->child = nullptr;

                // delete the subtree
                DestroySubTree(tmp);

            }

            // remove the current node
            delete x;

        }

    public:

        // PUBLIC ATTRIBUTES

        // basic constructor
        PriorityQueue(T _null) : root(nullptr), min(nullptr), A(0), D(-1), N(0), null_value(_null) {}

        // basic destructor
        ~PriorityQueue() {

            // destroy the entire heap
            ClearHeap();

        }

        // PUBLIC METHODS

        // insert a new element in the priority queue
        astar::PriorityQueueNodePtr<T> Add(T element, double Key) {

            // build a new fibonacci heap node
            astar::PriorityQueueNodePtr<T> node = new astar::PriorityQueueNode<T>(element, Key);

            // insert the node to the heap
            InsertRootList(node);

            // is it a new min node?
            if (nullptr == min || Key < min->Key) {

                min = node;

            }

            // incrementing the node counter
            N += 1;

            // return the node pointer
            return node;

        }

        // get the min element
        T Min() {

            // get the min element or just the null value
            // return the element
            return ((nullptr != min) ? min->element : null_value);

        }

        // extract the min element
        T DeleteMin() {

            // return the min element or just the null_value
            return ((nullptr != min) ? ExtractMin() : null_value);

        }

        // update the heap after a decrease key
        void DecreaseKey(astar::PriorityQueueNodePtr<T> x, double Key) {

            if (nullptr != x && Key < x->Key) {

                // update the node's key
                x->Key = Key;

                // build a new fibonacci heap node
                astar::PriorityQueueNodePtr<T> y = x->parent;

                if (nullptr != y && Key <= y->Key) {

                    // cut the node
                    Cut(x, y);

                    // cascading cut
                    CascadingCut(y);

                }

                // verify the min key
                if (nullptr == min || Key < min->Key) {

                    // update the new min node
                    min = x;

                }

            }

        }

        // is empty?
        bool isEmpty() {

            return 0 == N;

        }

        // delete a given node from the heap
        T DeleteNode(astar::PriorityQueueNode<T> *node) {

            // decrease the Key
            DecreaseKey(node, -std::numeric_limits<double>::max());

            // remove the node and return the element
            return DeleteMin();

        }

        // delete the entire heap
        void ClearHeap() {

            if (nullptr != root) {

                // Remove the entire subtree
                DestroySubTree(root);

                root = nullptr;
                min = nullptr;

                // reset the heap variables
                N = 0;
                D = -1;
                A.clear();

            }

        }

        unsigned int GetN() {return N;};

};

}

#endif
