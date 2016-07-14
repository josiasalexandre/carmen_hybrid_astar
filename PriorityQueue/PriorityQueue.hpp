#ifndef PRIORITY_QUEUE_TEMPLATE_HPP
#define PRIORITY_QUEUE_TEMPLATE_HPP

#include <vector>
#include <cmath>
#include <limits>

#include "PriorityQueueNode.hpp"

namespace astar {

template<typename T>
class PriorityQueue {

    private:

        // PRIVATE ATTRIBUTES

        // the internal fibonacci heap pointer
        astar::PriorityQueueNode<T> *min;

        // how many nodes?
        unsigned int N;

        // the null value
        T null_value;

        // PRIVATE METHODS

        // get the heap max degree
        unsigned int GetMaxDegree() {

            return(ceil(1.44 * log(N) / log(2.0) + 1.0));

        }

        // insert a given node to the heap
        void InsertNode(astar::PriorityQueueNode<T> *node) {

            if (nullptr != min) {

                // update the fibonacci heap pointers
                // place the current node between the min and the left one
                min->left->right = node;
                node->left = min->left;
                node->right = min;
                min->left = node;

            }

            if (nullptr == min || node->Key <= min->Key) {

                // set the root node
                min = node;

            }

            return;

        }

        // swap two nodes
        void SwapNodes(astar::PriorityQueueNode<T> **a, astar::PriorityQueueNode<T> **b) {

            // a helper node
            astar::PriorityQueueNode<T> *c = *a;

            // swapping
            *a = *b;
            *b = c;

            return;

        }

        // link nodes
        void LinkNodes(astar::PriorityQueueNode<T> *child, astar::PriorityQueueNode<T> *parent) {

            // disconect the current child from the left and right nodes
            child->left->right = child->right;
            child->right->left = child->left;

            // disconect the child node from it's parent
            child->parent = parent;

            // update the parent's child
            if (nullptr != parent->child) {

                // place the child between the parent's child right and the parent's child
                child->right = parent->child->right;
                parent->child->right->left = child;
                parent->child->right = child;
                child->left = parent->child;

            } else {

                // there's no children here, so let's place the new child
                child->left = child;
                child->right = child;
                parent->child = child;

            }

            // the child's marks is reseted here
            child->mark = false;

            // increase the parent degree
            parent->degree += 1;

            return;

        }

        // remove a node from the heap
        void RemoveNode(astar::PriorityQueueNode<T> *n) {

            if (nullptr != n) {

                // build a new node
                astar::PriorityQueueNode<T> *subs = nullptr;

                // update the subs pointer
                if (n == n->left) {

                    subs = n->left;

                }

                if (nullptr != subs) {

                    // update the node pointers
                    n->left->right = n->right;
                    n->right->left= n->left;

                }

                if (nullptr != n->parent && (n->parent->child == n)) {

                    // update the parent child relationship, now the parent gets the subs as the new child
                    n->parent->child = subs;

                    // decrease the parent degree
                    n->parent->degree -= 1;

                }

                // if the removed node is the min node we need to assign the min node to another node, in this case the subs node
                if (n == min) {

                    // the subs node is the new root
                    min = subs;

                }

                // remove the links around this node
                n->parent = nullptr;
                n->right = n;
                n->left = n;

            }

            return;

        }

        // remove a given node from the root list
        void RemoveNodeFromRootList(astar::PriorityQueueNode<T> *n) {

            if (min != min->right) {

                min = min->right;

            } else {

                // set the min as a nullptr
                min = nullptr;

            }

            // remove the node from the root list
            n->left->right = n->right;
            n->right->left = n->left;

            // reset the left and right pointers
            n->left = n->right = n;

            return;

        }

        void Cut(astar::PriorityQueueNode<T>* node) {

            // remove the node from the heap
            RemoveNode(node);

            // add it again
            InsertNode(node);

            // update the mark
            node->mark = false;

            return;

        }

        void CascadingCut(astar::PriorityQueueNode<T> *p) {

            // a helper
            astar::PriorityQueueNode<T> *aux = p->parent;

            if (nullptr != aux) {

                if (p->mark) {

                    // cut the p node
                    Cut(p);

                    // cascading cut the aux parent node
                    CascadingCut(aux);


                } else {

                    p->mark = true;

                }

            }

            return;

        }

        // consolidate the heap after some min removal
        void Consolidate() {

            // get the max degree value
            unsigned int degree = GetMaxDegree();

            // build a new node array
            astar::PriorityQueueNode<T> **A = new astar::PriorityQueueNode<T>*[degree];

            // clear the array
            for (unsigned int i = 0; i < degree; i++) {

                A[i] = nullptr;

            }

            // a helper node to iterate over the root list
            astar::PriorityQueueNodePtr<T> w;

            // pointer to the last root node
            astar::PriorityQueueNodePtr<T> x;

            // pointer to the node
            astar::PriorityQueueNodePtr<T> y;

            // get the
            unsigned int d;

            // iterate over the entire root list
            while(nullptr != (w = min)) {

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

            // set the min node as nullptr
            min = nullptr;

            // move the array to the root list
            for (unsigned int i = 0; i < degree; i++) {

                if (nullptr != A[i]) {

                    // insert the current node to the list
                    InsertNode(A[i]);

                }

            }

            // remove the node array
            delete [] A;

        }

        // delete a given child node
        void DestroyTree(astar::PriorityQueueNode<T> *root) {

            // some helpers
            astar::PriorityQueueNodePtr<T> tmp = root;

            while(tmp != tmp->right) {

                // get the next node
                root = tmp->right;

                // disconect the tmp from the list
                tmp->left->right = tmp->right;
                tmp->right->left = tmp->left;

                // reset the left and right pointers
                tmp->left = tmp->right = tmp;

                // delete the subtree
                DestroyTree(tmp);

                tmp = root;

            }

            if (nullptr != root->child) {

                // get the root child address
                tmp = root->child;

                // reset the root's children pointer
                root->child = nullptr;

                // delete the subtree
                DestroyTree(tmp);

            }


            // remove the root node
            delete(root);

            return;

        }

    public:

        // PUBLIC ATTRIBUTES

        // basic constructor
        PriorityQueue(T _null) : min(nullptr), N(0), null_value(_null) {}

        // basic destructor
        ~PriorityQueue() {

            // destroy the entire heap
            DestroyHeap();

        }

        // PUBLIC METHODS

        // insert a new element in the priority queue
        astar::PriorityQueueNodePtr<T> Add(T element, double Key) {

            // build a new fibonacci heap node
            astar::PriorityQueueNodePtr<T> node = new astar::PriorityQueueNode<T>(element, Key);

            // insert the node to the heap
            InsertNode(node);

            // incrementing the node counter
            N += 1;

            // return the node pointer
            return node;


        }

        // set the null value

        // get the min element
        T Min() {

            // build an element
            T element = null_value;

            if (nullptr != min) {

                element = min->element;

            }

            // return the element
            return element;

        }

        // extract the min element
        T DeleteMin() {

            // build the element to return
            T element = null_value;

            if (nullptr != min) {

                // get the min node
                astar::PriorityQueueNodePtr<T> z = min;

                // an auxiliar pointer to the child
                astar::PriorityQueueNodePtr<T> x = z->child;

                // find each child
                while(nullptr != x && nullptr != x->parent) {

                    // disconect the child from the parent
                    x->parent = nullptr;

                    // update to the next child
                    x = x->right;

                }

                if (nullptr != x) {

                    // concatenate the root list and the child list
                    z->left->right = x->right;
                    x->right->left = z->left;
                    z->left = x;
                    x->right = z;

                }

                // remove the z node from the root list
                z->left->right = z->right;
                z->right->left = z->left;

                if (z == z->right) {

                    // we got an empty root list
                    min = nullptr;

                } else {

                    // set the right node as the new root
                    min = z->right;

                    // consolidate the heap
                    Consolidate();

                }

                // remove a node
                N -= 1;

                // get the desired element
                element = z->element;

                // delete the node
                delete(z);

            }

            // return the min node
            return(element);

        }

        // update the heap after a decrease key
        void DecreaseKey(astar::PriorityQueueNode<T> *node, double Key) {

            if (Key < node->Key && nullptr != node) {

                // update the node's key
                node->Key = Key;

                // build a new fibonacci heap node
                astar::PriorityQueueNode<T> *p = node->parent;

                //
                if (nullptr != p && Key < p->Key) {

                    // cut the node
                    Cut(node);

                    // cascading cut
                    CascadingCut(p);

                }

                // verify the min key
                if (Key < min->Key) {

                    // update the new min node
                    min = node;

                }

            }

            return;

        }

        // is empty?
        bool isEmpty() {

            return 0 == N;

        }

        // delete a given node from the heap
        T DeleteNode(astar::PriorityQueueNode<T> *node) {

            // decrease the Key
            DecreaseKey(node, -std::numeric_limits<double>::infinity());

            // remove the node and return the element
            return DeleteMin();

        }

        // delete the entire heap
        void DestroyHeap() {

            // auxiliar node
            astar::PriorityQueueNodePtr<T> tmp;

            while(nullptr != (tmp = min)) {

                // remove the tmp node from the root list
                RemoveNodeFromRootList(tmp);

                // remove the subtree
                DestroyTree(tmp);

            }

            return;

        }

};

}

#endif
