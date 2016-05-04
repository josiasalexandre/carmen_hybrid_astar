#ifndef PRIORITY_QUEUE_TEMPLATE_HPP
#define PRIORITY_QUEUE_TEMPLATE_HPP

#include <vector>
#include <cmath>
#include <limits>

#include "FibonacciHeapNode.hpp";
#include "../../saliency_search/Channels/VisualFeatures.H"

extern std::pair< C::iterator, C::iterator > r;
namespace astar {

template<typename T>
class PriorityQueue {

    private:

        // PRIVATE ATTRIBUTES

        // the internal fibonacci heap pointer
        astar::FibonacciHeapNode<T> *min;

        // how many nodes?
        unsigned int N;

        // PRIVATE METHODS

        // get the heap max degree
        unsigned int GetMaxDegree() {

            return(ceil(1.44 * log(N) / log(2.0) + 1.0));

        }

        // insert a given node to the heap
        void InsertNode(astar::FibonacciHeapNode<T> *node) {

            if (nullptr != min) {

                // update the fibonacci heap pointers
                // place the current node between the min and the left one
                min->left->right = node;
                node->left = min->left;
                min->left = node;
                node->right = min;

            }

            if (nullptr == min || node->Key <= min->Key) {

                // set the root node
                min = node;

            }

            // incrementing the node counter
            N += 1;

            return;

        }

        // swap two nodes
        void SwapNodes(astar::FibonacciHeapNode<T> **a, astar::FibonacciHeapNode<T> **b) {

            // a helper node
            astar::FibonacciHeapNode<T> *c = *a;

            // swapping
            *a = *b;
            *b = c;

            return;

        }

        // link nodes
        void LinkNodes(astar::FibonacciHeapNode<T> *child, astar::FibonacciHeapNode<T> *parent) {

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
        void RemoveNode(astar::FibonacciHeapNode<T> *n) {

            if (nullptr != n) {

                // build a new node
                astar::FibonacciHeapNode<T> *subs = nullptr;

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

        void Cut(astar::FibonacciHeapNode<T>* node) {

            // remove the node from the heap
            RemoveNode(node);

            // add it again
            InsertNode(node);

            // update the mark
            node->mark = false;

            return;

        }

        void CascadingCut(astar::FibonacciHeapNode<T> *p) {

            // a helper
            astar::FibonacciHeapNode<T> *aux = p->parent;

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

            // get the max degree
            unsigned int Degree = GetMaxDegree();

            // build a new node array
            astar::FibonacciHeapNode<T> *node_array = new astar::FibonacciHeapNode<T>*[Degree](nullptr);

            // build some node helpers

            // iterate over the root list
            astar::FibonacciHeapNode<T> *w = min;

            // will be inserted in the auxiliar structure
            astar::FibonacciHeapNode<T> *x = nullptr;

            // store the tree tha was in the auxiliar structure
            astar::FibonacciHeapNode<T> *y = nullptr;

            // the index is the tree degree
            unsigned int index = 0;

            while (nullptr != w) {

                // save te current node
                x = w;

                // remove the node from the heap
                RemoveNode(w);

                // get the node degree/index
                index = x->degree;

                //
                while (nullptr != node_array[index]) {

                    // get the current node in the array
                    y = node_array[index];

                    // compare the Keys
                    if (x->Key > y->Key) {

                        // swap the two nodes
                        SwapNodes(&x, &y);

                    }

                    // link the nodes
                    LinkNodes(y, x);

                    // update the index pointer
                    node_array[index] = nullptr;

                    // incrementing the degree
                    index += 1;;
                }

                // save the x node to the node array list
                node_array[index] = x;

                // get the new min node
                w = min;

            }

            // reset the min node
            min = nullptr;

            // reinserting the removed nodes
            for (unsigned int i = 0; i < Degree; i++) {

                if (nullptr != node_array[i]) {

                    // insert the node again
                    Add(node_array[i], node_array[i]->Key);

                }

            }

            delete [] node_array;

        }

        // delete a given child node
        void DestroyTree(astar::FibonacciHeapNode<T> *root) {

            // some helpers
            astar::FibonacciHeapNode<T> *node = root->child, *start = root->child, f = nullptr;

            // remove all nodes at the same level
            while (nullptr != node) {

                // save the node
                f = node;

                // update the node to the right
                node = node->right;

                // delete the old node
                DestroyTree(f);

                // verify the turn around case
                if (start == node) {

                    break;

                }

            }

            // remove the node
            delete(node);

            return;

        }

        // delete the entire heap
        void DestroyHeap() {

            // some helpers
            astar::FibonacciHeapNode<T> *node = min, start = min, f = nullptr;

            while(nullptr != node) {

                // get the current node to delete
                f = node;

                // update the node pointer
                node = node->right;

                // remove the f node
                DestroyTree(f);

                // verify the complete turn
                if (start == node) {

                    break;

                }

            }

            return;

        }

    public:

        // PUBLIC ATTRIBUTES

        // basic constructor
        PriorityQueue() : min(nullptr), N(0) {}

        // basic destructor
        ~PriorityQueue() {

            // destroy the entire heap
            DestroyHeap();

        }

        // PUBLIC METHODS

        // insert a new element in the priority queue
        void Add(T element, double Key) {

            // build a new fibonacci heap node
            astar::FibonacciHeapNode<T> *node = new astar::FibonacciHeapNode<T>(T, Key);

            // inser the node to the heap
            InsertNode(node);

        }

        // extract the min element
        T DeleteMin() {

            // build the element to return
            T element;

            if (0 < N && nullptr != min) {

                // get the min node
                astar::FibonacciHeapNode<T> *min_node = min;

                // auxiliar node pointer
                astar::FibonacciHeapNode<T> *aux = min->child;

                // disconnect all children
                while(nullptr != aux && nullptr != aux->parent) {

                    // set the parent pointer to null
                    aux->parent = nullptr;

                    // go to the right node, same level
                    aux = aux->right;

                }

                //
                if (nullptr != aux) {

                    // update the min node
                    min_node->left->right = aux->right;
                    aux->right->left = min_node->left;
                    min_node->left = aux;
                    aux->right = min_node;

                }

                // update the min node
                min_node->left->right = min_node->right;
                min_node->right->left = min_node->left;


                if (min_node == min_node->right) {

                    // we got an empty priority queue
                    min = nullptr;

                } else {

                    // se the right node as the new root
                    min = min->right;

                    // consolidate the heap
                    Consolidate();

                }

                // decrease the nodes counter
                N -= 1;;

                // get the desired element
                element = min_node->element;

                // delete the node
                delete(min_node);

            }

            // return the min node
            return(element);

        }

        // update the heap after a decrease key
        void DecreaseKey(astar::FibonacciHeapNode<T> *node, double Key) {

            if (Key < node->Key && nullptr != node) {

                // update the node's key
                node->Key = Key;

                // build a new fibonacci heap node
                astar::FibonacciHeapNode<T> *p = node->parent;

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
        T DeleteNode(astar::FibonacciHeapNode<T> *node) {

            // decrease the Key
            DecreaseKey(node, -std::numeric_limits::infinity());

            // remove the node and return the element
            return DeleteMin();

        }

};

}

#endif
