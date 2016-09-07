/*
 This file belongs to the program BloodyStupidJohnson.

 BloodyStupidJohnson is an implementation of Johnson's algorithm
 for the All Pairs Shortest Paths problem.
 Copyright (C) 2010 Markus Pargmann and Alexander Weinhold

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, version 2 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "johnson_apsp/prio_queue.hpp"
#include <iostream>

/*
 * Gets the size of the heap (number of nodes in node-set),
 * the source node from which Dijkstra is started
 * and the set of nodes itself
 */
prio_queue::prio_queue(const int heap_size, apsp_node* source, const list<node*>* nodes)
{
    this->heap_size = heap_size;
    heap = new apsp_node*[heap_size];
    this->source = source;


    // tricksing a little here - we know that the source node
    // has distance 0, others Infinity, so we just set source as
    // root and the others in the order in which they come
    //  - no complex build heap operation needed
    set(1, source);
    int counter = 2;
    list<node*>::const_iterator i;
    for (i = (*nodes).begin(); i != (*nodes).end(); ++i)
    {
        if (source != *i)
        {
            set(counter, (apsp_node*) *i);
            counter++;
        }
    }
}

/*
 * returns index of left child
 */
int prio_queue::left(int i)
{
    return 2 * i;
}

/*
 * returns index of right child
 */
int prio_queue::right(int i)
{
    return 2 * i + 1;
}

/*
 * returns index of parent
 */
int prio_queue::parent(int i)
{
    return i / 2;
}

/*
 * returns element at heap index i
 */
apsp_node* prio_queue::get(int i)
{
    return heap[i - 1];
}

/*
 * sets heap element at index i to element
 * - also lets the element know its index
 */
void prio_queue::set(int i, apsp_node* element)
{
    heap[i - 1] = element;
    element->set_heap_index(i);
}

/*
 * restores min-heap property under a node
 * with index i in heap
 */
void prio_queue::min_heapify(int i)
{
    double l = left(i);
    double r = right(i);
    double smallest = i;
    if (l <= heap_size && get(l)->get_dist_from(source) < get(smallest)->get_dist_from(source))
        smallest = l;
    if (r <= heap_size && get(r)->get_dist_from(source) < get(smallest)->get_dist_from(source))
        smallest = r;
    if (smallest != i)
    {
        apsp_node* tmp = get(i);
        set(i, get(smallest));
        set(smallest, tmp);
        min_heapify(smallest);
    }
}

/*
 * pops minimum of priority queue
 */
apsp_node* prio_queue::extract_min()
{
    if (heap_size < 1)
        return NULL;
    apsp_node* min = get(1);
    set(1, get(heap_size));
    heap_size = heap_size - 1;
    min_heapify(1);
    return min;
}

/*
 * shifts element at index i up the min-heap
 * as long as parent is larger
 */
void prio_queue::decrease_key(int i)
{
    while (i > 1 && get(parent(i))->get_dist_from(source) > get(i)->get_dist_from(source))
    {
        apsp_node* tmp = get(i);
        set(i, get(parent(i)));
        set(parent(i), tmp);
        i = parent(i);
    }
}

/*
 * infix print of the heap tree structure
 */
void prio_queue::print()
{
    if (heap_size >= 1)
        infix_tree_walk(1);
    cout << endl;
}

/*
 * recursive infix print starting at index i
 */
void prio_queue::infix_tree_walk(int i)
{
    cout << "(";
    if (left(i) <= heap_size)
    {
        infix_tree_walk(left(i));
    }
    cout << get(i)->get_id() << ":" << get(i)->get_dist_from(source);
    if (right(i) <= heap_size)
    {
        infix_tree_walk(right(i));
    }
    cout << ")";
}

/*
 * Cormen would be so proud....
 */
