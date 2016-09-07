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


#ifndef PRIO_QUEUE_HPP_
#define PRIO_QUEUE_HPP_

#include "apsp_node.hpp"

/*
 * a priority queue specifically for Dijkstras SSSP
 * relying on a binary heap
 */
class prio_queue
{
private:
    apsp_node** heap;
    int heap_size;
    apsp_node* source;
    void min_heapify(int i);
    int left(int i);
    int right(int i);
    int parent(int i);
    apsp_node* get(int i);
    void set(int i, apsp_node* element);
    void infix_tree_walk(int i);
public:
    prio_queue(const int heap_size, apsp_node* source, const list<node*>* nodes);
    apsp_node* extract_min();
    void decrease_key(const int i);
    void print();
};

#endif /* PRIO_QUEUE_HPP_ */
