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

#ifndef APSP_NODE_HPP_
#define APSP_NODE_HPP_

#include <map>

#include "node.hpp"

/*
 * in addition to the properties inherited of node
 * this node knows its' distances and predecessors
 * on shortest paths from all source nodes that have
 * already been computed - and also its' index in the
 * currently used priority queue
 */
class apsp_node: public node
{
private:
    int heap_index;
protected:
    std::map<node*, double> dists;
    std::map<node*, node*> predecessors;
public:
    apsp_node(const int id);
    double get_dist_from(node* start);
    void set_dist_from(node* start, double dist);
    int get_heap_index();
    void set_heap_index(const int i);
    node* get_predecessor_on_path_from(node* source);
    void set_predecessor_on_path_from(node* source, node* predecessor);
};

#endif /* APSP_NODE_HPP_ */
