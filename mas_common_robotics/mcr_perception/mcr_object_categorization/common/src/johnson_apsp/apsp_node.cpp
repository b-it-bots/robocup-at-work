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

#include "johnson_apsp/apsp_node.hpp"

#include <map>

using namespace std;

apsp_node::apsp_node(const int id)
    : node(id)
{

}

double apsp_node::get_dist_from(node* start)
{
    return dists[start];
}

void apsp_node::set_dist_from(node* start, double dist)
{
    dists[start] = dist;
}

int apsp_node::get_heap_index()
{
    return heap_index;
}

void apsp_node::set_heap_index(const int i)
{
    heap_index = i;
}

node* apsp_node::get_predecessor_on_path_from(node* source)
{
    return predecessors[source];
}

void apsp_node::set_predecessor_on_path_from(node* source, node* predecessor)
{
    predecessors[source] = predecessor;
}
