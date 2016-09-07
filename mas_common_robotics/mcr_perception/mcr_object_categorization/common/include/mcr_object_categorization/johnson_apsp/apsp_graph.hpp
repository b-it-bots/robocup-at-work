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

#ifndef APSP_GRAPH_HPP_
#define APSP_GRAPH_HPP_

#include "graph.hpp"
#include "prio_queue.hpp"
#include "apsp_node.hpp"
#include <string>

#define APSP_INFINITY (1 << 30)
#define DEBUG false
#define TESTRELAX false

class apsp_graph: public graph
{
public:
    virtual node* add_node(const int id);
    void print_matrix() const;
    void print_matrix(string filename) const;
    apsp_graph() {};
    apsp_graph(const char* path);
    apsp_graph(const graph& g);
    apsp_graph(const apsp_graph& g);
    bool apspJohnson();

};

#endif /* APSP_GRAPH_HPP_ */
