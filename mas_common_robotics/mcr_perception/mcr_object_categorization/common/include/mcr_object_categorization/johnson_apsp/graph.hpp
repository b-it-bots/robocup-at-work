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

#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include <list>

#include "node.hpp"

using namespace std;

class graph
{
protected:
    list<node*> nodes;
public:


    virtual node* add_node(const int id);

    void add_undirectedEdge(const int id, const int tgt, const double weight);
    void add_edge(const int id, const int tgt, const double weight);
    void print() const;
    const std::list<node*>* get_nodes() const;
    graph();
    graph(const char* path);
    void load_file(const char* path);
    graph(const graph& g);
    void copy_from_graph(const graph& g);
    std::list<edge*>* create_edge_list();
    void remove_node(node* node);
};

#endif /* GRAPH_HPP_ */
