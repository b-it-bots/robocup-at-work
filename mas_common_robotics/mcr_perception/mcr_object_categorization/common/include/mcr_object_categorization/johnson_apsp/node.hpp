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


#ifndef NODE_HPP_
#define NODE_HPP_

#include <list>

#include "edge.hpp"
class edge;

using namespace std;

class node
{
protected:
    list<edge*> edges;
    int id;
public:
    node(int id);
    void add_edge(node* tgt, const double weight);
    int get_id() const;
    const std::list<edge*>* get_edges() const;
    void print() const;
};

#endif /* NODE_HPP_ */
