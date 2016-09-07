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

#include "johnson_apsp/node.hpp"

#include <list>
#include <iostream>

using namespace std;

node::node(int id)
{
    this->id = id;
}

int node::get_id() const
{
    return id;
}

/*
 * returns the node's adjacency list
 */
const std::list<edge*>* node::get_edges() const
{
    return &edges;
}

void node::add_edge(node* tgt, const double weight)
{
    //cout << " Add egde  to node "<< tgt->get_id()<<"\n";
    edges.push_back(new edge(this, tgt, weight));
}

void node::print() const
{
    cout << id << ":";
    list<edge*>::const_iterator i;
    for (i = edges.begin(); i != edges.end(); ++i)
    {
        cout << " ";
        (*i)->print();
    }
    cout << "\n";
}
