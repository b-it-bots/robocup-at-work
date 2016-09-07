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

#ifndef EDGE_HPP_
#define EDGE_HPP_

#include "node.hpp"
class node;

class edge
{
private:
    double weight;
    node* tgt;
    node* src;
public:

    edge(node* src, node* tgt, const double weight);

    double get_weight() const;
    void set_weight(double weight);
    node* get_tgt() const;
    void print() const;
    node *get_src() const;

};

#endif /* EDGE_HPP_ */
