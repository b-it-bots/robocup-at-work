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

#include <iostream>

#include "johnson_apsp/edge.hpp"

using namespace std;

edge::edge(node* src, node* tgt, const double weight)
{
    this->src = src;
    this->tgt = tgt;
    this->weight = weight;
}

double edge::get_weight() const
{
    return weight;
}

void edge::set_weight(double weight)
{
    this->weight = weight;
}

node* edge::get_tgt() const
{
    return tgt;
}

node *edge::get_src() const
{
    return src;
}

void edge::print() const
{
    cout << tgt->get_id() << "w" << weight;
}
