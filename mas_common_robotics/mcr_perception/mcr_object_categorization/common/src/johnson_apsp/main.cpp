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

#include "main.hpp"

#include <iostream>

#include "johnson_apsp/apsp_graph.hpp"
#include "johnson_apsp/prio_queue.hpp"


int main(int argc, char** argv)
{
//  if(argc < 2)
//  {
//      cout << "USAGE: " << argv[0] << " <INPUT_FILE>" << endl;
//      return -1;
//  }

    //apsp_graph bla = apsp_graph(argv[1]);

    apsp_graph bla;

    node *n1 = bla.add_node(1);
    node *n2 = bla.add_node(2);
    node *n3 = bla.add_node(3);
    node *n4 = bla.add_node(4);
    bla.add_undirectedEdge(n1->get_id(), n2->get_id(), 110.4);
    bla.add_undirectedEdge(n1->get_id(), n3->get_id(), 150.12);
    bla.add_undirectedEdge(n2->get_id(), n3->get_id(), 150.41);
    bla.add_undirectedEdge(n4->get_id(), n1->get_id(), 91.23);

//  n1->print();
//  n2->print();
//  n3->print();

    bla.apspJohnson();




//  if (bla.apspJohnson())
    bla.print_matrix();
    bla.print_matrix("out.txt");
//  else
//      cout << "negative-cycles" << endl;

    return 0;
}

