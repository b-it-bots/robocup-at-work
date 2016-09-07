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
#include <fstream>
#include <list>
#include <stdlib.h>


#include "johnson_apsp/graph.hpp"
#include "johnson_apsp/node.hpp"

using namespace std;

node* graph::add_node(const int id)
{
    list<node*>::iterator i;
    for (i = nodes.begin(); i != nodes.end(); ++i)
    {
        if ((*i)->get_id() == id)
        {
            return (*i);
        }
    }

//  std::cout<<"New Node = "<<id<<"\n";
    node* tmp = new node(id);
    nodes.push_back(tmp);
    return tmp;
}

/*
 * removes only the node from the graph's node list
 * works if there are no edges that have this node
 * as target - otherwise you're screwed
 */
void graph::remove_node(node* node)
{
    nodes.remove(node);
}

void graph::add_edge(const int id, const int tgt, const double weight)
{
    node* cur = add_node(id);

    //std::cout<<"New Edge from "<<cur->get_id()<<" to "<< tgt <<" distance= "<< weight<<" \n";
    cur->add_edge(add_node(tgt), weight);
}

void graph::add_undirectedEdge(const int id, const int tgt, const double weight)
{
    node* cur = add_node(id);
    node* curTgt = add_node(tgt);

    //std::cout<<"New Edge from "<<cur->get_id()<<" to "<< tgt <<" distance= "<< weight<<" \n";

    cur->add_edge(curTgt, weight);
    curTgt->add_edge(cur, weight);
}

void graph::print() const
{
    list<node*>::const_iterator i;
    for (i = nodes.begin(); i != nodes.end(); ++i)
    {
        (*i)->print();
        cout << endl;
    }
}

const std::list<node*>* graph::get_nodes() const
{
//  std::cout<<"GETNODE size "<<nodes.size()<<"\n";
    return &nodes;
}

/*
 * returns list of edges
 */
std::list<edge*>* graph::create_edge_list()
{
    list<edge*>* edges = new list<edge*>();
    list<node*>::const_iterator i;
    const list<node*>* all_nodes = get_nodes();
    for (i = all_nodes->begin(); i != all_nodes->end(); ++i)
    {
        const list<edge*>* edg = (*i)->get_edges();
        list<edge*>::const_iterator n;
        for (n = edg->begin(); n != edg->end(); ++n)
        {
            edges->push_back(*n);
        }
    }
    return edges;
}

graph::graph()
{
}

/*
 * deep copies a graph
 */
graph::graph(const graph& g)
{
    cout << "graph from graph" << endl;
    copy_from_graph(g);
}

/*
 * helper for deep copy
 */
void graph::copy_from_graph(const graph& g)
{
    list<node*>::const_iterator i;
    for (i = g.nodes.begin(); i != g.nodes.end(); ++i)
    {
        node* tmp = add_node((*i)->get_id());
        const list<edge*>* edges = (*i)->get_edges();
        list<edge*>::const_iterator n;
        for (n = edges->begin(); n != edges->end(); ++n)
        {
            node* tgt = add_node((*n)->get_tgt()->get_id());
            tmp->add_edge(tgt, (*n)->get_weight());
        }
    }
}

graph::graph(const char* path)
{
    cout << "graph from file" << endl;
    load_file(path);
}

/*
 * graph parser (state machine)
 */
void graph::load_file(const char* path)
{
    ifstream in;
    in.open(path);

    enum
    {
        COMMENT,
        NODE_ID,
        EDGE_TGT,
        EDGE_WEIGHT
    } state;
    state = NODE_ID;

    char buf[128];
    int buf_ptr = 0;
    char cur_chr;

    int edge_tgt;
    node* node_cur;
    while ((cur_chr = in.get()) != EOF)
    {

        if (cur_chr == '\n')
        {
            switch (state)
            {
            case COMMENT:
                break;
            case EDGE_WEIGHT:
            {
                buf[buf_ptr] = '\0';
                double edge_weight = atof(buf);
                node* tmp = add_node(edge_tgt);
                node_cur->add_edge(tmp, edge_weight);
                break;
            }
            default:
                break;
            }
            state = NODE_ID;
            buf_ptr = 0;
            continue;
        }
        else if (state == COMMENT)
        {
            continue;
        }
        else if (cur_chr == '#')
        {
            state = COMMENT;
        }
        else if (buf_ptr == 0 && (cur_chr == 'n' || cur_chr == 'm'))
        {
            state = COMMENT;
        }
        else if (state == EDGE_WEIGHT && (cur_chr == ' ' || cur_chr == '\t'))
        {
            buf[buf_ptr] = '\0';
            int edge_weight = atof(buf);
            state = EDGE_TGT;
            buf_ptr = 0;
            node* tmp = add_node(edge_tgt);
            node_cur->add_edge(tmp, edge_weight);
            continue;

        }
        else if (state == EDGE_TGT && cur_chr == 'w')
        {
            buf[buf_ptr] = '\0';
            state = EDGE_WEIGHT;
            buf_ptr = 0;
            edge_tgt = atoi(buf);
            continue;
        }
        else if (cur_chr == ':')
        {
            switch (state)
            {
            case NODE_ID:
            {
                buf[buf_ptr] = '\0';
                int node_id = atoi(buf);
                state = EDGE_TGT;
                buf_ptr = 0;
                node_cur = add_node(node_id);
                break;
            }
            default:
                break;
            }
        }
        else
        {
            buf[buf_ptr++] = cur_chr;
        }
    }
}

