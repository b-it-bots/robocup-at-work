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
#include <string>

#include "johnson_apsp/apsp_graph.hpp"
#include "johnson_apsp/apsp_node.hpp"

using namespace std;

apsp_graph::apsp_graph(const char* path)
{
    load_file(path);
}

apsp_graph::apsp_graph(const apsp_graph& g)
{
    copy_from_graph(g);
}

apsp_graph::apsp_graph(const graph& g)
{
    copy_from_graph(g);
}

node* apsp_graph::add_node(const int id)
{
    list<node*>::iterator i;
    for (i = nodes.begin(); i != nodes.end(); ++i)
    {
        if ((*i)->get_id() == id)
        {
            return (*i);
        }
    }
    apsp_node* tmp = new apsp_node(id);
    nodes.push_back(tmp);
    return tmp;
}

bool compare_node_ids(const node* a, const node* b)
{
    if (a->get_id() < b->get_id())
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
 * prints distance matrix
 */
void apsp_graph::print_matrix() const
{
    list<apsp_node*> all_nodes = list<apsp_node*> (
                                     *(list<apsp_node*>*) get_nodes());
    all_nodes.sort(compare_node_ids);
    list<apsp_node*>::iterator i;
    list<apsp_node*>::iterator n;
    for (i = all_nodes.begin(); i != all_nodes.end(); ++i)
    {
        cout << (*i)->get_id() << " : ";
        for (n = all_nodes.begin(); n != all_nodes.end(); ++n)
        {
            //  cout << (*n)->get_dist_from(*i) << " ";
            double dist = (*n)->get_dist_from(*i);
            if (dist == APSP_INFINITY)
            {
                cout << "inf ";
            }
            else
            {
                cout << dist << " ";
            }
        }
        cout << endl;
    }
}

/*
 * prints distance matrix
 */
void apsp_graph::print_matrix(string filename) const
{

    std::ofstream fout(filename.c_str(), ios::out | ios::binary);

    list<apsp_node*> all_nodes = list<apsp_node*> (
                                     *(list<apsp_node*>*) get_nodes());
    all_nodes.sort(compare_node_ids);
    list<apsp_node*>::iterator i;
    list<apsp_node*>::iterator n;
    //fout << "digraph A {\n" << "  rankdir=LR\n" << "size=\"5,3\"\n" << "ratio=\"fill\"\n" << "edge[style=\"bold\"]\n" << "node[shape=\"circle\"]\n";
    for (i = all_nodes.begin(); i != all_nodes.end(); ++i)
    {
        fout << (*i)->get_id() << " : ";
        for (n = all_nodes.begin(); n != all_nodes.end(); ++n)
        {
            double dist = (*n)->get_dist_from(*i);
            if (dist == APSP_INFINITY)
            {
                fout << "inf ";
            }
            else
            {
                fout << dist << " ";
            }
        }
        fout << endl;
    }
}


/*
 * initializes the single source shortest path
 * problem ( d[v]=INFINITY, pi[v]=NIL, d[s]=0 )
 */
void initSSP(apsp_graph* G, apsp_node* source)
{
    const std::list<node*>* nodes = G->get_nodes();
    list<node*>::const_iterator i;
    for (i = nodes->begin(); i != nodes->end(); ++i)
    {
        apsp_node* node = (apsp_node*)(*i);
        node->set_dist_from(source, APSP_INFINITY);
        node->set_predecessor_on_path_from(source, NULL);
    }
    source->set_dist_from(source, 0);
}

/*
 * relaxes an edge - returns true if distance and predecessor
 * of the edge's target are changed, false otherwise
 */
bool relax(edge* e, node* source)
{
    apsp_node* src = (apsp_node*)(e->get_src());
    apsp_node* tgt = (apsp_node*)(e->get_tgt());
    double weight = e->get_weight();

    // notice that we have to check src's distance for not being INFINITY
    // since INFINITY is just implemented as normal integer - might cause false results otherwise
    if (tgt->get_dist_from(source) > src->get_dist_from(source) + weight
            && src->get_dist_from(source) != APSP_INFINITY)
    {
        tgt->set_dist_from(source, src->get_dist_from(source) + weight);
        tgt->set_predecessor_on_path_from(source, src);
        return true;
    }
    return false;
}

/*
 * the Bellman-Ford algorithm in it's usual form
 */
bool bellman_ford(apsp_graph* G, apsp_node* source)
{
    initSSP(G, source);
    unsigned int i;
    std::list<edge*>* edges = G->create_edge_list();
    list<edge*>::const_iterator j;
    for (i = 1; i < G->get_nodes()->size(); i++)
    {

        for (j = edges->begin(); j != edges->end(); j++)
        {
            relax((*j), source);
        }
    }
    for (j = edges->begin(); j != edges->end(); j++)
    {
        apsp_node* edge_tgt = (apsp_node*)((*j)->get_tgt());
        apsp_node* edge_src = (apsp_node*)((*j)->get_src());
        if (edge_tgt->get_dist_from(source) > edge_src->get_dist_from(source)
                + (*j)->get_weight() && edge_src->get_dist_from(source)
                != APSP_INFINITY)
        {
            return false;
        }
    }
    return true;
}

/*
 * Dijkstra's SSSP algorithm - nothing abnormal here
 */
void dijkstra(apsp_graph* G, apsp_node* source)
{
    initSSP(G, source);

    // init Queue
    const std::list<node*>* nodes = G->get_nodes();
    prio_queue Q = prio_queue(nodes->size(), source, nodes);

    apsp_node* u;

    while ((u = Q.extract_min()) != NULL)
    {
        const std::list<edge*>* edges = u->get_edges();
        list<edge*>::const_iterator i;
        for (i = edges->begin(); i != edges->end(); i++)
        {
            if (relax((*i), source))
            {
                apsp_node* tgt = (apsp_node*)((*i)->get_tgt());
                Q.decrease_key(tgt->get_heap_index());
            }
        }
    }
}

bool apsp_graph::apspJohnson()
{
    apsp_graph* G = this;
    // add new node and connect it to all the others with weight 0
    const std::list<node*>* nodes = G->get_nodes();
    node* s = G->add_node(-1);
    list<node*>::const_iterator i;
    for (i = nodes->begin(); i != nodes->end(); i++)
    {
        G->add_edge(-1, (*i)->get_id(), 0);
    }

    if (!bellman_ford(G, (apsp_node*) s))
        return false;

    G->remove_node(s);

    // weight change for Dijkstra (so that each weight is positive)
    list<edge*>::const_iterator j;
    const std::list<edge*>* edges = G->create_edge_list();
    for (j = edges->begin(); j != edges->end(); j++)
    {
        double new_weight = (*j)->get_weight()
                            + ((apsp_node*)(*j)->get_src())->get_dist_from(s)
                            - ((apsp_node*)(*j)->get_tgt())->get_dist_from(s);
        if (TESTRELAX)
        {
            cout << "old edge weight: " << (*j)->get_weight() << endl;
            cout << "new edge weight: " << new_weight << endl;
        }
        (*j)->set_weight(new_weight);
    }

    // run Dijkstra for each node
    for (i = nodes->begin(); i != nodes->end(); i++)
    {
        dijkstra(G, (apsp_node*)(*i));

        // correct distances again
        // notice that we only have to correct distances that are not INFINITY
        list<node*>::const_iterator k;
        for (k = nodes->begin(); k != nodes->end(); k++)
        {
            apsp_node* node = (apsp_node*)(*k);
            if (node->get_dist_from((*i)) != APSP_INFINITY)
            {
                double new_distance = node->get_dist_from((*i))
                                      + node->get_dist_from(s)
                                      - ((apsp_node*)(*i))->get_dist_from(s);
                ((apsp_node*)(*k))->set_dist_from((*i), new_distance);
            }
        }
    }

    return true;
}
