/**************************************************************************************************
 * Authors: 
 *   Huiming Lv
 *
 * Routines:
 *   Implements finding 1-step out-neighbors from a given query vertex ID 
 *   
 *************************************************************************************************/

#ifndef __OneNH_H__
#define __OneNH_H__

#include "types.hpp"
#include "fog_engine.hpp"
#include "../headers/index_vert_array.hpp"
#include "print_debug.hpp"
#include "limits.h"

template <typename T>
class OneNh_program{
	public:
        void run(u32_t query_root)
        {
            index_vert_array<T> * vert_index = new index_vert_array<T>;
            T * t_edge;
            u32_t num_out_edges = vert_index->num_edges(query_root, OUT_EDGE);
            for (u32_t i = 0; i < num_out_edges; i++)
            {
                if (0 == i && 0 == query_root)
                {
                    continue;
                }
                t_edge = vert_index->get_out_edge(query_root, i);
                assert(t_edge);
                PRINT_DEBUG_LOG("vertex %d is one-step neighbour of query_root %d\n", t_edge->get_dest_value(), query_root);
            }

        }
};

#endif
