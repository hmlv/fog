/**************************************************************************************************
 * Authors: 
 *   Huiming Lv
 *
 * Routines:
 *   Implements finding 1-step out-neighbors from a given query vertex ID 
 *   
 *************************************************************************************************/

#ifndef __TwoNH_H__
#define __TwoNH_H__

#include "types.hpp"
#include "fog_engine.hpp"
#include "../headers/index_vert_array.hpp"
#include "print_debug.hpp"
#include "limits.h"
#include "stdlib.h"
#include <vector>
#include <sys/timeb.h>


template <typename T>
class TwoNh_program{
	public:
        void run(u32_t query_root)
        {
            ftime(&start_t);
            start = 1000 * start_t.time + start_t.millitm;
            /*
            sleep(3);
            ftime(&end_t);
            end = 1000 * end_t.time + end_t.millitm;

            PRINT_DEBUG("%lld\n", start);
            PRINT_DEBUG("%lld\n", end);
            PRINT_DEBUG_LOG("run time: %lld ms\n", end-start);
            */

            index_vert_array<T> * vert_index = new index_vert_array<T>;
            T * t_edge;
            u32_t num_out_edges = vert_index->num_edges(query_root, OUT_EDGE);
            //PRINT_DEBUG_LOG("vertex %d, num_out_edges is %d\n", query_root, num_out_edges);
            u32_t one_step_vertex = 0;
            u32_t one_step_num_out_edges = 0; 
            
            u32_t counts = 0;
            u32_t LEN = 10; 
            u32_t * p_nh = (u32_t *)malloc(sizeof(u32_t) * LEN);

            for (u32_t i = 0; i < num_out_edges; i++)
            {
                if (0 == i && 0 == query_root)
                {
                    continue;
                }
                t_edge = vert_index->get_out_edge(query_root, i);
                assert(t_edge);
                one_step_vertex = t_edge->get_dest_value();
                one_step_num_out_edges = vert_index->num_edges(one_step_vertex, OUT_EDGE);
                for (u32_t j = 0; j < one_step_num_out_edges; j++)
                {
                    t_edge = vert_index->get_out_edge(one_step_vertex, j);
                    assert(t_edge);

                    if (counts == LEN)
                    {
                        LEN = LEN * 10;
                        p_nh = (u32_t *)realloc(p_nh, sizeof(u32_t) * LEN);
                    }
                    if (query_root != t_edge->get_dest_value())
                    {
                        p_nh[counts] = t_edge->get_dest_value();
                        counts++;
                    }
                    //PRINT_DEBUG_LOG("vertex %d is two-step neighbour of query_root %d\n", t_edge->get_dest_value(), query_root);
                }
            }

            //PRINT_DEBUG("counts = %d\n", counts);
            std::vector<u32_t> nh_vec(p_nh, p_nh+counts);
            sort(nh_vec.begin(), nh_vec.end());
            nh_vec.erase(unique(nh_vec.begin(),nh_vec.end()), nh_vec.end()); 

            /*
            std::vector<u32_t>::iterator iter;
            for (iter = nh_vec.begin(); iter != nh_vec.end(); iter++)
            {
                //PRINT_DEBUG_LOG("%d\n", *iter);
                temp++;
            }
            */
            PRINT_DEBUG_LOG("vertex %d has %ld two-strip neighbours\n", query_root, nh_vec.size()); 

            ftime(&end_t);
            end = 1000 * end_t.time + end_t.millitm;
            PRINT_DEBUG("run time: %lld ms\n", end-start);

        }

        struct timeb start_t;
        struct timeb end_t;
        long long start;
        long long end;
};

#endif
