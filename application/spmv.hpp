#ifndef __SPMV_HPP__
#define __SPMV_HPP__

#define DAMPING_FACTOR	0.85

//this structure will define the "attribute" of one vertex, the only member will be the rank 
// value of the vertex
struct spmv_vert_attr{
	float origin_value;
    float spmv_value;
};

struct spmv_update
{
    float spmv_value;
};

class spmv_program{
	public:
		//initialize each vertex of the graph
		static void init( u32_t vid, spmv_vert_attr* this_vert )
		{
			this_vert->origin_value = (float)1.0 * vid;
            this_vert->spmv_value = (float)0.0;
		}
		static void init(u32_t vid, spmv_vert_attr * va, u32_t PHASE, u32_t init_forward_backward_phase, u32_t loop_counter,
            index_vert_array * vert_index){}
		static void init(u32_t vid, spmv_vert_attr* va, u32_t PHASE){}

		//This member function is defined to process one of the out edges of vertex "vid".
		//Explain the parameters:
		// vid: the id of the vertex to be scattered.
		// this_vert: point to the attribute of vertex to be scattered.
		// num_outedge: the number of out edges of the vertex to be scattered.
		// this_edge: the edge to be scattered this time. 
		//Notes: 
		// 1) this member fuction will be used to scatter ONE edge of a vertex.
		// 2) the return value will be a pointer to the generated update.
		//	However, it is possible that no update will be generated at all! 
		//	In that case, this member function should return NULL.
		// 3) This function should be "re-enterable", therefore, no global variables
		//	should be visited, or visited very carefully.
		static update<spmv_update>* scatter_one_edge( u32_t vid, 
					spmv_vert_attr* this_vert, 
					u32_t num_outedge, 
					edge* this_edge )
		{
			return NULL;
		}

		static update<spmv_update> *scatter_one_edge(u32_t vid,
                spmv_vert_attr * this_vert,
                edge * this_edge)
        {
        	update<spmv_update> * ret;
			//float scatter_weight = DAMPING_FACTOR *(this_vert->rank/num_outedge) + (1- DAMPING_FACTOR);
            float scatter_value = this_vert->origin_value * this_edge->edge_weight;
			ret = new update<spmv_update>;
			ret->dest_vert = this_edge->dest_vert;
			ret->vert_attr.spmv_value = scatter_value;
			return ret;
		}

		// Gather one update. Explain the parameters:
		// vid: the vertex id of destination vertex;
		// va: the attribute of destination vertex;
		// u: the update from the "update" buffer.
		static void gather_one_update( u32_t vid, spmv_vert_attr * dest_vert_attr, update<spmv_update> * u )
		{
			assert( vid == u->dest_vert );
			dest_vert_attr->spmv_value += u->vert_attr.spmv_value;
		}

		static void gather_one_update( u32_t vid, spmv_vert_attr* this_vert, 
                struct update<spmv_update>* this_update, 
                u32_t PHASE){}

		static void set_finish_to_vert(u32_t vid, spmv_vert_attr * this_vert){}
        static bool judge_true_false(spmv_vert_attr* va){return false;}
        static bool judge_src_dest(spmv_vert_attr *va_src, spmv_vert_attr *va_dst, float edge_weight){return false;}

        static void print_result(u32_t vid, spmv_vert_attr * va)
        {
            PRINT_DEBUG("SPMV:result[%d], origin_value = %f, spmv_value = %f\n", vid, va->origin_value, va->spmv_value);
        }

};

#endif
