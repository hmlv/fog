/**************************************************************************************************
 * Authors: 
 *   Jian He
 *
 * Routines:
 *   Implements  sparse matrix-vector multiplication algorithm
 *   
 *************************************************************************************************/


#include "fog_adapter.h"

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

template <typename T>
class spmv_program{
	public:
        static u32_t num_tasks_to_sched;
        static int forward_backward_phase;
        static int CONTEXT_PHASE;
        static int loop_counter;
        static bool init_sched;
        static bool set_forward_backward;
		//initialize each vertex of the graph
		static void init( u32_t vid, spmv_vert_attr* this_vert, index_vert_array<T> * vert_index )
		{
			this_vert->origin_value = (float)1.0;
            this_vert->spmv_value = (float)0.0;
		}

		static update<spmv_update> *scatter_one_edge(
                spmv_vert_attr * this_vert,
                T * this_edge,
                u32_t something)
        {
            assert(forward_backward_phase == FORWARD_TRAVERSAL);
        	update<spmv_update> * ret;
            float scatter_value = this_vert->origin_value * this_edge->get_edge_value();
			ret = new update<spmv_update>;
			ret->dest_vert = this_edge->get_dest_value();
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
        static void before_iteration()
        {
            PRINT_DEBUG("SPMV engine is running for the %d-th iteration, there are %d tasks to schedule!\n",
                    loop_counter, num_tasks_to_sched);
        }
        static int after_iteration()
        {
            PRINT_DEBUG("SPMV engine has finished the %d-th iteration!\n", loop_counter);
            return ITERATION_STOP;
        }
        static int finalize(spmv_vert_attr * va)
        {
            for (unsigned int id = 0; id < 100; id++)
                PRINT_DEBUG_LOG("SPMV:result[%d], origin_value = %f, spmv_value = %f\n", id, (va+id)->origin_value, (va+id)->spmv_value);

            PRINT_DEBUG("SPMV engine stops!\n");
            return ENGINE_STOP;
        }


};
template <typename T>
u32_t spmv_program<T>::num_tasks_to_sched = 0;

template <typename T>
int spmv_program<T>::forward_backward_phase = FORWARD_TRAVERSAL;

template <typename T>
int spmv_program<T>::CONTEXT_PHASE = 0;

template <typename T>
int spmv_program<T>::loop_counter = 0;

template <typename T>
bool spmv_program<T>::set_forward_backward = false;

template <typename T>
bool spmv_program<T>::init_sched = false;
//if you want add_schedule when init(), please set this to be true~!


template <typename T>
void start_engine()
{
    if(8 != sizeof(T))
    {
        PRINT_ERROR("This algorithm need 'type1' edge file!\n");
    }
    fog_engine<spmv_program<T>, spmv_vert_attr, spmv_update, T> * eng;
    (*(eng = new fog_engine<spmv_program<T>, spmv_vert_attr, spmv_update, T>(GLOBAL_ENGINE)))();
    delete eng;
}


int main(int argc, const char**argv)
{
    Fog_adapter *adapter = new Fog_adapter();
    unsigned int type1_or_type2 = adapter->init(argc, argv);

    if(1 == type1_or_type2)
    {
        start_engine<type1_edge>();
    }   
    else
    {
        start_engine<type2_edge>();
    }   

    return 0;

}
