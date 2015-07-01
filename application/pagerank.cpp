/**************************************************************************************************
 * Authors: 
 *   Zhiyuan Shao, Jian He, Huiming Lv
 *
 * Routines:
 *   Implements PageRank algorithm
 *************************************************************************************************/


#include "fog_adapter.h"
#include "fog_program.h"
#include "fog_task.h"
#include "../fogsrc/fog_task.cpp"

#define DAMPING_FACTOR	0.85

//this structure will define the "attribute" of one vertex, the only member will be the rank 
// value of the vertex
struct pagerank_vert_attr{
	float rank;
};

//template <typename VERT_ATTR, typename ALG_UPDATE, typename T>
template <typename T>
class pagerank_program : public Fog_program<pagerank_vert_attr, pagerank_vert_attr, T>
{
	public:
        //u32_t num_tasks_to_sched;
		u32_t iteration_times;	//how many iterations will there be?
        u32_t reduce_iters;
        //int forward_backward_phase;
        //int CONTEXT_PHASE;
        //int loop_counter;
        //bool init_sched;

        pagerank_program(int p_forward_backward_phase, bool p_init_sched):Fog_program<pagerank_vert_attr, pagerank_vert_attr, T>(p_forward_backward_phase, p_init_sched)
        {
            //num_tasks_to_sched = 0;
            iteration_times = 0;
            reduce_iters = 0;
            //forward_backward_phase = FORWARD_TRAVERSAL;
            //CONTEXT_PHASE = 0;
            //loop_counter = 0;
            //init_sched = false;
        }

        //initialize each vertex of the graph
        void init( u32_t vid, pagerank_vert_attr* this_vert, index_vert_array<T> * vert_index )
        {
            this_vert->rank = 1.0;
        }

		//This member function is defined to process one of the out edges of vertex "vid".
		//Explain the parameters:
		// vid: the id of the vertex to be scattered.
		// this_vert: point to the attribute of vertex to be scattered.
		// num_edge: the number of out edges of the vertex to be scattered.
		// this_edge: the edge to be scattered this time. 
		//Notes: 
		// 1) this member fuction will be used to scatter ONE edge of a vertex.
		// 2) the return value will be a pointer to the generated update.
		//	However, it is possible that no update will be generated at all! 
		//	In that case, this member function should return NULL.
		// 3) This function should be "re-enterable", therefore, no global variables
		//	should be visited, or visited very carefully.
		update<pagerank_vert_attr>* scatter_one_edge(
                    pagerank_vert_attr* this_vert, 
					T * this_edge, // type1 or type2 , only available for FORWARD_TRAVERSAL
					u32_t num_edges) 
		{
            if (this->forward_backward_phase == BACKWARD_TRAVERSAL)
                PRINT_ERROR("forward_backward_phase must set to FORWARD_TRAVERSAL\n");
			update<pagerank_vert_attr> * ret;
			ret = new update<pagerank_vert_attr>;
            ret->dest_vert = this_edge->get_dest_value();

            assert(this->forward_backward_phase == FORWARD_TRAVERSAL);
            float scatter_weight = DAMPING_FACTOR *(this_vert->rank/num_edges) + (1- DAMPING_FACTOR);
            ret->vert_attr.rank = scatter_weight;
			return ret;
		}

		// Gather one update. Explain the parameters:
		// vid: the vertex id of destination vertex;
		// va: the attribute of destination vertex;
		// u: the update from the "update" buffer.
		void gather_one_update( u32_t vid, pagerank_vert_attr * dest_vert_attr, update<pagerank_vert_attr> * u )
		{
			assert( vid == u->dest_vert );
			dest_vert_attr->rank += u->vert_attr.rank;
		}

        void before_iteration()
        {
            PRINT_DEBUG("PageRank engine is running for the %d-th iteration, there are %d tasks to schedule!\n",
                    this->loop_counter, this->num_tasks_to_sched);
        }
        int after_iteration()
        {
            reduce_iters = iteration_times - this->loop_counter;
            PRINT_DEBUG("Pagerank engine has finished the %d-th iteration!\n", this->loop_counter);
            if (reduce_iters == 0)
                return ITERATION_STOP;
            return ITERATION_CONTINUE;
        }
        int finalize(pagerank_vert_attr * va)
        {
            for (unsigned int id = 0; id < 100; id++)
                PRINT_DEBUG_LOG("Pagerank:result[%d], rank = %f\n", id, (va+id)->rank);

            PRINT_DEBUG("Pagerank engine stops!\n");
            return ENGINE_STOP;
        }
};
/*
template <typename T>
bool pagerank_program<T>::set_forward_backward = false;

template <typename T>
u32_t pagerank_program<T>::num_tasks_to_sched = 0;

template <typename T>
u32_t pagerank_program<T>::iteration_times = 0;

template <typename T>
u32_t pagerank_program<T>::reduce_iters = 0;

template <typename T>
int pagerank_program<T>::forward_backward_phase = FORWARD_TRAVERSAL;

template <typename T>
int pagerank_program<T>::CONTEXT_PHASE = 0;

template <typename T>
int pagerank_program<T>::loop_counter = 0;

template <typename T>
bool pagerank_program<T>::init_sched = false;
//if you want add_schedule when init(), please set this to be true~!
*/

template <typename T>
void start_engine()
{
    /*
    pagerank_program<T> *pr_obj = new pagerank_program<T>();
    pr_obj->iteration_times = vm["pagerank::niters"].as<unsigned long>();
    unsigned int iteration_times = pr_obj->iteration_times;
    if(10 == iteration_times)
    {
        std::cout<<"You didn't input the pagerank::niters or you chose the default value:10, the algorithm will run 10 iterations."<<std::endl;
    }
    PRINT_DEBUG( "pagerank_program iteration_times = %d\n", pr_obj->iteration_times );
    //ready and run
    fog_engine<pagerank_program<T>, pagerank_vert_attr, pagerank_vert_attr, T> * eng;
    (*(eng = new fog_engine<pagerank_program<T>, pagerank_vert_attr, pagerank_vert_attr, T>(GLOBAL_ENGINE, pr_obj)))();
    delete eng;
    */

    Fog_program<pagerank_vert_attr, pagerank_vert_attr, T> *pr_ptr = new pagerank_program<T>(FORWARD_TRAVERSAL, false);
    ((pagerank_program<T>*)pr_ptr)->iteration_times = 1;
    unsigned int iteration_times = vm["pagerank::niters"].as<unsigned long>();
    if(10 == iteration_times)
    {
        std::cout<<"You didn't input the pagerank::niters or you chose the default value:10, the algorithm will run 10 iterations."<<std::endl;
    }
    PRINT_DEBUG( "pagerank_program iteration_times = %d\n", iteration_times);

    fog_engine<pagerank_program<T>, pagerank_vert_attr, pagerank_vert_attr, T> * eng;
    eng = new fog_engine<pagerank_program<T>, pagerank_vert_attr, pagerank_vert_attr, T>(GLOBAL_ENGINE, pr_ptr);

    for(u32_t i = 0; i < iteration_times; i++)
    {
        PRINT_DEBUG_LOG("task %d!\n", i);
        Fog_program<pagerank_vert_attr, pagerank_vert_attr, T> *pr_ptr = new pagerank_program<T>(FORWARD_TRAVERSAL, false);
        ((pagerank_program<T>*)pr_ptr)->iteration_times = 1;

        struct task_config * task_config_ptr = new struct task_config;
        task_config_ptr->min_vert_id = gen_config.min_vert_id;
        task_config_ptr->max_vert_id = gen_config.max_vert_id;
        task_config_ptr->num_edges = gen_config.num_edges;
        Fog_task< Fog_program<pagerank_vert_attr, pagerank_vert_attr, T> > *task
            = new Fog_task< Fog_program<pagerank_vert_attr, pagerank_vert_attr, T> >(pr_ptr, i, task_config_ptr);
        eng->run_task(task);

        delete pr_ptr;
        delete task;
        delete task_config_ptr;
    }
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




