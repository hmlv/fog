/**************************************************************************************************
 * Authors: 
 *   Jian He
 *
 * Routines:
 *   Implements strongly connected component algorithm
 *   
 *************************************************************************************************/


#include "fog_adapter.h"

struct scc_vert_attr{
	u32_t prev_root;
	u32_t component_root;
    bool found_component;
}__attribute__((__packed__));

struct scc_update
{
    u32_t component_root;
};

template <typename T>
class scc_program{
	public:
        u32_t num_tasks_to_sched;
        int do_scc_phase;
        int forward_backward_phase;
        int CONTEXT_PHASE;
        int loop_counter;
        bool init_sched;
        
        int out_loop;

        scc_program()
        {
            num_tasks_to_sched = 0;
            out_loop = 0;
            forward_backward_phase = FORWARD_TRAVERSAL;
            CONTEXT_PHASE = 0;
            loop_counter = 0;
            init_sched = true;
        }

        void init( u32_t vid, scc_vert_attr* va, index_vert_array<T> * vert_index)
        {
            if (out_loop == 0 && forward_backward_phase == FORWARD_TRAVERSAL)
            {
                if (vert_index->num_edges(vid, OUT_EDGE) == 0 || vert_index->num_edges(vid, IN_EDGE) == 0)
                {
                    va->found_component = true;
                    va->prev_root = va->component_root = vid;
                }
                else
                {
                    //first init
                    va->component_root = vid;
                    va->prev_root = (u32_t)-1;
                    va->found_component = false;
                    //add schedule for cc work
                    fog_engine<scc_program<T>, scc_vert_attr, scc_update, T>::add_schedule( vid, 
                            CONTEXT_PHASE /*phase:decide which buf to read and write */
                            );
                }
            }
            else
            {
                if ( forward_backward_phase == FORWARD_TRAVERSAL )
                {
                    if (va->component_root != va->prev_root)
                    {
                        va->prev_root = va->component_root;
                        va->component_root = vid;
                        fog_engine<scc_program<T>, scc_vert_attr, scc_update, T>::add_schedule(vid, CONTEXT_PHASE);
                    }
                }
                else 
                {
                    assert(forward_backward_phase == BACKWARD_TRAVERSAL);
                    if (va->component_root != va->prev_root)
                    {
                        va->prev_root = va->component_root;
                        va->component_root = vid;
                        if (va->component_root == va->prev_root)
                        {
                            va->found_component = true;
                            fog_engine<scc_program<T>, scc_vert_attr, scc_update, T>::add_schedule(vid, CONTEXT_PHASE);
                        }
                    }
                    else //(va->component_root == va->prev_root)
                    {
                        assert(va->component_root == va->prev_root);
                        if(va->found_component == false)
                        {
                            va->found_component = true;
                            fog_engine<scc_program<T>, scc_vert_attr, scc_update, T>::add_schedule(vid, CONTEXT_PHASE);
                        } 
                    }
                }
            }
		}

		//scatter updates at vid-th vertex 
		update<scc_update> *scatter_one_edge(
                scc_vert_attr * this_vert,
                T * this_edge,
                u32_t backward_update_dest)
        {
            update<scc_update> *ret;
            ret = new update<scc_update>;
            if (forward_backward_phase == FORWARD_TRAVERSAL) 
                ret->dest_vert = this_edge->get_dest_value();
            else 
            {
                assert (forward_backward_phase == BACKWARD_TRAVERSAL);
                ret->dest_vert = backward_update_dest;
            }
            ret->vert_attr.component_root = this_vert->component_root;
            return ret;
		}
		//gather one update "u" from outside
		void gather_one_update( u32_t vid, scc_vert_attr* this_vert, 
                struct update<scc_update>* this_update)
        {
            /*
             * just gather everything
             */
            if (forward_backward_phase == FORWARD_TRAVERSAL)
            {
                if (this_update->vert_attr.component_root < this_vert->component_root && this_vert->found_component == false)
                {
                    this_vert->component_root = this_update->vert_attr.component_root;
                    fog_engine<scc_program<T>, scc_vert_attr, scc_update, T>::add_schedule(vid, CONTEXT_PHASE);
                }
            }
            else
            {
                assert (forward_backward_phase == BACKWARD_TRAVERSAL);
                if (this_update->vert_attr.component_root == this_vert->prev_root && this_vert->found_component == false)
                {
                    this_vert->component_root = this_update->vert_attr.component_root;
                    this_vert->found_component = true;
                    fog_engine<scc_program<T>, scc_vert_attr, scc_update, T>::add_schedule(vid, CONTEXT_PHASE);
                }
            }
		}
        void before_iteration()
        {
            if (forward_backward_phase == FORWARD_TRAVERSAL)
                PRINT_DEBUG("SCC engine is running FORWARD_TRAVERSAL for the %d-th iteration, there are %d tasks to schedule!\n",
                        loop_counter, num_tasks_to_sched);
            else
            {
                assert(forward_backward_phase == BACKWARD_TRAVERSAL);
                PRINT_DEBUG("SCC engine is running BACKWARD_TRAVERSAL for the %d-th iteration, there are %d tasks to schedule!\n",
                        loop_counter, num_tasks_to_sched);
            }
        }
        int after_iteration()
        {
            PRINT_DEBUG("SCC engine has finished the %d-th iteration, there are %d tasks to schedule at next iteration!\n",
                    loop_counter, num_tasks_to_sched);
            if (num_tasks_to_sched == 0)
                return ITERATION_STOP;
            else
                return ITERATION_CONTINUE;
        }
        int finalize(scc_vert_attr * va)
        {
            out_loop ++;
            //lhm test
            return ENGINE_STOP;
            //test end
            if (forward_backward_phase == FORWARD_TRAVERSAL)
            {
                forward_backward_phase = BACKWARD_TRAVERSAL;
                loop_counter = 0;
                CONTEXT_PHASE = 0;
                return ENGINE_CONTINUE;
            }
            else
            {
                assert(forward_backward_phase == BACKWARD_TRAVERSAL);
                forward_backward_phase = FORWARD_TRAVERSAL;
                loop_counter = 0;
                CONTEXT_PHASE = 0;
                return ENGINE_CONTINUE;
            }
        }
};

/*
template <typename T>
unsigned int scc_program<T>::num_tasks_to_sched = 0;

template <typename T>
int scc_program<T>::out_loop = 0;

template <typename T>
int scc_program<T>::do_scc_phase = 0;

 * forward_backward_phase is setup for backward algorithms,
 * we will let the fog-engine read in-edge for backward algorithms.
 * -1 for initilization, you can set in main() and finalize(). 
 * FORWARD_TRAVERSAL = 0
 * BACKWARD_TRAVERSAL = 1
template <typename T>
int scc_program<T>::forward_backward_phase = FORWARD_TRAVERSAL;

template <typename T>
int scc_program<T>::CONTEXT_PHASE = 0;

template <typename T>
int scc_program<T>::loop_counter = 0;

template <typename T>
bool scc_program<T>::init_sched = true;
//if you want add_schedule when init(), please set this to be true~!

template <typename T>
bool scc_program<T>::set_forward_backward = false;
*/

template <typename T>
void start_engine() 
{
    scc_program<T> * scc_obj = new scc_program<T>();
    int check = access(gen_config.in_edge_file_name.c_str(), F_OK);
    if(-1 ==check )
    {
        PRINT_ERROR("in_edge file doesn't exit or '-i' is false!\n");
    }

    fog_engine<scc_program<T>, scc_vert_attr, scc_update, T> * eng;
    (*(eng = new fog_engine<scc_program<T>, scc_vert_attr, scc_update, T>(TARGET_ENGINE, scc_obj)))();
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
