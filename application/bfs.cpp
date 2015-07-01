/**************************************************************************************************
 * Authors: 
 *   Jian He,
 *
 * Routines:
 *   Implements Breadth-First Search algorithm
 *   
 *************************************************************************************************/


#include "fog_adapter.h"
#include "fog_program.h"
#include "fog_task.h"
#include "../fogsrc/fog_task.cpp"
    

struct bfs_vert_attr{
	u32_t bfs_level;
};

template <typename T>
class bfs_program{
	public:
        u32_t num_tasks_to_sched;
		u32_t bfs_root;
        int forward_backward_phase;
        int CONTEXT_PHASE;
        int loop_counter;
        bool init_sched;
        bool set_forward_backward;


        bfs_program()
        {
            num_tasks_to_sched = 0;
            bfs_root = 0;
            forward_backward_phase = FORWARD_TRAVERSAL;
            CONTEXT_PHASE = 0;
            loop_counter = 0;
            init_sched = false;

        }
		//init the vid-th vertex
		void init(u32_t vid, bfs_vert_attr* va, index_vert_array<T> * vert_index){
			if ( vid == bfs_root){
				va->bfs_level = 0;
                //PRINT_DEBUG("VID = %d\n", vid);
				fog_engine<bfs_program<T>, bfs_vert_attr, bfs_vert_attr, T>::add_schedule( vid, 
                        CONTEXT_PHASE /*phase:decide which buf to read and write */
                        );
			}
            else
            { 
				va->bfs_level = UINT_MAX;
            }
		}

		//scatter updates at vid-th vertex 
        /*
         * params:
         * 1.attribute
         * 2.edge(type1, type2, in_edge)
         * 3.update_src
         */
		update<bfs_vert_attr> *scatter_one_edge(
                bfs_vert_attr * this_vert,
                T * this_edge,
                u32_t update_src)
        {
            assert(forward_backward_phase == FORWARD_TRAVERSAL);
            update<bfs_vert_attr> *ret;
            u32_t scatter_value = this_vert->bfs_level + 1;
            ret = new update<bfs_vert_attr>; 
            ret->dest_vert = this_edge->get_dest_value();
            ret->vert_attr.bfs_level = scatter_value;
            return ret;
		}

		//gather one update "u" from outside
		void gather_one_update( u32_t vid, bfs_vert_attr* this_vert, 
                struct update<bfs_vert_attr>* this_update)
        {
			//compare the value of u, if it is smaller, absorb the update
			if( this_update->vert_attr.bfs_level < this_vert->bfs_level){
				*this_vert = this_update->vert_attr;
				//should add schedule of {vid,0}, need api from engine
				fog_engine<bfs_program<T>, bfs_vert_attr, bfs_vert_attr, T>::add_schedule( vid, CONTEXT_PHASE);
			}
		}

        void before_iteration()
        {
            PRINT_DEBUG("BFS engine is running for the %d-th iteration, there are %d tasks to schedule!\n",
                    loop_counter, num_tasks_to_sched);
        }
        int after_iteration()
        {
            PRINT_DEBUG("BFS engine has finished the %d-th iteration, there are %d tasks to schedule at next iteration!\n",
                    loop_counter, num_tasks_to_sched);
            if (num_tasks_to_sched == 0)
            {
                if(1 == loop_counter)
                {
                    PRINT_ERROR("The bfs::root is bad, there is no out-edge of bfs::root=%d!\n", bfs_root);
                }
                return ITERATION_STOP;
            }
            else
            {
                return ITERATION_CONTINUE;
            }
        }
        int finalize(bfs_vert_attr * va)
        {
            for (unsigned int id = 0; id < 100; id++)
                PRINT_DEBUG_LOG("BFS:result[%d], bfs_level = %d\n", id, (va+id)->bfs_level);
            PRINT_DEBUG("BFS engine stops!\n");
            return ENGINE_STOP;
        }
};
/*
template <typename T>
unsigned int bfs_program<T>::num_tasks_to_sched = 0;

template <typename T>
unsigned int bfs_program<T>::bfs_root = 0;

template <typename T>
int bfs_program<T>::forward_backward_phase = FORWARD_TRAVERSAL;

template <typename T>
int bfs_program<T>::CONTEXT_PHASE = 0;

template <typename T>
int bfs_program<T>::loop_counter = 0;

template <typename T>
bool bfs_program<T>::set_forward_backward = false;

template <typename T>
bool bfs_program<T>::init_sched = false;
//if you want add_schedule when init(), please set this to be true~!
*/

template <typename T>
void start_engine()
{
    /*
    bfs_program<T> *bfs_obj = new bfs_program<T>;
    bfs_obj->bfs_root = vm["bfs::bfs-root"].as<unsigned long>();
    unsigned int root_vid = bfs_obj->bfs_root;
    if(0 == root_vid)
    {
        std::cout<<"You didn't input the bfs::bfs_root or you chose the default value:0, the program will start at bfs_root=0."<<std::endl;
    }
    PRINT_DEBUG("bfs_program bfs_root = %d\n", bfs_obj->bfs_root);
    fog_engine<bfs_program<T>, bfs_vert_attr, bfs_vert_attr, T> *eng;
    (*(eng = new fog_engine<bfs_program<T>, bfs_vert_attr, bfs_vert_attr, T>(TARGET_ENGINE, bfs_obj)))();
    delete eng;
    */
    bfs_program<T> *bfs_obj = new bfs_program<T>;
    fog_engine<bfs_program<T>, bfs_vert_attr, bfs_vert_attr, T> *eng;
    eng = new fog_engine<bfs_program<T>, bfs_vert_attr, bfs_vert_attr, T>(TARGET_ENGINE, bfs_obj);
    for(int i = 0; i < 5; i++)
    {
        bfs_program<T> *bfs_obj = new bfs_program<T>;
        bfs_obj->bfs_root = i;
        PRINT_DEBUG("bfs_program bfs_root = %d\n", bfs_obj->bfs_root);

        struct task_config * task_config_ptr = new struct task_config;
        task_config_ptr->min_vert_id = gen_config.min_vert_id;
        task_config_ptr->max_vert_id = gen_config.max_vert_id;
        task_config_ptr->num_edges = gen_config.num_edges;
        Fog_task< bfs_program<T> > * task = new Fog_task< bfs_program<T> >(bfs_obj, i, task_config_ptr);
        eng->run_task(task);
        delete bfs_obj;
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
