/**************************************************************************************************
 * Authors: 
 *   Zhiyuan Shao, Jian He
 *
 * Routines:
 *   Implements the single source shortest path algorithm.
 *************************************************************************************************/

#include "fog_adapter.h"

struct sssp_vert_attr{
	u32_t predecessor;
	float	value;
};

template <typename T>
class sssp_program{
	public:
        static u32_t num_tasks_to_sched;
		static u32_t start_vid;
        static int forward_backward_phase;
        static int CONTEXT_PHASE;
        static int loop_counter;
        static bool init_sched;
        static bool set_forward_backward;
		//init the vid-th vertex
		static void init(u32_t vid, sssp_vert_attr* va, index_vert_array<T> * vert_index){
			if ( vid == start_vid ){
				va->value = 0;
                //PRINT_DEBUG("VID = %d\n", vid);
				fog_engine<sssp_program<T>, sssp_vert_attr, sssp_vert_attr, T>::add_schedule( vid, 
                        CONTEXT_PHASE /*phase:decide which buf to read and write */
                        );
			}
            else
            { 
				va->value = INFINITY;
            }
                va->predecessor = (u32_t)-1;
		}

		//scatter updates at vid-th vertex 
        /*
         * params:
         * 1.attribute
         * 2.edge(type1, type2, in_edge)
         * 3.update_src
         */
		static update<sssp_vert_attr> *scatter_one_edge(
                sssp_vert_attr * this_vert,
                T * this_edge,
                u32_t update_src)
        {
            assert(forward_backward_phase == FORWARD_TRAVERSAL);
            update<sssp_vert_attr> *ret;
            float scatter_weight = this_vert->value + this_edge->get_edge_value();
            u32_t scatter_predecessor = update_src;
            ret = new update<sssp_vert_attr>; 
            ret->dest_vert = this_edge->get_dest_value();
            ret->vert_attr.value = scatter_weight;
            ret->vert_attr.predecessor = scatter_predecessor;
            return ret;
		}

		//gather one update "u" from outside
		static void gather_one_update( u32_t vid, sssp_vert_attr* this_vert, 
                struct update<sssp_vert_attr>* this_update)
        {
			//compare the value of u, if it is smaller, absorb the update
            if (FLOAT_EQ(this_update->vert_attr.value, this_vert->value) == 0)
			if( this_update->vert_attr.value < this_vert->value ){
				*this_vert = this_update->vert_attr;
				//should add schedule of {vid,0}, need api from engine
				fog_engine<sssp_program<T>, sssp_vert_attr, sssp_vert_attr, T>::add_schedule( vid, CONTEXT_PHASE);
                    //PRINT_DEBUG("this_update.value = %f, this_vert->value = %f\n", this_update->vert_attr.value, this_vert->value);
			}
		}

        static void before_iteration()
        {
            PRINT_DEBUG("SSSP engine is running for the %d-th iteration, there are %d tasks to schedule!\n",
                    loop_counter, num_tasks_to_sched);
        }
        static int after_iteration()
        {
            PRINT_DEBUG("SSSP engine has finished the %d-th iteration, there are %d tasks to schedule at next iteration!\n",
                    loop_counter, num_tasks_to_sched);
            if (num_tasks_to_sched == 0)
            {
                if(1 == loop_counter)
                {
                    PRINT_ERROR("The sssp::source is bad, there is no out-edge of sssp::source=%d\n!",start_vid);
                }
                return ITERATION_STOP;
            }
            else
            {
                return ITERATION_CONTINUE;
            }
        }
        static int finalize(sssp_vert_attr * va)
        {
            for (unsigned int id = 0; id < 100; id++)
                PRINT_DEBUG_LOG("SSSP:result[%d], predecessor = %d, value = %f\n", id, (va+id)->predecessor, (va+id)->value);
            PRINT_DEBUG("SSSP engine stops!\n");
            return ENGINE_STOP;
        }

};

template <typename T>
unsigned int sssp_program<T>::num_tasks_to_sched = 0;

template <typename T>
unsigned int sssp_program<T>::start_vid = 0;

template <typename T>
int sssp_program<T>::forward_backward_phase = FORWARD_TRAVERSAL;

template <typename T>
int sssp_program<T>::CONTEXT_PHASE = 0;

template <typename T>
int sssp_program<T>::loop_counter = 0;

template <typename T>
bool sssp_program<T>::set_forward_backward = false;

template <typename T>
bool sssp_program<T>::init_sched = false;
//if you want add_schedule when init(), please set this to be true~!

template<typename T>
void start_engine()
{
    sssp_program<T>::start_vid = vm["sssp::source"].as<unsigned long>();
    unsigned int start_vid = sssp_program<T>::start_vid;
    if(0 == start_vid)
    {
        std::cout<<"You didn't input the sssp::source or you chose the default value:0, the program will start at start_vid=0."<<std::endl;
    }
    if(8 != sizeof(T))
    {
        PRINT_ERROR("This algorithm need 'type1' edge file!\n");
    }
    PRINT_DEBUG( "sssp_program start_vid = %d\n", sssp_program<T>::start_vid );
    //ready and run
    fog_engine<sssp_program<T>, sssp_vert_attr, sssp_vert_attr, T> *eng;
    (*(eng = new fog_engine<sssp_program<T>, sssp_vert_attr, sssp_vert_attr, T>(TARGET_ENGINE)))();
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
