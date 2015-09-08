/**************************************************************************************************
 * Authors: 
 *   Huiming Lv 
 *
 * Routines:
 *   Implements strongly connected component algorithm
 *   
 *************************************************************************************************/


#include "fog_adapter.h"
#include "fog_program.h"
#include "fog_task.h"
#include "../fogsrc/fog_task.cpp"
#include "filter.h"
#include "../fogsrc/filter.cpp"
#include "convert.h"
#include "config_parse.h"
#include <vector>
#include <stdio.h>
#include <memory.h>

//#define TRIM 

/**************************************************************************************************
 *TRIM
 **************************************************************************************************/

struct trim_vert_attr
{
    u32_t in_degree;
    u32_t out_degree;
    bool is_trim; 
};

struct trim_update
{

};

template <typename T>
class trim_program : public Fog_program<trim_vert_attr, trim_update, T> 
{
    public:
        int out_loop;

        trim_program(int p_forward_backward_phase, bool p_init_sched):Fog_program<trim_vert_attr, trim_update, T>(p_forward_backward_phase, p_init_sched)
        {
            out_loop = 0;
        }

        void init( u32_t vid, trim_vert_attr * va, index_vert_array<T> * vert_index) 
        {
            if(out_loop == 0)
            {
                va->in_degree  = vert_index->num_edges(vid, IN_EDGE);
                va->out_degree = vert_index->num_edges(vid, OUT_EDGE);
                if(0==va->in_degree && 0==va->out_degree)
                {
                    va->is_trim = true;
                }
                else
                {
                    va->is_trim = false;
                }
            }
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
            {
                if(0==va->in_degree && va->is_trim==false)
                {
                    va->is_trim = true;
                    fog_engine<trim_vert_attr, trim_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                }
            }
            else
            {
                if(0==va->out_degree && va->is_trim==false)
                {
                    va->is_trim = true;
                    fog_engine<trim_vert_attr, trim_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                }
            }
        }

        //scatter updates at vid-th vertex 
        void scatter_one_edge(
                trim_vert_attr * this_vert,
                T &this_edge,
                u32_t backward_update_dest,
                update<trim_update> &this_update)
        {
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
            { 
                this_update.dest_vert = this_edge.get_dest_value();
            }
            else 
            {
                assert (this->forward_backward_phase == BACKWARD_TRAVERSAL);
                this_update.dest_vert = backward_update_dest;
            }
            //ret->vert_attr.
            //return ret;
		}

		//gather one update "u" from outside
		void gather_one_update( u32_t vid, trim_vert_attr* this_vert, 
                struct update<trim_update>* this_update)
        {
            /*
             * just gather everything
             */
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
            {
                if(this_vert->is_trim == false)
                {
                    this_vert->in_degree = this_vert->in_degree - 1;
                    if(0==this_vert->in_degree)
                    {
                        this_vert->is_trim = true; 
                        fog_engine<trim_vert_attr, trim_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                    }
                }
            }
            else
            {
                assert (this->forward_backward_phase == BACKWARD_TRAVERSAL);
                if(this_vert->is_trim == false)
                {
                    this_vert->out_degree = this_vert->out_degree - 1;
                    if(0==this_vert->out_degree)
                    {
                        this_vert->is_trim = true;
                        fog_engine<trim_vert_attr, trim_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                    }
                }
            }
		}
        void before_iteration()
        {
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
                PRINT_DEBUG("TRIM engine is running FORWARD_TRAVERSAL for the %d-th iteration, there are %d tasks to schedule!\n",
                        this->loop_counter, this->num_tasks_to_sched);
            else
            {
                assert(this->forward_backward_phase == BACKWARD_TRAVERSAL);
                PRINT_DEBUG("TRIM engine is running BACKWARD_TRAVERSAL for the %d-th iteration, there are %d tasks to schedule!\n",
                        this->loop_counter, this->num_tasks_to_sched);
            }
        }
        int after_iteration()
        {
            PRINT_DEBUG("TRIM engine has finished the %d-th iteration, there are %d tasks to schedule at next iteration!\n",
                    this->loop_counter, this->num_tasks_to_sched);
            if (this->num_tasks_to_sched == 0)
                return ITERATION_STOP;
            else
                return ITERATION_CONTINUE;
        }
        int finalize(trim_vert_attr * va)
        {
            out_loop ++;
            if (out_loop==2)
            {
                return ENGINE_STOP;
            }
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
            {
                this->forward_backward_phase = BACKWARD_TRAVERSAL;
                this->loop_counter = 0;
                this->CONTEXT_PHASE = 0;
                return ENGINE_CONTINUE;
            }
            else
            {
                assert(this->forward_backward_phase == BACKWARD_TRAVERSAL);
                this->forward_backward_phase = FORWARD_TRAVERSAL;
                this->loop_counter = 0;
                this->CONTEXT_PHASE = 0;
                return ENGINE_CONTINUE;
            }
        }

};


/***************************************************************************************************
 *BFS_based
 ****************************************************************************************************/

//fw_bw_label = 0, start
//fw_bw_label = 1, FW
//fw_bw_label = 2, is_found = TRUE,  SCC
//fw_bw_label = 2, is_found = FALSE, BW
//fw_bw_label = 3, TRIM 
struct scc_vert_attr{
	u32_t fw_bw_label;
	//u32_t bw_label;
    bool is_found;
}__attribute__((__packed__));

struct scc_update
{
    u32_t fw_bw_label;
};

template <typename T>
class scc_fb_program : public Fog_program<scc_vert_attr, scc_update, T>
{
	public:
        int out_loop;
        u32_t m_pivot; 

        scc_fb_program(int p_forward_backward_phase, bool p_init_sched, u32_t pivot):Fog_program<scc_vert_attr, scc_update, T>(p_forward_backward_phase, p_init_sched)
        {
            out_loop = 0;
            m_pivot = pivot; 
        }

        void init( u32_t vid, scc_vert_attr* va, index_vert_array<T> * vert_index)
        {
            if (out_loop == 0 && this->forward_backward_phase == FORWARD_TRAVERSAL)
            {
                if(vid==m_pivot)
                {
                    std::cout<<vert_index->num_edges(vid, OUT_EDGE)<<std::endl;
                    std::cout<<vert_index->num_edges(vid, IN_EDGE)<<std::endl;
                }

                if (vert_index->num_edges(vid, OUT_EDGE) == 0 || vert_index->num_edges(vid, IN_EDGE) == 0)
                {
                    va->is_found = true;
                    va->fw_bw_label = 3;
                }
                else
                {
                    va->is_found = false;
                    //if(vid == gen_config.min_vert_id)
                    if(vid == m_pivot)
                    {
                        va->fw_bw_label = 0;
                        fog_engine<scc_vert_attr, scc_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                    }
                    else 
                    {
                        va->fw_bw_label = UINT_MAX;
                    }
                }
            }
            else
            {
                if (this->forward_backward_phase == BACKWARD_TRAVERSAL )
                {
                    //if(vid == gen_config.min_vert_id)
                    if(vid == m_pivot)
                    {
                        va->is_found = true;
                        va->fw_bw_label = 2;
                        fog_engine<scc_vert_attr, scc_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                    }
                    else if(va->fw_bw_label==0)
                    {
                        va->fw_bw_label = 1;
                    }
                }
            }
		}

		//scatter updates at vid-th vertex 
		void scatter_one_edge(
                scc_vert_attr * this_vert,
                T &this_edge,
                u32_t backward_update_dest,
                update<scc_update> &this_update)
        {
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
            { 
                this_update.dest_vert = this_edge.get_dest_value();
            }
            else 
            {
                assert (this->forward_backward_phase == BACKWARD_TRAVERSAL);
                this_update.dest_vert = backward_update_dest;
            }
            this_update.vert_attr.fw_bw_label = this_vert->fw_bw_label;
            //return ret;
		}
		//gather one update "u" from outside
		void gather_one_update( u32_t vid, scc_vert_attr* this_vert, 
                struct update<scc_update>* this_update)
        {
            /*
             * just gather everything
             */
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
            {
                //if(this_update->vert_attr.fw_bw_label!=0)
                //{
                    //std::cout<<this_update->vert_attr.fw_bw_label<<std::endl;
                //}
                assert(this_update->vert_attr.fw_bw_label==0);
                if (this_vert->fw_bw_label==UINT_MAX)
                {
                    this_vert->fw_bw_label = this_update->vert_attr.fw_bw_label;
                    fog_engine<scc_vert_attr, scc_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                }
            }
            else
            {
                assert (this->forward_backward_phase == BACKWARD_TRAVERSAL);
                if (this_vert->fw_bw_label==UINT_MAX)
                {
                    this_vert->fw_bw_label = this_update->vert_attr.fw_bw_label;
                    fog_engine<scc_vert_attr, scc_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                }
                else if (this_vert->fw_bw_label==1 && this_vert->is_found == false)
                {
                    assert(this_update->vert_attr.fw_bw_label==2);
                    this_vert->fw_bw_label = this_update->vert_attr.fw_bw_label;
                    this_vert->is_found = true;
                    fog_engine<scc_vert_attr, scc_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                }
            }
		}
        void before_iteration()
        {
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
                PRINT_DEBUG("SCC engine is running FORWARD_TRAVERSAL for the %d-th iteration, there are %d tasks to schedule!\n",
                        this->loop_counter, this->num_tasks_to_sched);
            else
            {
                assert(this->forward_backward_phase == BACKWARD_TRAVERSAL);
                PRINT_DEBUG("SCC engine is running BACKWARD_TRAVERSAL for the %d-th iteration, there are %d tasks to schedule!\n",
                        this->loop_counter, this->num_tasks_to_sched);
            }
        }
        int after_iteration()
        {
            PRINT_DEBUG("SCC engine has finished the %d-th iteration, there are %d tasks to schedule at next iteration!\n",
                    this->loop_counter, this->num_tasks_to_sched);
            if (this->num_tasks_to_sched == 0)
                return ITERATION_STOP;
            else
                return ITERATION_CONTINUE;
        }
        int finalize(scc_vert_attr * va)
        {
            out_loop ++;
            if (out_loop==2)
            {
                //std::cout<<"finalize "<<va<<std::endl;
                return ITERATION_STOP;
            }
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
            {
                this->forward_backward_phase = BACKWARD_TRAVERSAL;
                this->loop_counter = 0;
                this->CONTEXT_PHASE = 0;
                return ENGINE_CONTINUE;
            }
            else
            {
                assert(this->forward_backward_phase == BACKWARD_TRAVERSAL);
                this->forward_backward_phase = FORWARD_TRAVERSAL;
                this->loop_counter = 0;
                this->CONTEXT_PHASE = 0;
                return ENGINE_CONTINUE;
            }
        }
};

/********************************************************************************************
 *minimal label propagation
 ********************************************************************************************/

struct scc_color_vert_attr
{
    u32_t prev_root;
    u32_t component_root;
    bool is_found;
}__attribute__((__packed__));

struct scc_color_update
{
    u32_t component_root;
};

template <typename T>
class scc_color_program : public Fog_program<scc_color_vert_attr, scc_color_update, T>
{
	public:
        int out_loop;

        scc_color_program(int p_forward_backward_phase, bool p_init_sched):Fog_program<scc_color_vert_attr, scc_color_update, T>(p_forward_backward_phase, p_init_sched)
        {
            out_loop = 0;
        }

        void init( u32_t vid, scc_color_vert_attr* va, index_vert_array<T> * vert_index)
        {
            if (out_loop == 0 && this->forward_backward_phase == FORWARD_TRAVERSAL)
            {

                if (vert_index->num_edges(vid, OUT_EDGE) == 0 || vert_index->num_edges(vid, IN_EDGE) == 0)
                {
                    va->is_found = true;
                    va->prev_root = va->component_root = vid;
                }
                else
                {
                    va->is_found = false;
                    va->component_root = vid;
                    va->prev_root = (u32_t)-1;
                    fog_engine<scc_color_vert_attr, scc_color_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                }
            }
            else
            {
                if (this->forward_backward_phase == FORWARD_TRAVERSAL )
                {
                    if (va->component_root != va->prev_root)
                    {
                        va->prev_root = va->component_root;
                        va->component_root = vid;
                        fog_engine<scc_color_vert_attr, scc_color_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                    }
                }
                else
                {
                    assert(this->forward_backward_phase == BACKWARD_TRAVERSAL);
                    if (va->component_root != va->prev_root)
                    {
                        va->prev_root = va->component_root;
                        va->component_root = vid;
                        if (va->component_root == va->prev_root)
                        {
                            va->is_found = true;
                            fog_engine<scc_color_vert_attr, scc_color_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                        }
                    }
                    else
                    {
                        assert(va->component_root == va->prev_root);
                        if (va->is_found == false)
                        {
                            va->is_found = true;
                            fog_engine<scc_color_vert_attr, scc_color_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                        }
                    }
                }
            }
		}

		//scatter updates at vid-th vertex 
		void scatter_one_edge(
                scc_color_vert_attr * this_vert,
                T &this_edge,
                u32_t backward_update_dest,
                update<scc_color_update> &this_update)
        {
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
            { 
                this_update.dest_vert = this_edge.get_dest_value();
            }
            else 
            {
                assert (this->forward_backward_phase == BACKWARD_TRAVERSAL);
                this_update.dest_vert = backward_update_dest;
            }
            this_update.vert_attr.component_root = this_vert->component_root;
            //return ret;
		}
		//gather one update "u" from outside
		void gather_one_update( u32_t vid, scc_color_vert_attr* this_vert, 
                struct update<scc_color_update>* this_update)
        {
            // just gather everything
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
            {
                if (this_update->vert_attr.component_root < this_vert->component_root && this_vert->is_found == false)
                {
                    this_vert->component_root = this_update->vert_attr.component_root;
                    fog_engine<scc_color_vert_attr, scc_color_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                }
            }
            else
            {
                assert (this->forward_backward_phase == BACKWARD_TRAVERSAL);
                if (this_update->vert_attr.component_root == this_vert->prev_root && this_vert->is_found == false)
                {
                    this_vert->component_root = this_update->vert_attr.component_root;
                    this_vert->is_found = true;
                    fog_engine<scc_color_vert_attr, scc_color_update, T>::add_schedule(vid, this->CONTEXT_PHASE);
                }
            }
		}
        void before_iteration()
        {
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
                PRINT_DEBUG("SCC engine is running FORWARD_TRAVERSAL for the %d-th iteration, there are %d tasks to schedule!\n",
                        this->loop_counter, this->num_tasks_to_sched);
            else
            {
                assert(this->forward_backward_phase == BACKWARD_TRAVERSAL);
                PRINT_DEBUG("SCC engine is running BACKWARD_TRAVERSAL for the %d-th iteration, there are %d tasks to schedule!\n",
                        this->loop_counter, this->num_tasks_to_sched);
            }
        }
        int after_iteration()
        {
            PRINT_DEBUG("SCC engine has finished the %d-th iteration, there are %d tasks to schedule at next iteration!\n",
                    this->loop_counter, this->num_tasks_to_sched);
            if (this->num_tasks_to_sched == 0)
                return ITERATION_STOP;
            else
                return ITERATION_CONTINUE;
        }
        int finalize(scc_color_vert_attr * va)
        {
            out_loop ++;
            if (this->forward_backward_phase == FORWARD_TRAVERSAL)
            {
                this->forward_backward_phase = BACKWARD_TRAVERSAL;
                this->loop_counter = 0;
                this->CONTEXT_PHASE = 0;
                return ENGINE_CONTINUE;
            }
            else
            {
                assert(this->forward_backward_phase == BACKWARD_TRAVERSAL);
                this->forward_backward_phase = FORWARD_TRAVERSAL;
                this->loop_counter = 0;
                this->CONTEXT_PHASE = 0;
                return ENGINE_CONTINUE;
            }
        }
};


template <typename T>
//u32_t select_pivot(const index_vert_array<T> * vert_index)
u32_t select_pivot(const struct task_config * p_task_config)
{
    gen_config.min_vert_id = p_task_config->min_vert_id;
    gen_config.max_vert_id = p_task_config->max_vert_id;
    gen_config.num_edges = p_task_config->num_edges;
    gen_config.vert_file_name = p_task_config->vert_file_name;
    gen_config.edge_file_name = p_task_config->edge_file_name;
    gen_config.attr_file_name = p_task_config->attr_file_name;
    gen_config.in_vert_file_name = p_task_config->in_vert_file_name;
    gen_config.in_edge_file_name = p_task_config->in_edge_file_name;
    index_vert_array<T> * vert_index = new index_vert_array<T>;
    u64_t temp_deg = 0;
    u64_t max_deg = 0;
    u32_t pivot = 0;
    for (u32_t i = gen_config.min_vert_id; i <= gen_config.max_vert_id; i++)
    {
        temp_deg = vert_index->num_edges(i, IN_EDGE) * vert_index->num_edges(i, OUT_EDGE);
        if(temp_deg > max_deg)
        {
            pivot = i;
            max_deg = temp_deg;
        }

    }
    delete vert_index;
    return pivot; 
}

template <typename T>
void start_engine() 
{
    int check = access(gen_config.in_edge_file_name.c_str(), F_OK);
    if(-1 ==check )
    {
        PRINT_ERROR("in_edge file doesn't exit or '-i' is false!\n");
    }
    
    TASK_ID = 0; 
    std::queue< Fog_task<scc_vert_attr, scc_update, T> * > fb_queue_task; 
    std::queue< Fog_task<scc_color_vert_attr, scc_color_update, T> * > color_queue_task; 
    
#ifdef TRIM
    /*
     *TRIM
     */
    Fog_program<trim_vert_attr, trim_update, T> * trim_ptr = new trim_program<T>(FORWARD_TRAVERSAL, true);
    fog_engine<trim_vert_attr, trim_update, T> * eng_trim = new fog_engine<trim_vert_attr, trim_update, T>(TARGET_ENGINE, trim_ptr);
    (*eng_trim)();
    //TRIM_filter->do_trim_filter(eng_trim->get_attr_array_header(), eng_trim->get_vert_index(), TASK_ID);
    Filter<trim_vert_attr> * TRIM_filter = new Filter<trim_vert_attr>();
    TRIM_filter->do_trim_filter(eng_trim->get_attr_array_header(), TASK_ID);
    delete TRIM_filter;

    //return;

    while(!queue_bag_config.empty())
    {
        struct bag_config b_config = queue_bag_config.front();
        queue_bag_config.pop();
        if(b_config.data_size==0)
        {
            continue;
        }
        //remain to do : use the bag.data_size to decide which algorithm is the best(suitable) 
        Fog_task<scc_vert_attr, scc_update, T> * m_task = new Fog_task<scc_vert_attr, scc_update, T>();
        m_task->set_task_id(b_config.bag_id);
        struct task_config * ptr_sub_task_config = new struct task_config;
        std::cout<<b_config.bag_id<<std::endl;
        std::cout<<b_config.data_size<<std::endl;
        std::cout<<b_config.data_name<<std::endl;
        std::string desc_data_name = m_task->create_dataset(&b_config, eng_trim->get_vert_index());

        init_graph_desc(desc_data_name);

        ptr_sub_task_config->min_vert_id = pt.get<u32_t>("description.min_vertex_id");
        ptr_sub_task_config->max_vert_id = pt.get<u32_t>("description.max_vertex_id");
        ptr_sub_task_config->num_edges = pt.get<u32_t>("description.num_of_edges");
        ptr_sub_task_config->is_remap = true;
        ptr_sub_task_config->remap_file_name = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".remap";
        ptr_sub_task_config->vert_file_name  = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".index";
        ptr_sub_task_config->edge_file_name  = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".edge";
        ptr_sub_task_config->attr_file_name  = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".attr";
        //ptr_sub_task_config->with_in_edge = ;
        if(gen_config.with_in_edge)
        {
            ptr_sub_task_config->in_vert_file_name = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".in_index";
            ptr_sub_task_config->in_edge_file_name = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".in_edge";
        }
        m_task->set_task_config(ptr_sub_task_config);
        
        //remain to do : use the bag.data_size to decide which algorithm is the best(suitable) 
        fb_queue_task.push(m_task);
    }
    delete trim_ptr;
    delete eng_trim;

#endif

    
#ifndef TRIM
    //task 0
    struct task_config * ptr_task_config = new struct task_config;
    Fog_task<scc_vert_attr, scc_update, T> *task = new Fog_task<scc_vert_attr, scc_update, T>();
    ptr_task_config->min_vert_id = gen_config.min_vert_id;
    ptr_task_config->max_vert_id = gen_config.max_vert_id;
    ptr_task_config->num_edges = gen_config.num_edges;
    ptr_task_config->is_remap = false;
    ptr_task_config->graph_path = gen_config.graph_path;
    ptr_task_config->vert_file_name = gen_config.vert_file_name;
    ptr_task_config->edge_file_name = gen_config.edge_file_name;
    ptr_task_config->attr_file_name = gen_config.attr_file_name;
    ptr_task_config->in_vert_file_name = gen_config.in_vert_file_name;
    ptr_task_config->in_edge_file_name = gen_config.in_edge_file_name;
    ptr_task_config->with_in_edge = gen_config.with_in_edge;
    task->set_task_config(ptr_task_config);
    task->set_task_id(0);
    assert(fb_queue_task.empty());
    fb_queue_task.push(task);
#endif
   

    //Fog_program<scc_vert_attr, scc_update, T> *scc_ptr = new scc_fb_program<T>(FORWARD_TRAVERSAL, true, 0);
    fog_engine<scc_vert_attr, scc_update, T> * eng_fb;
    eng_fb = new fog_engine<scc_vert_attr, scc_update, T>(TARGET_ENGINE);

    //fb_queue_task_id.push(TASK_ID);
    //while(!fb_queue_task_id.empty())
    while(!fb_queue_task.empty())
    {
        //temp_task_id = fb_queue_task_id.front();
        //std::cout<<"task : "<<temp_task_id<<std::endl;
        //fb_queue_task_id.pop();
        struct Fog_task<scc_vert_attr, scc_update, T> * main_task = fb_queue_task.front();
        fb_queue_task.pop();
        PRINT_DEBUG("*********************************** task %d****************************************\n", main_task->get_task_id());

        u32_t pivot = select_pivot<T>(main_task->m_task_config);
        //u32_t pivot = select_pivot<T>(eng_fb->get_vert_index());
        Fog_program<scc_vert_attr, scc_update, T> *scc_ptr = new scc_fb_program<T>(FORWARD_TRAVERSAL, true, pivot);

        //index_vert_array<type1_edge> * vert_index = new index_vert_array<type1_edge>;
        //if(typeid(*scc_ptr) == typeid(scc_fb_program<T>))
        //{
        //}
        main_task->set_alg_ptr(scc_ptr);
        eng_fb->run_task(main_task);
        
        //filter->do_scc_filter(eng_fb->get_attr_array_header(), eng_fb->get_vert_index(), main_task->get_task_id());
        Filter<scc_vert_attr> * filter = new Filter<scc_vert_attr>();
        filter->do_scc_filter(eng_fb->get_attr_array_header(), main_task->get_task_id());
        delete filter;

        while(!queue_bag_config.empty())
        {
            struct bag_config b_config = queue_bag_config.front();
            queue_bag_config.pop();
            if(b_config.data_size==0)
            {
                continue;
            }
            //remain to do : use the bag.data_size to decide which algorithm is the best(suitable) 
            //Fog_task<scc_vert_attr, scc_update, T> *sub_task = new Fog_task<scc_vert_attr, scc_update, T>();
            Fog_task<scc_color_vert_attr, scc_color_update, T> *sub_task = new Fog_task<scc_color_vert_attr, scc_color_update, T>();
            sub_task->set_task_id(b_config.bag_id);
            struct task_config * ptr_sub_task_config = new struct task_config;
            std::cout<<b_config.bag_id<<std::endl;
            std::cout<<b_config.data_size<<std::endl;
            std::cout<<b_config.data_name<<std::endl;
            std::string desc_data_name = sub_task->create_dataset(&b_config, eng_fb->get_vert_index());

            init_graph_desc(desc_data_name);
            
            ptr_sub_task_config->min_vert_id = pt.get<u32_t>("description.min_vertex_id");
            ptr_sub_task_config->max_vert_id = pt.get<u32_t>("description.max_vertex_id");
            ptr_sub_task_config->num_edges = pt.get<u32_t>("description.num_of_edges");
            ptr_sub_task_config->is_remap = true;
            ptr_sub_task_config->remap_file_name = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".remap";
            ptr_sub_task_config->vert_file_name  = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".index";
            ptr_sub_task_config->edge_file_name  = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".edge";
            ptr_sub_task_config->attr_file_name  = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".attr";
            //ptr_sub_task_config->with_in_edge = ;
            if(gen_config.with_in_edge)
            {
                ptr_sub_task_config->in_vert_file_name = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".in_index";
                ptr_sub_task_config->in_edge_file_name = desc_data_name.substr(0, desc_data_name.find_last_of(".")) + ".in_edge";
            }
            sub_task->set_task_config(ptr_sub_task_config);

            //remain to do : use the bag.data_size to decide which algorithm is the best(suitable) 
            //fb_queue_task.push(sub_task);
            color_queue_task.push(sub_task);

        }


        delete scc_ptr;
        delete main_task;
    }


    delete eng_fb;
    //Fog_program<scc_color_vert_attr, scc_color_update, T> *color_scc_ptr = new scc_color_program<T>(FORWARD_TRAVERSAL, true);
    fog_engine<scc_color_vert_attr, scc_color_update, T> * eng_color;
    eng_color = new fog_engine<scc_color_vert_attr, scc_color_update, T>(TARGET_ENGINE);

    while(!color_queue_task.empty())
    {
        struct Fog_task<scc_color_vert_attr, scc_color_update, T> * sub_task = color_queue_task.front();
        color_queue_task.pop();
        PRINT_DEBUG("*********************************** task %d****************************************\n", sub_task->get_task_id());

        Fog_program<scc_color_vert_attr, scc_color_update, T> *scc_ptr = new scc_color_program<T>(FORWARD_TRAVERSAL, true);

        sub_task->set_alg_ptr(scc_ptr);
        eng_color->run_task(sub_task);

        //delete scc_ptr;
        delete sub_task;
    }

    delete eng_color;
}

int main(int argc, const char**argv)
{
    time_t start_time;
    time_t end_time;
    Fog_adapter *adapter = new Fog_adapter();
    unsigned int type1_or_type2 = adapter->init(argc, argv);

    start_time = time(NULL);

    if(1 == type1_or_type2)
    {
        start_engine<type1_edge>();
    }   
    else
    {
        start_engine<type2_edge>();
    }   

    end_time = time(NULL);

    PRINT_DEBUG("The SCC program's run time = %.f seconds\n", difftime(end_time, start_time));

    return 0;

}
