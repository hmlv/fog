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
#include <vector>
#include <algorithm>
#include <functional>

#define REMAP_BUFFER_LEN 2048*2048
struct out_degree_node
{
    u32_t out_degree;
    u32_t vert_id;
    //ascend
    bool operator < (const out_degree_node& rhs) const
    {
        return out_degree < rhs.out_degree;
    }
};

struct in_degree_node
{
    u32_t in_degree;
    u32_t vert_id;
    //descend
    bool operator > (const in_degree_node& rhs) const
    {
        return in_degree > rhs.in_degree;
    }
};

template<typename T>
void run_out_degree_ascend()
{
    index_vert_array<T> * vert_index = new index_vert_array<T>; 
    struct bag_config output_bag;
    //std::stringstream output_id_stream;
    //output_id_stream<<(TASK_ID+1);
    //std::string result_file_name = gen_config.graph_path + result_id_stream.str(); + ".result";
    std::string remap_file_name = gen_config.graph_path + "/temp0" + ".remap";

    int remap_fd = open( remap_file_name.c_str(), O_CREAT|O_WRONLY, S_IRUSR);
    if(-1 == remap_fd)
    {
        printf("Cannot create remap file:%s\n Aborted.\n", remap_file_name.c_str());
        exit(-1);
    }

    std::vector<struct out_degree_node> out_vect; 
    for (unsigned int id = gen_config.min_vert_id; id <= gen_config.max_vert_id; id++)
    {
        struct out_degree_node temp_node;
        temp_node.vert_id = id;
        temp_node.out_degree = vert_index->num_edges(id, OUT_EDGE);
        out_vect.push_back(temp_node);
    }
    stable_sort(out_vect.begin(), out_vect.end(), std::less<out_degree_node>());
    int remap_buf_suffix = 0;
    int remap_buf_offset = 0;

    int vert_num = 0;
    //int vert_to_TRIM = 0;
    u32_t * remap_buffer = new u32_t[REMAP_BUFFER_LEN];
    memset( (char*)remap_buffer, 0, REMAP_BUFFER_LEN*sizeof(u32_t) );

    size_t vec_size = out_vect.size();
    for (size_t i = 0; i < vec_size; i++)
    {
        PRINT_DEBUG_CV_LOG("vid %d, out_degree %d\n", out_vect[i].vert_id, out_vect[i].out_degree);
        remap_buf_suffix = vert_num - remap_buf_offset*REMAP_BUFFER_LEN;
        remap_buffer[remap_buf_suffix] = out_vect[i].vert_id;
        if(remap_buf_suffix == REMAP_BUFFER_LEN-1)
        {
            flush_buffer_to_file(remap_fd, (char*)remap_buffer, REMAP_BUFFER_LEN*sizeof(u32_t));
            memset( (char*)remap_buffer, 0, REMAP_BUFFER_LEN*sizeof(u32_t) );
            remap_buf_offset++;
        }
        vert_num++;
    }
    
    flush_buffer_to_file(remap_fd, (char*)remap_buffer, (vert_num - remap_buf_offset*REMAP_BUFFER_LEN)*sizeof(u32_t));
    
    PRINT_DEBUG("all = %d\n", vert_num);

    delete [] remap_buffer;

    output_bag.creater_id = 0; 
    output_bag.data_name = remap_file_name;
    //TASK_ID++;
    //queue_task_id.push(TASK_ID);
    output_bag.bag_id = 0;
    output_bag.data_size = vert_num;

    //queue_bag_config.push(output_bag);
    //std::cout<<"queue size: "<<queue_bag_config.size()<<std::endl;
    //std::cout<<"trim filter over!"<<std::endl;
    //
    Fog_task<out_degree_node, out_degree_node, T> *sub_task = new Fog_task<out_degree_node, out_degree_node, T>();
    sub_task->set_task_id(output_bag.bag_id);
    std::string desc_data_name = sub_task->create_dataset_sequence(&output_bag, vert_index);
    std::cout<<desc_data_name<<std::endl;
    
}

template<typename T>
void run_in_degree_descend()
{
    index_vert_array<T> * vert_index = new index_vert_array<T>; 
    struct bag_config output_bag;
    //std::stringstream output_id_stream;
    //output_id_stream<<(TASK_ID+1);
    //std::string result_file_name = gen_config.graph_path + result_id_stream.str(); + ".result";
    std::string remap_file_name = gen_config.graph_path + "/temp0" + ".remap";
    //std::string remap_file_name = gen_config.edge_file_name.substr(0, gen_config.edge_file_name.find_last_of(".")) + "in_deg_descend.remap"

    int remap_fd = open( remap_file_name.c_str(), O_CREAT|O_WRONLY, S_IRUSR);
    if(-1 == remap_fd)
    {
        printf("Cannot create remap file:%s\n Aborted.\n", remap_file_name.c_str());
        exit(-1);
    }

    std::vector<struct in_degree_node> output_vect; 
    for (unsigned int id = gen_config.min_vert_id; id <= gen_config.max_vert_id; id++)
    {
        if(id % 10000000 == 0)
        {
            std::cout<<id<<std::endl;
        }
        struct in_degree_node temp_node;
        temp_node.vert_id = id;
        temp_node.in_degree = vert_index->num_edges(id, IN_EDGE);
        output_vect.push_back(temp_node);
    }
    std::cout<<"output_vect ok\n";
    stable_sort(output_vect.begin(), output_vect.end(), std::greater<in_degree_node>());
    int remap_buf_suffix = 0;
    int remap_buf_offset = 0;

    int vert_num = 0;
    //int vert_to_TRIM = 0;
    u32_t * remap_buffer = new u32_t[REMAP_BUFFER_LEN];
    memset( (char*)remap_buffer, 0, REMAP_BUFFER_LEN*sizeof(u32_t) );

    size_t vec_size = output_vect.size();
    for (size_t i = 0; i < vec_size; i++)
    {
        PRINT_DEBUG_CV_LOG("vid %d, in_degree %d\n", output_vect[i].vert_id, output_vect[i].in_degree);
        remap_buf_suffix = vert_num - remap_buf_offset*REMAP_BUFFER_LEN;
        remap_buffer[remap_buf_suffix] = output_vect[i].vert_id;
        if(remap_buf_suffix == REMAP_BUFFER_LEN-1)
        {
            flush_buffer_to_file(remap_fd, (char*)remap_buffer, REMAP_BUFFER_LEN*sizeof(u32_t));
            memset( (char*)remap_buffer, 0, REMAP_BUFFER_LEN*sizeof(u32_t) );
            remap_buf_offset++;
        }
        vert_num++;
    }
    
    flush_buffer_to_file(remap_fd, (char*)remap_buffer, (vert_num - remap_buf_offset*REMAP_BUFFER_LEN)*sizeof(u32_t));
    
    PRINT_DEBUG("all = %d\n", vert_num);

    delete [] remap_buffer;

    output_bag.creater_id = 0; 
    output_bag.data_name = remap_file_name;
    //TASK_ID++;
    //queue_task_id.push(TASK_ID);
    output_bag.bag_id = 0;
    output_bag.data_size = vert_num;

    //queue_bag_config.push(output_bag);
    //std::cout<<"queue size: "<<queue_bag_config.size()<<std::endl;
    //std::cout<<"trim filter over!"<<std::endl;
    //
    Fog_task<out_degree_node, out_degree_node, T> *sub_task = new Fog_task<out_degree_node, out_degree_node, T>();
    sub_task->set_task_id(output_bag.bag_id);
    std::string desc_data_name = sub_task->create_dataset_sequence(&output_bag, vert_index);
    std::cout<<desc_data_name<<std::endl;
    
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
        run_in_degree_descend<type1_edge>();
    }   
    else
    {
        run_in_degree_descend<type2_edge>();
    }   

    end_time = time(NULL);

    PRINT_DEBUG("The remap program's run time = %.f seconds\n", difftime(end_time, start_time));

    return 0;

}
