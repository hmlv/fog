/*****************************************************************************************************
 * Authors:
 *    Huiming Lv
 *
 * Routines:
 *    base class of the programs
 *
 * Notes:
 *
 * 
 * ***************************************************************************************************/
#ifndef __FOG_ADAPTER_H__
#define __FOG_ADAPTER_H__


#include <stdlib.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/program_options.hpp>
#include "../fogsrc/fog_engine.cpp"
#include "../fogsrc/index_vert_array.cpp"
#include <queue>

extern struct general_config gen_config;
extern FILE * log_file;
extern FILE * test_log_file;
extern FILE * cv_log_file;
extern boost::program_options::options_description desc;
extern boost::program_options::variables_map vm; 
extern boost::property_tree::ptree pt;
extern int TASK_ID;
extern std::queue<struct bag_config> queue_bag_config;
extern std::queue<int> queue_task_id; 
//extern FILE * debug_out_edge_file;
//extern FILE * debug_in_edge_file;

class Fog_adapter{

    public:
        Fog_adapter();
        unsigned int init(int argc, const char** argv);

    private:
};

int flush_buffer_to_file( int fd, char* buffer, unsigned int size );

#endif
