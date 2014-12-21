/**************************************************************************************************
 * Authors: 
 *   Zhiyuan Shao, Jian He, Huiming Lv 
 *
 * Declaration:
 *   Program parameter parsing.
 *
 * Notes:
 *   1.modify bfs::bfs_root to bfs::bfs-root by Huiming Lv   2014/12/21 
 *************************************************************************************************/

#ifndef _OPTIONS_UTILS_
#define _OPTIONS_UTILS_

#include <boost/program_options.hpp>
#include <iostream>

boost::program_options::options_description desc;
boost::program_options::variables_map vm;

static void setup_options_convert(int argc, const char* argv[])
{
  desc.add_options()
	( "help,h", "Display help message")
	( "type,t", boost::program_options::value<std::string>()->required(), "Type of the snap file(edgelist or adjlist)")
	( "graph,g", boost::program_options::value<std::string>()->required(), "Name of the graph in snap format")
	( "destination,d",  boost::program_options::value<std::string>()->required(), "Destination folder that will contain the index and edge file")
    ("out-type,o", boost::program_options::value<std::string>()->required(), "Type of out edge file, type1(with edge value) or type2(no edge value)")
    ("in-edge,i", boost::program_options::value<bool>()->default_value(false),"With or without in-edge, backtrace-algorithm(WCC, SCC) must be true!");
  try {
    boost::program_options::store(boost::program_options::parse_command_line(argc,
									     argv,
									     desc),
				  vm);
    boost::program_options::notify(vm);
  }
  catch (boost::program_options::error &e) {
    if(vm.count("help") || argc ==1) {
      std::cerr << desc << "\n";
    }
    std::cerr << "Error:" << e.what() << std::endl;
    std::cerr << "Try: " << argv[0] << " --help" << std::endl;
    exit(-1);
  }
}

static void setup_options_fog(int argc, const char* argv[])
{
  desc.add_options()
	( "help,h", "Display help message")
	( "graph,g", boost::program_options::value<std::string>()->required(), 
		"The name of the graph to be processed (i.e., the .desc file).")
	( "application,a", boost::program_options::value<std::string>()->required(), 
		"The name of the application, e.g., pagerank, sssp, bfs.")
    ("in-edge,i", boost::program_options::value<bool>()->default_value(false),"With or without in-edge, backtrace-algorithm(WCC, SCC) must be true!")
	//following are system parameters
    ( "memory,m", boost::program_options::value<unsigned long>()->default_value(1024), //default 1GB
     "Size of the buffer for writing (unit is MB)")
    ( "processors,p",  boost::program_options::value<unsigned long>()->default_value(4),
      "Number of processors")
    ( "diskthreads,d",  boost::program_options::value<unsigned long>()->default_value(2),
      "Number of Disk(I/O) threads")
	//following are the parameters for appilcations
	// pagerank
    ("pagerank::niters", boost::program_options::value<unsigned long>()->default_value(10),
     "number of iterations for pagerank.")
	// sssp
    ("sssp::source", boost::program_options::value<unsigned long>()->default_value(0),
     "source vertex id for sssp")
    ("bfs::bfs-root", boost::program_options::value<unsigned long>()->default_value(0),
     "bfs root for bfs");
	// belief propagation
    //("spmv::niters", boost::program_options::value<unsigned long>()->default_value(10),
    // "number of iterations for spmv");

  try {
    boost::program_options::store(boost::program_options::parse_command_line(argc,
									     argv,
									     desc),
									  	 vm);
    boost::program_options::notify(vm);
  }
  catch (boost::program_options::error &e) {
    if(vm.count("help") || argc ==1) {
      std::cerr << desc << "\n";
    }
    std::cerr << "Error:" << e.what() << std::endl;
    std::cerr << "Try: " << argv[0] << " --help" << std::endl;
    exit(-1);
  }
}

#endif
