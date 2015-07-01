/**************************************************************************************************
 * Authors: 
 *   Zhiyuan Shao
 *
 * Routines:
 *   Disk threads.
 *************************************************************************************************/

#include <unistd.h>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/thread.hpp>
//#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include "print_debug.hpp"
#include "disk_thread.hpp"
#include "config.hpp"

#include <sys/stat.h>

extern general_config gen_config;

//impletation of io_work
io_work::io_work( const char *file_name_in, u32_t oper, char* buf, u64_t offset_in, u64_t size_in )
    :operation(oper), finished(0), buffer(buf), offset(offset_in), 
     size(size_in), someone_work_on_it( false ), io_file_name(file_name_in)
{}

void io_work::operator() (u32_t disk_thread_id)
{   
    //dump buffer to file
    switch( operation ){
        case FILE_READ:
        {
            //PRINT_DEBUG( "read to disk content to buffer :0x%llx, offset:%llu, size:%llu\n", 
             //   (u64_t)buffer, offset, size );
                    
            int read_finished=0, remain=size, res;

            //the file should exist now
            fd = open( io_file_name, O_RDWR, S_IRUSR | S_IRGRP | S_IROTH );
            if( fd < 0 ){
                PRINT_ERROR( "Cannot open attribute file for writing!\n");
                exit( -1 );
            }
            if( lseek( fd, offset, SEEK_SET ) < 0 ){
                PRINT_ERROR( "Cannot seek the attribute file!\n");
                exit( -1 );
            }
            while( read_finished < (int)size ){
                if( (res = read(fd, buffer, remain)) < 0 ) 
                    PRINT_ERROR( "failure on disk reading!\n" );
                read_finished += res;
                remain -= res;
                //PRINT_DEBUG("remain = %d\n", remain);
            }

            close(fd);
            //PRINT_DEBUG("Read work done!~~\n");

            break;
        }
        case FILE_WRITE:
        {
            //PRINT_DEBUG( "dump to disk tasks is received by disk thread, buffer:0x%llx, offset:%llu, size:%llu\n", 
            //	(u64_t)buffer, offset, size );
                
            int written=0, remain=size, res;

            //the file should exist now
            fd = open( io_file_name, O_RDWR , S_IRUSR | S_IRGRP | S_IROTH );
            if( fd < 0 ){
                PRINT_ERROR( "Cannot open attribute file: %s for writing!\n",io_file_name);
                exit( -1 );
            }
            if( lseek( fd, offset, SEEK_SET ) < 0 ){
                PRINT_ERROR( "Cannot seek the attribute file!\n");
                exit( -1 );
            }
            while( written < (int)size ){
                if( (res = write(fd, buffer, remain)) < 0 )
                    PRINT_ERROR( "failure on disk writing!\n" );
                written += res;
                remain -= res;
            }
            //add by hejian
            //fsync(fd);

            close(fd);
            //PRINT_DEBUG("Write work done!~~\n");
            break;
        }
    }
    //atomically increment finished 
    __sync_fetch_and_add(&finished, 1);
    __sync_synchronize();
}

//impletation of disk_thread
disk_thread::disk_thread(unsigned long disk_thread_id_in, class io_queue* work_queue_in)
    :disk_thread_id(disk_thread_id_in), work_queue(work_queue_in)
{
}
disk_thread::~disk_thread(){
}
void disk_thread::operator() ()
{
   do{
        work_queue->io_queue_sem.wait();

        if(work_queue->terminate_all) {
            //PRINT_DEBUG( "disk thread terminating\n" );
            printf( "disk thread terminating\n" );
            break;
        }

        //find work to do now.
        //Concurrency HAZARD: the io_work may be deleted during the searching!
        // e.g., the main thread delete some io work by invoking io_queue->del_io_task,
        // at the time of searching!
        //THEREFOR, do NOT allow any changes to the io task queue during searching.
        //HOWEVER, keep this critical section as small as possible!
        io_work * found=NULL;
        unsigned i;
        for( i=0; i<work_queue->io_work_queue.size(); i++ )
        {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(work_queue->io_queue_mutex);
            found = work_queue->io_work_queue.at(i);
            if( found->finished ) continue;
            if( found->someone_work_on_it ) continue;
            found->someone_work_on_it = true;
            break;
        }

        if( (found==NULL) || (i == work_queue->io_work_queue.size()) ) //nothing found! unlikely to happen
            printf( "IO Thread %lu found no work to do!\n", disk_thread_id );
            //PRINT_DEBUG( "IO Thread %lu found no work to do!\n", disk_thread_id );

        (*found)(disk_thread_id);

    }while(1);
};

//impletation of io_queue
io_queue::io_queue():io_queue_sem(0), terminate_all(false)
{
    //clear io work array
    io_work_queue.clear();
    std::vector<struct io_work*>().swap(io_work_queue);

    /*
    //the attribute file may not exist!
    attr_fd = open( gen_config.attr_file_name.c_str(), O_WRONLY|O_CREAT|O_TRUNC, S_IRUSR | S_IRGRP | S_IROTH );
    if( attr_fd < 0 ){
        PRINT_ERROR( "Cannot create attribute file for writing!\n");
        exit( -1 );
    }
    if( ftruncate( attr_fd, 0 ) < 0 ){
        PRINT_ERROR( "Cannot create attribute file for writing!\n");
        exit( -1 );
    }
        
    close( attr_fd );
    */

    //invoke the disk threads
    disk_threads = new disk_thread * [gen_config.num_io_threads];
    boost_disk_threads = new boost::thread *[gen_config.num_io_threads];
    for( u32_t i=0; i<gen_config.num_io_threads; i++ ){
        disk_threads[i] = new disk_thread( DISK_THREAD_ID_BEGIN_WITH + i, this );
        boost_disk_threads[i] = new boost::thread( boost::ref(*disk_threads[i]) );
    }
}

io_queue::~io_queue()
{
    //should wait the termination of all disk threads
    terminate_all = true;
    for( u32_t i=0; i<gen_config.num_io_threads; i++ )
        io_queue_sem.post();
    
    for( u32_t i=0; i<gen_config.num_io_threads; i++ )
        boost_disk_threads[i]->join();

    PRINT_DEBUG( "IO_QUEUE, terminated all disk threads\n" );
    //reclaim the resources occupied by the disk threads
}

void io_queue::add_io_task( io_work* new_task )
{
    assert( new_task->finished==0 );
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(io_queue_mutex);
    io_work_queue.push_back( new_task );

    //activate one disk thread to handle this task
    io_queue_sem.post();
}

void io_queue::del_io_task( io_work * task_to_del )
{
    assert( task_to_del->finished==1 );
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(io_queue_mutex);
    for( unsigned i=0; i<io_work_queue.size(); i++){
        if( io_work_queue.at(i) == task_to_del ){
//				PRINT_DEBUG( "in del_io_task, will delete the element at position %u.\n", i );
            io_work_queue.erase(io_work_queue.begin()+i);
        }
    }
    delete task_to_del;
}

//Note:
// After calling this member function, the thread will spin on waiting for the completion
// of specific io task!
// i.e., this is a BLOCKING operation!
void io_queue::wait_for_io_task( io_work * task_to_wait )
{
    while( 1 ){
        //should measure the time spent on waiting
        if( task_to_wait->finished ) break;
    };
}

