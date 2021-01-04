#include <iostream>
#include <fstream>

#include "csubscriber.h"
#include <thread>

CSubscriber* g_pSubscriber;

int main(int argc, char** argv)
{

    ros::init(argc, argv, "lidar_camera_offline_calibration"); 
    g_pSubscriber = new CSubscriber;

    if(!g_pSubscriber->IsInitialized())
    {   
        return (EXIT_FAILURE);
    }

    std::thread ros_thread;
    ros_thread = std::thread( std::mem_fn(&CSubscriber::run), g_pSubscriber);
    ros_thread.join();
    return 0;
}
