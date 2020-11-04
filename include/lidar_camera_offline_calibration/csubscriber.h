#ifndef CSUBSCRIBER_H
#define CSUBSCRIBER_H
#include <lcm/lcm-cpp.hpp>
#include "chandler.h"

class CSubscriber
{
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Subscriber sub_cloud;
    ros::Subscriber sub_image;
    ros::Subscriber sub_transform;

public:
    CSubscriber();
    void run();
    bool IsInitialized();
    void ShowImages();
    void setImageConfig(int i, int x);
    void setPointCloudConfig(int i, float x);
    bool showLock();
    bool showUnLock();

private:
    bool InitializeLCM();

public:

private:

    lcm::LCM* m_pLCM;
    bool m_bInitialized;
    CHandler m_Handler;
};

#endif // CSUBSCRIBER_H
