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

    void ImageCallback(const sensor_msgs::Image::ConstPtr& image_msg);
    void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    //void CallbackTransform(const lcm::ReceiveBuffer* recvBuf, const std::string& channelName, const TRANSFORM* msg);
    void initROS();
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
