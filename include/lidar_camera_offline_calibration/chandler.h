#ifndef CHANDLER_H
#define CHANDLER_H
#include <lcm/lcm-cpp.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <vector>
#include <mutex>
#include <GMSL_IMAGE_ENCODE.hpp>
#include <RSLIDAR_32_FRAME.hpp>
#include <TRANSFORM.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>

using namespace std;

class CHandler
{
public:
    CHandler();

    void CallbackRawImage_a_0(const lcm::ReceiveBuffer* recvBuf, const std::string& channelName, const GMSL_IMAGE_ENCODE* msg);
    void CallbackRawLidar_32(const lcm::ReceiveBuffer* recvBuf, const std::string& channelName, const RSLIDAR_32_FRAME* msg);
    void CallbackTransform(const lcm::ReceiveBuffer* recvBuf, const std::string& channelName, const TRANSFORM* msg);
    void ImageCallback(const sensor_msgs::Image::ConstPtr& image_msg);
    void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    //void CallbackTransform(const lcm::ReceiveBuffer* recvBuf, const std::string& channelName, const TRANSFORM* msg);

    void recordPair();
    void clearRecord();
    void deleteLast();
    void countTrans();
    int countRT();
    void showRecord();
    void testResult();
    void project2image(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pc, cv::Mat &raw_image, cv::Mat& output_image, Eigen::Matrix4f RT, Eigen::Matrix3f camera_param);
    pcl::visualization::PCLVisualizer* pointcloud_viewer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pc;
    cv::Mat m_image;
    cv::Point2f point_2d;
    cv::Point3f point_3d;
    std::vector<cv::Point2f> points_2d;
    std::vector<cv::Point3f> points_3d;
    std::vector<float> pcl_config;
    std::vector<int> image_config;

    mutex m_mtuImageConfig;
    mutex m_mtuPclConfig;
    mutex m_mtuShowConfig;

    bool hasResult;
private:

    bool m_bGetLidarClouds;
    bool m_bGetImage;

    mutex m_mtuRawImageFrame;
    mutex m_mtuRawLidarFrame;
    mutex m_mtuPoint2d;
    mutex m_mtuPoint3d;

    GMSL_IMAGE_ENCODE m_msgRawImageFrame_a_0;
    RSLIDAR_32_FRAME  m_msgRawLidarFrame_32;
    TRANSFORM m_msgTransform;
    std::mutex m_mtuTransform;
    bool m_bTransformInited;

    const bool ClockDiv(const int div );
    int64_t m_nRollingCounter;

    cv::Mat T, R;
private:


};

#endif // CHANDLER_H
