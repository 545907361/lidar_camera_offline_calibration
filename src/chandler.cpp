#include "chandler.h"
Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
std::string save_path = "";
CHandler::CHandler()
{
    m_bGetLidarClouds = false;
    m_bGetImage = false;
    hasResult = false;
    pointcloud_viewer = new pcl::visualization::PCLVisualizer("pointcloud_viewer");
    m_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pointcloud_viewer -> setBackgroundColor(0, 0, 0);
    //pcl::visualization::PCLVisualizer* my_viewer = new pcl::visualization::PCLVisualizer("my ");
    //my_viewer -> setBackgroundColor(0, 0, 200);
    m_mtuPclConfig.lock();
    pcl_config.resize(7);
    pcl_config[0] = -1.0;
    pcl_config[1] =  0.5;
    pcl_config[2] = -2.0;
    pcl_config[3] =  1.5;
    pcl_config[4] =  2.0;
    pcl_config[5] =  5.0;
    pcl_config[6] =  0.30;
    m_mtuPclConfig.unlock();

    m_mtuImageConfig.lock();
    image_config.resize(7);
    image_config[0] = 60;
    image_config[1] = 130;
    image_config[2] = 25;
    image_config[3] = 255;
    image_config[4] = 25;
    image_config[5] = 255;
    image_config[6] = 200;
    m_mtuImageConfig.unlock();

}
void dynamic_callback(lidar_camera_offline_calibration::CalibrationConfig &config, uint32_t level) 
{
  ROS_INFO("Reconfigure Request: %s %s %s", 
            config.pointcloud_input.c_str(), config.image_input.c_str(), 
            config.camera_matrix.c_str()
            );
    // std::string tmp;
    // Eigen::Matrix3f myK = Eigen::Matrix3f::Zero();
    // std::stringstream input(config.camera_matrix.c_str());
    // for (size_t i = 0; i < 3*3; i++)
    // {
    //     getline(input, tmp, ',');
    //     myK<<atof(tmp.c_str());
    // }
    // ROS_INFO("myK double: %f %f %f", 
    //         myK(0,0),myK(0,1),myK(0,2)
    //         );
}
void CHandler::ImageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    m_mtuRawImageFrame.lock();
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");
    cv::Mat image = cv_image->image;
    cv::Mat jpegimage;
    cv::cvtColor(image, jpegimage, cv::COLOR_BGR2RGB);
    jpegimage.copyTo(m_image);
    m_mtuRawImageFrame.unlock();
    m_bGetImage = true;
    fprintf(stdout, "do image process....\n");
    std::cout << jpegimage.rows << ", " << jpegimage.cols << std::endl;
    //cv::namedWindow("image", cv::WINDOW_NORMAL);
//    cv::createTrackbar("hl", "image", &hl, 180, NULL);
//    cv::createTrackbar("sl", "image", &sl, 255, NULL);
//    cv::createTrackbar("vl", "image", &vl, 255, NULL);
//    cv::createTrackbar("hh", "image", &hh, 180, NULL);
//    cv::createTrackbar("sh", "image", &sh, 255, NULL);
//    cv::createTrackbar("vh", "image", &vh, 255, NULL);
//    cv::createTrackbar("contours_num", "image", &contours_num, 1000, NULL);
//    cv::createButton("Blur", blurCallback, NULL, cv::QT_CHECKBOX, 0);
//    cv::createTrackbar("xia", "image", &xia, 200, NULL);
//    cv::createTrackbar("shang", "image", &shang, 200, NULL);
//    cv::createTrackbar("hou", "image", &hou, 200, NULL);
//    cv::createTrackbar("qian", "image", &qian, 200, NULL);
//    cv::createTrackbar("zuo", "image", &zuo, 200, NULL);
//    cv::createTrackbar("you", "image", &you, 200, NULL);
//    cv::createTrackbar("r", "image", &r, 100, NULL);

    m_mtuImageConfig.lock();
    int hl = image_config[0];
    int hh = image_config[1];
    int sl = image_config[2];
    int sh = image_config[3];
    int vl = image_config[4];
    int vh = image_config[5];
    int contours_num = image_config[6];
    m_mtuImageConfig.unlock();
    cv::Mat hsv_image;
    cv::cvtColor(jpegimage, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat mask;

    cv::inRange(hsv_image, cv::Scalar(hl, sl, vl),
                cv::Scalar(hh, sh, vh), mask);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(mask, mask, element);//腐蚀
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::dilate(mask, mask, element);//膨胀
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(mask, mask, element);
    cv::Mat mask_color;
    cv::cvtColor(mask, mask_color, cv::COLOR_GRAY2BGR);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    cv::Point2f ellips_center;
    int ellipse_num = 0;
    for(size_t i = 0; i < contours.size(); i++)
    {
        size_t count = contours[i].size();
        if( count < contours_num )
            continue;
        cv::drawContours(mask_color, contours, i, cv::Scalar(0, 255, 0), 1);

        cv::RotatedRect box = cv::fitEllipseDirect(contours[i]);
        if( !(box.size.width>=0) || !(box.size.height>=0) )continue;
        if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*1.5 )
            continue;



        ellips_center.x = box.center.x;
        ellips_center.y = box.center.y;

        cv::ellipse(mask_color, box, cv::Scalar(0,0,255), 1, CV_AA);
        cv::ellipse(jpegimage, box, cv::Scalar(0,0,255), 1, CV_AA);
        ellipse_num++;
    }
    
    if(ellipse_num != 1){
        cv::Mat dst;
        dst.create(jpegimage.rows, jpegimage.cols * 2, CV_8UC3);
        jpegimage.copyTo(dst(cv::Rect(0, 0, jpegimage.cols, jpegimage.rows)));
        mask_color.copyTo(dst(cv::Rect(mask_color.cols, 0, mask_color.cols, mask_color.rows)));
        m_mtuShowConfig.lock();
        cv::imshow("image", dst);
        cv::waitKey(10);
        m_mtuShowConfig.unlock();
        m_mtuRawImageFrame.unlock();
        std::cout << "\033[33mWarning: have " << ellipse_num << " (!=1) ellipses\033[0m" << std::endl;
        return;
    }
    m_mtuPoint2d.lock();
    point_2d.x = ellips_center.x;
    point_2d.y = ellips_center.y;
    m_mtuPoint2d.unlock();
    cv::circle(jpegimage, ellips_center, 2, cv::Scalar(255, 0, 0), -1);
    cv::circle(mask_color, ellips_center, 2, cv::Scalar(255, 0, 0), -1);
    std::cout << "ellipse_center (x, y): " << point_2d.x << ", " << point_2d.y << std::endl;
    cv::Mat dst;
    dst.create(jpegimage.rows, jpegimage.cols * 2, CV_8UC3);
    jpegimage.copyTo(dst(cv::Rect(0, 0, jpegimage.cols, jpegimage.rows)));
    mask_color.copyTo(dst(cv::Rect(mask_color.cols, 0, mask_color.cols, mask_color.rows)));
    m_mtuShowConfig.lock();
    cv::imshow("image", dst);
    cv::waitKey(10);
    m_mtuShowConfig.unlock();

}
void CHandler::CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    m_mtuRawLidarFrame.lock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *raw_pc);
    m_bGetLidarClouds = true;
    fprintf(stdout, "do lidar process....\n");
    //std::cout << "get " << points_num << "points" << std::endl;
    pcl::copyPointCloud(*raw_pc, *m_pc);
    m_mtuRawLidarFrame.unlock();
    m_mtuPclConfig.lock();
    float zl = pcl_config[0];
    float zh = pcl_config[1];
    float xl = pcl_config[2];
    float xh = pcl_config[3];
    float yl = pcl_config[4];
    float yh = pcl_config[5];
    float ball_r = pcl_config[6];
    m_mtuPclConfig.unlock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_other_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_SPHERE(new pcl::ModelCoefficients);
    std::vector<int> indices;
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_SPHERE(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>()), cloud_SPHERE(new pcl::PointCloud<pcl::PointXYZ>());

/******************************************************************************/
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(raw_pc);
    pass.setFilterFieldName("z");
    pass.setNegative(false);
    pass.setFilterLimits(MIN(zl, zh), MAX(zl, zh));
    pass.filter(*cloud_filtered);
    pass.setNegative(true);
    pass.filter(*cloud_filtered2);
    *pass_other_points += *cloud_filtered2;

    pass.setNegative(false);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(MIN(xl, xh), MAX(xl, xh));
    pass.filter(*raw_pc);
    pass.setNegative(true);
    pass.filter(*cloud_filtered2);
    *pass_other_points += *cloud_filtered2;

    pass.setNegative(false);
    pass.setInputCloud(raw_pc);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(MIN(yl, yh), MAX(yl, yh));
    pass.filter(*cloud_filtered);
    pass.setNegative(true);
    pass.filter(*cloud_filtered2);
    *pass_other_points += *cloud_filtered2;
    //std::cout<<"Points num: "<<raw->points.size()<<" change to "<<cloud_filtered->points.size()<<std::endl;

/******************************************************************************/

//    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
//    sor.setInputCloud(cloud_filtered);
//    sor.setMeanK(OutlierRemoval_k);
//    sor.setStddevMulThresh(OutlierRemoval_StddevMulThresh);
//    sor.filter(*cloud_filtered);
//
//    sensor_msgs::PointCloud2::Ptr filter_msg_ptr2(new sensor_msgs::PointCloud2);
//    pcl::toROSMsg(*cloud_filtered, *filter_msg_ptr2);
//    filter_msg_ptr2 -> header = input -> header;
//    FilteredPublisher2.publish(*filter_msg_ptr2);

/******************************************************************************/
/*
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers_plane, *coefficients_plane);
//    std::cout<<"Plane coefficients: "<<*coefficients_plane<<std::endl;


    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    extract.setNegative(true);
    extract.filter(*cloud_filtered2);
    //std::cout<<"Plane has "<<cloud_plane->points.size()<<" points"<<std::endl;
*/
    pcl::PointXYZ center_p;
    if(cloud_filtered -> size() > 10){
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setInputCloud(cloud_filtered);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(3000);
    seg.setDistanceThreshold(0.03);
    seg.setRadiusLimits(ball_r-0.05, ball_r+0.05);
    seg.segment(*inliers_SPHERE, *coefficients_SPHERE);
    //std::cout << "Spheer coefficients: " << *coefficients_SPHERE;
    std::cout << "(x, y, z, r): " << "(" << coefficients_SPHERE->values[0] << ", " << coefficients_SPHERE->values[1] << ", " << coefficients_SPHERE->values[2] << ", " << coefficients_SPHERE->values[3] << std::endl;
    m_mtuPoint3d.lock();
    point_3d.x = coefficients_SPHERE->values[0];
    point_3d.y = coefficients_SPHERE->values[1];
    point_3d.z = coefficients_SPHERE->values[2];
    m_mtuPoint3d.unlock();
    center_p.x = point_3d.x;
    center_p.y = point_3d.y;
    center_p.z = point_3d.z;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_SPHERE);
    extract.setNegative(false);
    extract.filter(*cloud_SPHERE);
    extract.setNegative(true);
    extract.filter(*cloud_filtered3);
    }
    else{
    std::cout << "pos error" << std::endl;
    }
    //std::cout<<"Spheer has "<<cloud_cylinder->points.size()<<" points"<<std::endl;

    //std::cout<<"lidar ok"<<std::endl<<std::endl;

    if(pointcloud_viewer -> removeAllPointClouds()){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_pass_other_points(pass_other_points, 255, 255, 255);
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud_plane(cloud_plane, 0, 0, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud_SPHERE(cloud_SPHERE, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud_filtered3(cloud_filtered3, 255, 0, 0);
        pointcloud_viewer -> addPointCloud<pcl::PointXYZ>(pass_other_points, color_pass_other_points, "pass_other_points");
        //pointcloud_viewer -> addPointCloud<pcl::PointXYZ>(cloud_plane, color_cloud_plane, "plane_pointcloud");
        pointcloud_viewer -> addPointCloud<pcl::PointXYZ>(cloud_SPHERE, color_cloud_SPHERE, "SPHERE_pointcloud");
        pointcloud_viewer -> addPointCloud<pcl::PointXYZ>(cloud_filtered3, color_cloud_filtered3, "other_pointcloud");
	//if(cloud_filtered -> size() > 10){
	//	pointcloud_viewer -> addSphere(center_p, pcl_config[6], "Sphere", 0);
	//}

    }
    pointcloud_viewer -> spinOnce(100);
}

const bool CHandler::ClockDiv(const int div )
{
    return !( (m_nRollingCounter++)%div );
}

void CHandler::project2image(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pc, cv::Mat &raw_image, cv::Mat& output_image, Eigen::Matrix4f RT, Eigen::Matrix3f camera_param){

    Eigen::Matrix<float, 3, 4> TT_local, TTT_local;//lida2image=T*(T2)*T3
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    TT_local = RT.topRows(3);
    TTT_local = camera_param * TT_local;
    m_mtuRawImageFrame.lock();
    m_mtuRawLidarFrame.lock();
    raw_image.copyTo(output_image);
    pcl::copyPointCloud(*input_pc, *pc);
    m_mtuRawImageFrame.unlock();
    m_mtuRawLidarFrame.unlock();
    pcl::PointXYZ r;
    Eigen::Vector4f raw_point;
    Eigen::Vector3f trans_point;
    double deep, deep_config;//deep_config: normalize, max deep
    int point_r;
    deep_config = 80;
    point_r = 1;
    //std::cout << "image size; " << raw_image.cols << " * " << raw_image.rows << std::endl;
    for(int i=0; i<pc->size(); i++){
        r = pc->points[i];
        raw_point(0, 0) = r.x;
        raw_point(1, 0) = r.y;
        raw_point(2, 0) = r.z;
        raw_point(3, 0) = 1;
        trans_point = TTT_local * raw_point;
        int x = (int)(trans_point(0, 0) / trans_point(2, 0));
        int y = (int)(trans_point(1, 0) / trans_point(2, 0));
        if(x<0 || x>(raw_image.cols-1) || y<0 || y>(raw_image.rows-1))continue;
        deep = trans_point(2, 0) / deep_config;
        //deep = r.intensity / deep_config;
        int blue, red, green;
        if(deep <= 0.5){
            green = (int)((0.5-deep)/0.5*255);
            red   = (int)(deep/0.5*255);
            blue  = 0;
        }
        else if(deep <= 1){
            green = 0;
            red   = (int)((1-deep)/0.5*255);
            blue  = (int)((deep-0.5)/0.5*255);
        }
        else{
            blue  = 0;
            green = 0;
            red   = 255;
        };

        cv::circle(output_image, cv::Point2f(x, y), point_r, cv::Scalar(blue,green,red), -1);
        //cv::circle(edge_distance_image2, cv::Point2f(x, y), point_r, cv::Scalar(red,green,blue), -1);
    }
}

int CHandler::countRT(){

    // cv::Matx33d camera_p(
    //         475.0, 0, 240,
    //         0, 475.0, 151,
    //         0, 0, 1);
    cv::Matx33d camera_p;
    for (size_t i = 0; i < 3*3; i++)
        camera_p(i/3,i%3)=K(i/3,i%3);
    std::vector<cv::Point3d> points_3d4;
    std::vector<cv::Point2d> points_2d4;
    int point_number = points_2d.size();
    if (point_number != points_3d.size()) {
        std::cout << "\033[33mWarning: 3D num != 2D num\033[0m" << std::endl;
        return -1;
    }
    int num[4] = {point_number, point_number, point_number, point_number};
    int n = 10;

    unsigned seed = std::time(0);
    std::srand(seed);
    cv::Mat Rod_r, TransMatrix, RotationR, pos;
    cv::Mat trans = cv::Mat(3, 1, CV_64F);
    cv::Mat rr = cv::Mat(3, 1, CV_64F);
    trans.at<double>(0) = 0;
    trans.at<double>(1) = 0;
    trans.at<double>(2) = 0;
    rr.at<double>(0) = 0;
    rr.at<double>(1) = 0;
    rr.at<double>(2) = 0;
    std::cout << "start compute" << std::endl;
    for (int i = 0; i < n; i++) {
        for (int ii = 0; ii < 4; ii++) {
            num[ii] = std::rand() % point_number;
            for (int iii = 0; iii < ii; iii++) {
                if (num[ii] == num[iii]) {
                    num[ii] = std::rand() % point_number;
                    iii = -1;
                }
            }
            points_3d4.push_back(points_3d[num[ii]]);
            points_2d4.push_back(points_2d[num[ii]]);
        }

        bool success = cv::solvePnP(points_3d4, points_2d4, camera_p, cv::noArray(), Rod_r, TransMatrix, false,
                                    cv::SOLVEPNP_P3P);
//        std::cout << "r:" << std::endl << Rod_r << std::endl;
//        std::cout << "R:" << std::endl << RotationR << std::endl;
//        std::cout << "T:" << std::endl << TransMatrix << std::endl;
        points_3d4.clear();
        points_2d4.clear();
        for (int ii = 0; ii < 4; ii++) {
            num[ii] = point_number;
        }
        rr.at<double>(0) += Rod_r.at<double>(0);
        rr.at<double>(1) += Rod_r.at<double>(1);
        rr.at<double>(2) += Rod_r.at<double>(2);
        trans.at<double>(0) += TransMatrix.at<double>(0);
        trans.at<double>(1) += TransMatrix.at<double>(1);
        trans.at<double>(2) += TransMatrix.at<double>(2);
    }

    rr.at<double>(0) = rr.at<double>(0) / 10.0;
    rr.at<double>(1) = rr.at<double>(1) / 10.0;
    rr.at<double>(2) = rr.at<double>(2) / 10.0;
    trans.at<double>(0) = trans.at<double>(0) / 10.0;
    trans.at<double>(1) = trans.at<double>(1) / 10.0;
    trans.at<double>(2) = trans.at<double>(2) / 10.0;
    cv::Rodrigues(rr, RotationR);
    RotationR.copyTo(R);
    trans.copyTo(T);
    std::cout << "R:" << R << std::endl;
    std::cout << "T:" << T << std::endl;
    ofstream outfile;
    outfile.open(save_path + "/test_result.txt");
    outfile << R << endl;
    outfile << T << endl;
    outfile.close();
    return 1;
}

void CHandler::recordPair(){
    m_mtuPoint2d.lock();
    points_2d.push_back(point_2d);
    m_mtuPoint2d.unlock();
    m_mtuPoint3d.lock();
    points_3d.push_back(point_3d);
    m_mtuPoint3d.unlock();
    std::cout << "\033[32m******************************\033[0m"<<std::endl;
    std::cout << "\033[32mrecord one pair\033[0m" << std::endl;
    std::cout << "\033[32mnow has pair num: " << points_3d.size() << "\033[0m" << std::endl;
    std::cout << "\033[32m******************************\033[0m"<<std::endl;
}

void CHandler::clearRecord(){
    points_2d.clear();
    points_3d.clear();
    std::cout << "\033[32m******************************\033[0m"<<std::endl;
    std::cout << "\033[32mclear finish\033[0m" << std::endl;
    std::cout << "\033[32m******************************\033[0m"<<std::endl;
}
void CHandler::deleteLast(){
    if(!(points_3d.size() > 0)){
        std::cout << "\033[32m******************************\033[0m"<<std::endl;
        std::cout << "\033[32mnow has no pair\033[0m" << std::endl;
        std::cout << "\033[32m******************************\033[0m"<<std::endl;
        return;
    }
    else{
        points_3d.pop_back();
        points_2d.pop_back();
        std::cout << "\033[32m******************************\033[0m"<<std::endl;
        std::cout << "\033[32mdelete one pair\033[0m" << std::endl;
        std::cout << "\033[32mnow has pair num: " << points_3d.size() << "\033[0m" << std::endl;
        std::cout << "\033[32m******************************\033[0m"<<std::endl;
        return;
    }
}
void CHandler::countTrans(){
    if(points_2d.size() < 4){
        std::cout << "\033[33mWarning: pair num < 4\033[0m" << std::endl;
        return;
    }
    if(countRT() != 1){
        std::cout << "\033[33mCount Error\033[0m" << std::endl;
        return;
    }
    hasResult = true;
    return;
}

void CHandler::showRecord(){
    std::cout << "\033[32mnow has pair num: " << points_3d.size() << "\033[0m" << std::endl;
    for(int i=0; i<points_3d.size(); i++){
        std::cout << "\033[32m(" << points_3d[i].x << ", " << points_3d[i].y << ", " << points_3d[i].z << ") - (" << points_2d[i].x << ", " << points_2d[i].y << ")\033[0m" << std::endl;
    }
    return;
}

void CHandler::testResult(){
    if(!hasResult){
        std::cout << "\033[33mhas no result\033[0m" << std::endl;
        return;
    }
    cv::Mat result_image;
    Eigen::Matrix4f ET = Eigen::Matrix4f::Identity();
    ET(0,0) = (float)R.at<double>(0, 0);
    ET(0,1) = (float)R.at<double>(0, 1);
    ET(0,2) = (float)R.at<double>(0, 2);
    ET(1,0) = (float)R.at<double>(1, 0);
    ET(1,1) = (float)R.at<double>(1, 1);
    ET(1,2) = (float)R.at<double>(1, 2);
    ET(2,0) = (float)R.at<double>(2, 0);
    ET(2,1) = (float)R.at<double>(2, 1);
    ET(2,2) = (float)R.at<double>(2, 2);
    ET(0,3) = (float)T.at<double>(0);
    ET(1,3) = (float)T.at<double>(1);
    ET(2,3) = (float)T.at<double>(2);
    ET(3,0) = 0.0;
    ET(3,1) = 0.0;
    ET(3,2) = 0.0;
    ET(3,3) = 1.0;
    // Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
    // K << 475.0, 0.0, 240.0, 0.0, 475.0, 151.0, 0.0, 0.0, 1.0;
    project2image(m_pc, m_image, result_image, ET, K);
    std::string image_name = "/test_result.png";
    // std::string save_path = ros::package::getPath("lidar_camera_offline_calibration");
    cv::imwrite(save_path+image_name, result_image);
    std::cout << "\033[32mtest result has saved to " << save_path << "\033[0m" << std::endl;
    return;
}
