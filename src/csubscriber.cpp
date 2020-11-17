#include "csubscriber.h"
#define CVUI_IMPLEMENTATION
#include "cvui.h"
CSubscriber::CSubscriber() :
        nh_private("~"),
        m_bInitialized ( false )
{
    //IsInitialized();
}


void CSubscriber::run()
{
    std::cout << "CSubscriber::run" << std::endl;
    int hl = 110;
    int sl = 25;
    int vl = 25;
    int hh = 140;
    int sh = 255;
    int vh = 255;
    int contours_num = 200;

    float zl = -1.0;
    float zh =  2.5;
    float xl = -0.3;
    float xh =  10.0;
    float yl =  -5.0;
    float yh =  5.0;
    float ball_r = 30.0;


    int hlo = 100;
    int slo = 25;
    int vlo = 25;
    int hho = 140;
    int sho = 255;
    int vho = 255;
    int contours_numo = 200;

    float zlo = -0.0;
    float zho =  2.5;
    float xlo = -0.3;
    float xho =  5.0;
    float ylo =  -5.0;
    float yho =  5.0;
    float ball_ro = 30.0;
    //cv::namedWindow("panel");
    cvui::init("panel");
    cv::Mat frame = cv::Mat(720, 710, CV_8UC3);

    while (ros::ok()) {
        //0==m_pLCM->handle()
        ros::spinOnce();
        frame = cv::Scalar(49, 52, 49);

        cvui::text(frame, 10, 20, "image config");
        cvui::text(frame, 10, 50, "hl");
        cvui::trackbar(frame, 10, 80, 300, &hl, 0, 180, 3);
        cvui::text(frame, 10, 140, "hh");
        cvui::trackbar(frame, 10, 170, 300, &hh, 0, 180, 3);
        cvui::text(frame, 10, 230, "sl");
        cvui::trackbar(frame, 10, 260, 300, &sl, 0, 255);
        cvui::text(frame, 10, 320, "sh");
        cvui::trackbar(frame, 10, 350, 300, &sh, 0, 255);
        cvui::text(frame, 10, 410, "vl");
        cvui::trackbar(frame, 10, 440, 300, &vl, 0, 255);
        cvui::text(frame, 10, 500, "vh");
        cvui::trackbar(frame, 10, 530, 300, &vh, 0, 255);
        cvui::text(frame, 10, 590, "contours_num");
        cvui::trackbar(frame, 10, 620, 300, &contours_num, 0, 1000, 5);

        cvui::text(frame, 400, 20, "pointcloud config");
        cvui::text(frame, 400, 50, "xl");
        cvui::trackbar(frame, 400, 80, 300, &xl, (float)-15.0, (float)15.0, 6);
        cvui::text(frame, 400, 140, "xh");
        cvui::trackbar(frame, 400, 170, 300, &xh, (float)-15.0, (float)15.0, 6);
        cvui::text(frame, 400, 230, "yl");
        cvui::trackbar(frame, 400, 260, 300, &yl, (float)-15.0, (float)15.0, 6);
        cvui::text(frame, 400, 320, "yh");
        cvui::trackbar(frame, 400, 350, 300, &yh, (float)-15.0, (float)15.0, 6);
        cvui::text(frame, 400, 410, "zl");
        cvui::trackbar(frame, 400, 440, 300, &zl, (float)-15.0, (float)15.0, 6);
        cvui::text(frame, 400, 500, "zh");
        cvui::trackbar(frame, 400, 530, 300, &zh, (float)-15.0, (float)15.0, 6);
        cvui::text(frame, 400, 590, "ball_r");
        cvui::trackbar(frame, 400, 620, 300, &ball_r, (float)0.0, (float)50.0, 5);

        if (cvui::button(frame, 10, 680, "Record")) {
            m_Handler.recordPair();
        }
        if (cvui::button(frame, 110, 680, "Clear")) {
            m_Handler.clearRecord();
        }
        if (cvui::button(frame, 210, 680, "Delete")) {
            m_Handler.deleteLast();
        }
        if (cvui::button(frame, 310, 680, "ShowInfo")) {
            m_Handler.showRecord();
        }
        if (cvui::button(frame, 410, 680, "Count")) {
            m_Handler.countTrans();
        }
        if (cvui::button(frame, 510, 680, "TestResult")) {
            m_Handler.testResult();
        }


        cvui::update();

        if(hl != hlo){
            setImageConfig(0, hl);
            hlo = hl;
        }
        if(hh != hho){
            setImageConfig(1, hh);
            hho = hh;
        }
        if(sl != slo){
            setImageConfig(2, sl);
            slo = sl;
        }
        if(sh != sho){
            setImageConfig(3, sh);
            sho = sh;
        }
        if(vl != vlo){
            setImageConfig(4, vl);
            vlo = vl;
        }
        if(vh != vho){
            setImageConfig(5, vh);
            vho = vh;
        }
        if(contours_num != contours_numo){
            setImageConfig(6, contours_num);
            contours_numo = contours_num;
        }
        if(zl != zlo){
            setPointCloudConfig(0, zl);
            zlo = zl;
        }
        if(zh != zho){
            setPointCloudConfig(1, zh);
            zho = zh;
        }
        if(xl != xlo){
            setPointCloudConfig(2, xl);
            xlo = xl;
        }
        if(xh != xho){
            setPointCloudConfig(3, xh);
            xho = xh;
        }
        if(yl != ylo){
            setPointCloudConfig(4, yl);
            ylo = yl;
        }
        if(yh != yho){
            setPointCloudConfig(5, yh);
            yho = yh;
        }
        if(ball_r != ball_ro){
            setPointCloudConfig(6, ball_r / 100.0);
            ball_ro = ball_r;
        }
        showLock();
        cv::imshow("panel", frame);

        if (cv::waitKey(30) == 27) {
            break;
        }
        showUnLock();
    }
}


bool CSubscriber::IsInitialized()
{
    std::string pointscloud_input, image_input, camera_info_input, fusison_output_topic,camera_matrix,tmp;
    nh_private.param<std::string>("pointcloud_input", pointscloud_input, "/rslidar_points");
    nh_private.param<std::string>("image_input", image_input, "/pylon/raw_image_2");
    nh_private.param<std::string>("camera_info_input", camera_info_input, "/camera_info");
    nh_private.param<std::string>("fusion_output_topic", fusison_output_topic, "/points_output");
    nh_private.param<std::string>("camera_matrix", camera_matrix, "475.0,0.0,240.0,0.0,475.0,151.0,0.0,0.0,1.0");
    nh_private.param<std::string>("save_path", save_path, "/home/feng/catkin_ws/src/lidar_camera_offline_calibration/cfg");

    std::stringstream input(camera_matrix);
    for (size_t i = 0; i < 3*3; i++)
    {
        getline(input, tmp, ',');
        // fprintf(stdout, "tmp:%s==%f\n",tmp.c_str(),atof(tmp.c_str()));
        K(i/3,i%3)=atof(tmp.c_str());
    }
    // ROS_INFO("K : %f %f %f %f %f %f %f %f %f", 
    //         K(0,0),K(0,1),K(0,2),
    //         K(1,0),K(1,1),K(1,2),
    //         K(2,0),K(2,1),K(2,2)
    //         );

    sub_cloud = nh.subscribe(pointscloud_input, 10, &CHandler::CloudCallback, &m_Handler);
    sub_image = nh.subscribe(image_input, 10, &CHandler::ImageCallback, &m_Handler);
    // sub_transform = nh.subscribe(camera_info_input, 1, &CHandler::IntrinsicsCallback, this);

    /*动态配置参数,编译错误*/
    // f = boost::bind(&CHandler::dynamic_callback,&m_Handler, _1, _2);
    // server.setCallback(f);
    /********************/

    m_bInitialized = true;
    std::cout << "ros IsInitialized" << std::endl;
}


void CSubscriber::setImageConfig(int i, int x){
    m_Handler.m_mtuImageConfig.lock();
    if(m_Handler.image_config.size() == 7){
        m_Handler.image_config[i] = x;
    }
    m_Handler.m_mtuImageConfig.unlock();
}
void CSubscriber::setPointCloudConfig(int i, float x){
    m_Handler.m_mtuPclConfig.lock();
    if(m_Handler.pcl_config.size() == 7){
        m_Handler.pcl_config[i] = x;
    }
    m_Handler.m_mtuPclConfig.unlock();
}
bool CSubscriber::showLock(){
    m_Handler.m_mtuShowConfig.lock();
    return true;
}
bool CSubscriber::showUnLock(){
    m_Handler.m_mtuShowConfig.unlock();
    return true;
}
