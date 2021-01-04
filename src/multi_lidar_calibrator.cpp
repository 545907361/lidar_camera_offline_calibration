#include "multi_lidar_calibrator.h"

void ROSMultiLidarCalibratorApp::PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header.frame_id = parent_frame_;
	in_publisher.publish(cloud_msg);
}


void ROSMultiLidarCalibratorApp::MatrixToTranform(Eigen::Matrix4f & matrix, tf::Transform & trans){
    tf::Vector3 origin;
    origin.setValue(static_cast<double>(matrix(0,3)),static_cast<double>(matrix(1,3)),static_cast<double>(matrix(2,3)));

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(matrix(0,0)), static_cast<double>(matrix(0,1)), static_cast<double>(matrix(0,2)),
    static_cast<double>(matrix(1,0)), static_cast<double>(matrix(1,1)), static_cast<double>(matrix(1,2)),
    static_cast<double>(matrix(2,0)), static_cast<double>(matrix(2,1)), static_cast<double>(matrix(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    trans.setOrigin(origin);
    trans.setRotation(tfqt);
}

void ROSMultiLidarCalibratorApp::PerformNdtOptimize(){

    if (in_parent_cloud_== nullptr || in_child_cloud_== nullptr){
        return;
    }

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;

    ndt.setTransformationEpsilon(ndt_epsilon_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setResolution(ndt_resolution_);

    ndt.setMaximumIterations(ndt_iterations_);

    ndt.setInputSource(in_child_filtered_cloud_);
    ndt.setInputTarget(in_parent_cloud_);

    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);



    if(current_guess_ == Eigen::Matrix4f::Identity())
    {
        Eigen::Translation3f init_translation(transfer_map_[points_child_topic_str][0],
                transfer_map_[points_child_topic_str][1], transfer_map_[points_child_topic_str][2]);
        Eigen::AngleAxisf init_rotation_x(transfer_map_[points_child_topic_str][5], Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(transfer_map_[points_child_topic_str][4], Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(transfer_map_[points_child_topic_str][3], Eigen::Vector3f::UnitZ());


        Eigen::Matrix4f init_guess_ = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

        current_guess_ = init_guess_;
    }

    ndt.align(*output_cloud, current_guess_);

    std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged ()
              << " score: " << ndt.getFitnessScore () << " prob:" << ndt.getTransformationProbability() << std::endl;
    std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*in_child_cloud_, *output_cloud, ndt.getFinalTransformation());

    current_guess_ = ndt.getFinalTransformation();

    Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
    Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);


    std::cout << "This transformation can be replicated using:" << std::endl;
    std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
              << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " /" << parent_frame_
              << " /" << child_frame_ << " 10" << std::endl;

    std::cout << "Corresponding transformation matrix:" << std::endl
              << std::endl << current_guess_ << std::endl << std::endl;


    PublishCloud(calibrated_cloud_publisher_, output_cloud);

    tf::Transform t_transform;
    MatrixToTranform(current_guess_,t_transform);
    tf_br.sendTransform(tf::StampedTransform(t_transform, ros::Time::now(), parent_frame_, child_frame_));
}

void ROSMultiLidarCalibratorApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg,
                                                  const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud_msg)
{
    pcl::PointCloud<PointT>::Ptr parent_cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr child_cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr child_filtered_cloud (new pcl::PointCloud<PointT>);

	pcl::fromROSMsg(*in_parent_cloud_msg, *parent_cloud);
	pcl::fromROSMsg(*in_child_cloud_msg, *child_cloud);

	parent_frame_ = in_parent_cloud_msg->header.frame_id;
	child_frame_ = in_child_cloud_msg->header.frame_id;

	DownsampleCloud(child_cloud, child_filtered_cloud, voxel_size_);
	in_parent_cloud_ = parent_cloud;
	in_child_cloud_ = child_cloud;
    in_child_filtered_cloud_ = child_filtered_cloud;
}

void ROSMultiLidarCalibratorApp::DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                                                 pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
                                                 double in_leaf_size)
{
	pcl::VoxelGrid<PointT> voxelized;
	voxelized.setInputCloud(in_cloud_ptr);
	voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
	voxelized.filter(*out_cloud_ptr);
}


void ROSMultiLidarCalibratorApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
	//get params

	std::string initial_pose_topic_str = "/initialpose";
	std::string calibrated_points_topic_str = "/points_calibrated";

	// x, y, z, yaw, pitch, roll
    std::string init_file_path;
    in_private_handle.param<std::string>("init_params_file_path", init_file_path, " ");
    std::ifstream ifs(init_file_path);

    ifs>>child_topic_num_;

    for (int j = 0; j < child_topic_num_; ++j) {
        std::string child_name;
        ifs>>child_name;
        std::vector<double> tmp_transfer;
        for (int k = 0; k < 6; ++k) {
            // read xyzypr
            double tmp_xyzypr;
            ifs>>tmp_xyzypr;
            tmp_transfer.push_back(tmp_xyzypr);
        }
        transfer_map_.insert(std::pair<std::string, std::vector<double>>(child_name, tmp_transfer));
    }

	in_private_handle.param<std::string>("points_parent_src", points_parent_topic_str, "points_raw");
	ROS_INFO("[%s] points_parent_src: %s",__APP_NAME__, points_parent_topic_str.c_str());

	in_private_handle.param<std::string>("points_child_src", points_child_topic_str, "points_raw");
	ROS_INFO("[%s] points_child_src: %s",__APP_NAME__, points_child_topic_str.c_str());

	in_private_handle.param<double>("voxel_size", voxel_size_, 0.1);
	ROS_INFO("[%s] ndt_epsilon: %.2f",__APP_NAME__, voxel_size_);

	in_private_handle.param<double>("ndt_epsilon", ndt_epsilon_, 0.01);
	ROS_INFO("[%s] voxel_size: %.2f",__APP_NAME__, ndt_epsilon_);

	in_private_handle.param<double>("ndt_step_size", ndt_step_size_, 0.1);
	ROS_INFO("[%s] ndt_step_size: %.2f",__APP_NAME__, ndt_step_size_);

	in_private_handle.param<double>("ndt_resolution", ndt_resolution_, 1.0);
	ROS_INFO("[%s] ndt_resolution: %.2f",__APP_NAME__, ndt_resolution_);

	in_private_handle.param<int>("ndt_iterations", ndt_iterations_, 400);
	ROS_INFO("[%s] ndt_iterations: %d",__APP_NAME__, ndt_iterations_);

	//generate subscribers and synchronizer
	cloud_parent_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                     points_parent_topic_str, 1);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_parent_topic_str.c_str());

	cloud_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                        points_child_topic_str, 1);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_child_topic_str.c_str());

	calibrated_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(calibrated_points_topic_str, 1);
	ROS_INFO("[%s] Publishing PointCloud to... %s",__APP_NAME__, calibrated_points_topic_str.c_str());

	cloud_synchronizer_ =
			new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100),
			                                               *cloud_parent_subscriber_,
			                                               *cloud_child_subscriber_);
	cloud_synchronizer_->registerCallback(boost::bind(&ROSMultiLidarCalibratorApp::PointsCallback, this, _1, _2));

}


void ROSMultiLidarCalibratorApp::Run()
{
	ros::NodeHandle private_node_handle("~");

	InitializeROSIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);

	ros::Rate loop_rate(10);
	while (ros::ok()){

	    ros::spinOnce();

	    // start NDT process here
        PerformNdtOptimize();

        loop_rate.sleep();
	}

	ros::spin();

	ROS_INFO("[%s] END",__APP_NAME__);
}

ROSMultiLidarCalibratorApp::ROSMultiLidarCalibratorApp()
{
	//initialpose_quaternion_ = tf::Quaternion::getIdentity();
	current_guess_ = Eigen::Matrix4f::Identity();
}
