lidar_camera_offline_calibration
===========
![ROS](https://img.shields.io/badge/ROS-Kinetic-brightgreen.svg)  ![OS](https://img.shields.io/badge/OS-Ubuntu%2016.04-orange.svg ) ![OpenCV](https://img.shields.io/badge/OpenCV-3.0-blue.svg)

The center of the sphere is calculated from the LiDAR point cloud and the image. In the LiDAR point cloud, the center of the ellipse is steadily determined using RANSAC. In the image, the contour of the sphere is detected by the edge, and then the ellipse is fitted to the edge point. Since the size of the ellipse is known,according to the ellipse parameters,the center of the sphere is determined and the external parameters of the LiDAR and camera pair are finally given.

## Usage:

Follow the steps below to use this (`lidar_camera_offline_calibration`) package:

1. [Create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) (if you do not have one setup already). 
1. Navigate to the `src` folder in your catkin workspace: `cd ~/catkin_ws/src`
1. Clone this repository: `git clone https://github.com/GroupOfTHUMengShiGuo/lidar_camera_offline_calibration.git`
1. Compile and build the package: `cd ~/catkin_ws && catkin_make`
1. Add the catkin workspace to your ROS environment: `source ~/catkin_ws/devel/setup.bash`
1. Run the ROS launch in this package: `roslaunch lidar_camera_offline_calibration lidar_camera_offline_calibration.launch`


## Subscribed topics:
-------------------
- /pointcloud_input(sensor_msgs/PointCloud2)
    - you can change the subscribed pointcloud topic by changing the parameter "pointcloud_input" in the launch file.
- /image_input(sensor_msgs/Image)
    - to get the image input. You should specify image topic by the parameter "image_input" in the launch file.

## Instructions:

1. Run the Calibration tools: `roslaunch lidar_camera_offline_calibration lidar_camera_offline_calibration.launch`
1. Play data or run sensors: `rosbag play -l -r 0.5 xxxxxx.bag`
1. In the Panel interface, the color can be selected through the HSV range and adjusted according to the recognition effect.
1. In the panel interface, the range of candidate point clouds can be reduced by adjusting the xyz range of point clouds, and R is the radius of the sphere props used. When the point cloud display interface, red points represent the candidate range and green points represent the identification result of the sphere.
1. Observe the image recognition effect and point cloud recognition effect. When the effect is good, click Record button at the bottom of panel interface to Record a group of corresponding points.
1. Move the sphere to the appropriate position for many times, record multiple sets of point pairs, at least 4 sets. In order to improve the accuracy, record as many sets as possible, and try to ensure that the position of the sphere varies greatly each time.
1. When at least four groups of point pairs are recorded, click Count key at the bottom of panel interface to calculate external parameters, and the calculated results are saved in test_result.txt file.
1. The Panel interface Clear key is used to Clear the currently saved pair records, the Delete button is used to Delete the last saved pair, and the ShowInfo button is used to view the currently saved pair information.The TestResult button is used to test the calibration result,and The calibration result is saved in the file test_resul.PNG by point cloud projected onto the picture.

## To do list 

* Improved image and point cloud sphere recognition algorithm.
* Improved external reference solution algorithm.
* After the completion of the calibration, the point cloud projection topic was added to reflect the calibration accuracy more intuitively.

