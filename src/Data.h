//
// Created by lab on 2020/6/15.
//

#ifndef SRC_DATA_H
#define SRC_DATA_H

//#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include "parameters.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/package.h>

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <fstream>
#include <condition_variable>
#include <iostream>


using namespace cv;
using namespace std;
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace Eigen;

class Data
{
public:
    Data();
    ~Data();
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Ps[(WINDOW_SIZE + 1)];
    std::vector<Eigen::Vector3d> vPath_to_draw;
    std::map<double, Eigen::Vector3d> vPath_stamped;

    Eigen::Vector3d current_pos;
    std::string solver_flag;
    cv::Mat tracked_img;
    bool shouldUpdate;
    bool hasloop;
    cv::Mat loop_match_img;
    cv::Mat show_img;
    std::map< double, std::pair<Matrix3d,Vector3d> > loop_path;


    Matrix3d current_Rot;
    Matrix3d current_Rot_rct;
    std::map< double, std::vector<Vector3d> > rect_point_buf;
    std::map< double, std::vector<Vector3d> > marged_point_buf;
    std::vector<double> loopPath_stamp;
    std::map<int,std::pair<Eigen::Vector3d,Eigen::Vector3d>> loop_links;
    std::mutex m_data;

    int win_cnt;
    int pose_cnt;
    void img_callback(const sensor_msgs::ImageConstPtr& img_msg);
    void img_loop_callback(const sensor_msgs::ImageConstPtr& img_msg);
    void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg);
    void vio_loop_callback(const nav_msgs::Path::ConstPtr &path_msg);
    void rect_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg);
    void marged_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg);
    void loop_stamp_callback(const visualization_msgs::MarkerArray::ConstPtr &array);

};


#endif //SRC_DATA_H
