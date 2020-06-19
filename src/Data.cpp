//
// Created by lab on 2020/6/15.
//

#include "Data.h"
Data::Data()
{
    shouldUpdate = true;
    hasloop = false;
    tracked_img = cv::Mat(480*2, 752 * 2, CV_8UC3, cv::Scalar(0, 0, 0));
    loop_match_img = cv::Mat(480, 752 * 2, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::vconcat(tracked_img, loop_match_img, show_img);

    vPath_to_draw.push_back(Eigen::Vector3d(0,0,0));
    pose_cnt = 0;
    win_cnt = 0;

}

Data::~Data()
{

}

void Data::img_callback(const sensor_msgs::ImageConstPtr& img_msg)
{
    m_data.lock();
    tracked_img =  cv_bridge::toCvShare(img_msg,"bgr8")->image;
    cv::vconcat(tracked_img, loop_match_img, show_img);
    shouldUpdate = true;
    m_data.unlock();
}


void Data::img_loop_callback(const sensor_msgs::ImageConstPtr& img_msg)
{

    m_data.lock();
    loop_match_img =  cv_bridge::toCvShare(img_msg,"bgr8")->image;
//    loop_match_img.resize();
    resize(loop_match_img, loop_match_img, tracked_img.size());
    cv::vconcat(tracked_img, loop_match_img, show_img);
    shouldUpdate = true;
    hasloop = true;

    m_data.unlock();
}


void
Data::vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_data.lock();

    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;

    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    current_Rot = vio_q.toRotationMatrix();
    vPath_to_draw.emplace_back(vio_t);

    vPath_stamped[pose_msg->header.stamp.toSec()] = vio_t;

    if(win_cnt < (WINDOW_SIZE+1) ) {
        Rs[win_cnt] = current_Rot;
        Ps[win_cnt] = vio_t;
        win_cnt++;
        pose_cnt++;

    } else {

        for (int i = 0; i < WINDOW_SIZE; ++i){
            Rs[i] = Rs[i+1];
            Ps[i] = Ps[i+1];
        }
        Rs[WINDOW_SIZE] = current_Rot;
        Ps[WINDOW_SIZE] = vio_t;

        pose_cnt++;
        solver_flag = "NON_LINEAR";
    }

    m_data.unlock();
}

void
Data::vio_loop_callback(const nav_msgs::Path::ConstPtr &path_msg)
{
    m_data.lock();
    Quaterniond vio_q;
    Vector3d vio_t;
    Eigen::Matrix3d Rx;
    for (auto &it:path_msg->poses)
    {
        vio_t= Vector3d(it.pose.position.x, it.pose.position.y, it.pose.position.z);
        vio_q.w() = it.pose.orientation.w;
        vio_q.x() = it.pose.orientation.x;
        vio_q.y() = it.pose.orientation.y;
        vio_q.z() = it.pose.orientation.z;
        Rx = vio_q.toRotationMatrix();
        loop_path[it.header.stamp.toSec()] = make_pair(Rx, vio_t);
    }
    auto it = loop_path.end();
    it--;
    current_Rot_rct = it->second.first;
    m_data.unlock();
}

void
Data::rect_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg){
    rect_point_buf.clear();
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        Vector3d p_3d;
        p_3d.x() = point_msg->points[i].x;
        p_3d.y() = point_msg->points[i].y;
        p_3d.z() = point_msg->points[i].z;
        rect_point_buf[point_msg->header.stamp.toSec()].push_back(p_3d);
    }
}

void
Data::marged_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg){
//    rect_point_buf[point_msg.header.stamp.toSec()].push_back();
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        Vector3d p_3d;
        p_3d.x() = point_msg->points[i].x;
        p_3d.y() = point_msg->points[i].y;
        p_3d.z() = point_msg->points[i].z;
        marged_point_buf[point_msg->header.stamp.toSec()].push_back(p_3d);
    }

}

void
Data::loop_stamp_callback(const visualization_msgs::MarkerArray::ConstPtr &array){

    for(int i = 0;i<array->markers.size();i++){
        loop_links[i].first = Eigen::Vector3d(array->markers[i].points[0].x,array->markers[i].points[0].y,array->markers[i].points[0].z);
        loop_links[i].second = Eigen::Vector3d(array->markers[i].points[1].x,array->markers[i].points[1].y,array->markers[i].points[1].z);

    }
}