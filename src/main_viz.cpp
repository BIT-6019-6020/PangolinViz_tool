/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <ros/ros.h>

#include <vector>


#include <thread>

#include <condition_variable>



#include "Draw.h"
#include "Data.h"

using namespace cv;
using namespace std;
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;

std::shared_ptr<Draw> pDraw;
Data* pData;


int main(int argc, char **argv)
{

    ros::init(argc, argv, "dm_svi_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    pData = new Data;
    pDraw.reset(new Draw(pData));


    std::thread thd_Draw(&Draw::drawThread, pDraw);
    thd_Draw.join();

    ros::Subscriber sub_imu;
    ros::Subscriber sub_img0;


    ros::Subscriber frature_image = n.subscribe("/vins_estimator/image_track", 2000, &Data::img_callback, pData);
    ros::Subscriber loop_match_image = n.subscribe("/loop_fusion/match_image", 2000, &Data::img_loop_callback, pData);

    ros::Subscriber odometry_rect = n.subscribe("/vins_estimator/odometry", 100, &Data::vio_callback, pData);
    ros::Subscriber pose_graph_path = n.subscribe("/loop_fusion/pose_graph_path", 100, &Data::vio_loop_callback, pData);
    ros::Subscriber rect_point_cloud = n.subscribe("/loop_fusion/point_cloud_loop_rect", 100, &Data::rect_point_callback, pData);
    ros::Subscriber marged_point_cloud = n.subscribe("/loop_fusion/margin_cloud_loop_rect", 100, &Data::marged_point_callback, pData);
    ros::Subscriber loop_stamp = n.subscribe("/loop_fusion/pose_graph", 100, &Data::loop_stamp_callback, pData);

    ros::spin();

    delete pData;

    return 0;
}
