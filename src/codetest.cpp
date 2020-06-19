//
// Created by lab on 2020/6/16.
//


#include <eigen3/Eigen/Dense>
#include "parameters.h"
#include <eigen3/Eigen/Dense>
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
#include <string>
using namespace std;
using namespace Eigen;
int main(){
    std::map<double,string> data;
    std::map< double, std::pair<Matrix3d,Vector3d> > path;
    Matrix3d R;

    R <<1, 0, 0, 0, 1, 0, 0, 0, 2;
    Vector3d t;
    t << 1, 1, 1;

    vector<double> data_double = {1.1, 2.2, 4.4, 3.3, 5.5};
    vector<string> data_string = {"a", "b", "c", "d", "e"};

    for (int i = 0; i < 5; ++i)
    {
        data[data_double[i]] = data_string[i];
        path[data_double[i]] = make_pair(data_double[i]*R, data_double[i]*t);
    }

    cout<<"data test:"<<data[4.4]<<endl;

    for(auto &it:data){
        cout<<"it.first :"<<it.first<<endl;
        cout<<"it.second:"<<it.second<<endl;
    }


    for(auto &it:path){  ///自动排序
        cout<<"--------------------"<<endl;
        cout<<"stamp :"<<it.first<<endl;
        cout<<"   R:\n"<<it.second.first<<endl;
        cout<<"   t:\n"<<it.second.second<<endl;
    }
    cout<<"--------------------"<<endl;

    auto iter = path.end(); iter --;
    cout<<iter->second.second<<endl;
    cout<<iter->second.first<<endl;

}