/**
 * This file is part of FSD Localization

 * Copyright (c) 2020 Renan Maffei <rqmaffei at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * For more information see <https://github.com/phir2-lab/FSD_localization>

 * FSD Localization is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * FSD Localization is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with FSD Localization.  If not, see <https://www.gnu.org/licenses/>.
**/

#ifndef ROBOTBASE_ROS_H
#define ROBOTBASE_ROS_H

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "Grid.h"
#include "Utils.h"
#include "RobotBase.h"

class RobotBase_ROS : public RobotBase
{
public:
    RobotBase_ROS(int argc, char *argv[]);

    // ROS stuff
    bool initialize();
    void resumeMovement();

    // Sensors stuff
    bool readOdometryAndSensors();
    int getNumCams();

private:

    ros::NodeHandle* n_;
    ros::Rate* rate_;
    tf2_ros::TransformListener* listener;
    tf2_ros::Buffer* tf_buffer_;

    ros::WallTime start, last, current;

    nav_msgs::Odometry odomROS_;
    geometry_msgs::PoseStamped poseOdomROS_;
    sensor_msgs::LaserScan laserROS_;
    std::vector<sensor_msgs::PointCloud2> depthCloudROS_;

    // Ros topics subscribers
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_laser_;
    std::vector<ros::Subscriber> sub_depthCloud_;

    int numCams_, maxNumCams_;


    void receiveOdom(const nav_msgs::Odometry::ConstPtr &value);
    void receivePoseOdom(const geometry_msgs::PoseStamped::ConstPtr &value);
    void receiveLaser(const sensor_msgs::LaserScan::ConstPtr &value);
    void receiveDepthCloud(const sensor_msgs::PointCloud2::ConstPtr &value, int num);


};

#endif // ROBOTBASE_ROS_H
