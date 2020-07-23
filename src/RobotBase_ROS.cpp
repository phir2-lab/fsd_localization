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

#include "RobotBase_ROS.h"

RobotBase_ROS::RobotBase_ROS(int argc, char* argv[])
{
//    int argc=0;
//    char** argv=NULL;

    ros::init(argc, argv, "control_robot");
    ROS_INFO("control_robot!");

    n_ = new ros::NodeHandle("~");
    rate_ = new ros::Rate(100);

    start = ros::WallTime::now();
    last = start;

    tf_buffer_ = new tf2_ros::Buffer;
    listener = new tf2_ros::TransformListener(*tf_buffer_);

    maxNumCams_ = 4;
    depthCloudROS_.resize(maxNumCams_);
    sub_depthCloud_.resize(maxNumCams_);


    n_->param<std::string>("mapName",mapName,"alma");
    mapName = mapName+="-s1";
    std::cout << "mapName " << mapName << std::endl;

    n_->param<std::string>("method",methodName,"INT_PCL_FSD");
    std::cout << "methodName " << methodName << std::endl;

    std::string datasetType;
    n_->param<std::string>("datasetType",datasetType,"multi_cam");
    std::cout << "datasetType " << datasetType << std::endl;

    if(datasetType.compare("single_cam")==0)
        numCams_ = 1;
    else //if(datasetType.compare("multi_cam")==0)
        numCams_ = 4;

    // n_->param<int>("numCams",numCams_,4);
    std::cout << "numCams " << numCams_ << std::endl;

    // Initialize publishers and subscribers
    sub_odom_ = n_->subscribe("/odom", 100, &RobotBase_ROS::receiveOdom, this);
    sub_laser_ = n_->subscribe("/scan", 100, &RobotBase_ROS::receiveLaser, this);

    for(int i=0; i<maxNumCams_; ++i){
        std::string topic = "/camera/RGBD_"+std::to_string(i+1)+"/depth/points_clean";
        std::cout << "Topic: " << topic << std::endl;
        sub_depthCloud_[i] = n_->subscribe<sensor_msgs::PointCloud2>(topic, 100, boost::bind(&RobotBase_ROS::receiveDepthCloud, this, _1, i));
    }


    srcPath = ros::package::getPath("fsd_localization");
    std::cout << "PATH " << srcPath << std::endl;

    fullMapName = srcPath+"/maps/"+mapName;

}

//////////////////////////////////
///// INITIALIZATION METHODS /////
//////////////////////////////////

bool RobotBase_ROS::initialize()
{

    return true;
}

int RobotBase_ROS::getNumCams()
{
    return numCams_;
}

////////////////////////////////////////////////////////////////
////// METHODS FOR READING ODOMETRY & SENSORS MEASUREMENTS /////
////////////////////////////////////////////////////////////////

bool RobotBase_ROS::readOdometryAndSensors()
{
    geometry_msgs::TransformStamped transform;

    double roll, pitch, yaw;

    // ********************* SLAM Pose *********************** //

    bool goodSLAMPose = true;

    try{
        // faster lookup transform so far
        transform = tf_buffer_->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) {
        ros::Duration(0.05).sleep();
        goodSLAMPose=false;
        return false;
    }

    if(goodSLAMPose){
        slamPose_.x = transform.transform.translation.x;
        slamPose_.y = transform.transform.translation.y;

        tf::Quaternion q4(transform.transform.rotation.x,
                          transform.transform.rotation.y,
                          transform.transform.rotation.z,
                          transform.transform.rotation.w);
        tf::Matrix3x3 m4(q4);
        m4.getRPY(roll,pitch,yaw);

        slamPose_.theta = RAD2DEG(yaw);
    }

//    std::cout << "SLAM : " << slamPose_ << std::endl;

    // ********************* Odometry *********************** //
   odometry_.x = odomROS_.pose.pose.position.x;
   odometry_.y = odomROS_.pose.pose.position.y;

   tf::Quaternion q1(odomROS_.pose.pose.orientation.x,
                    odomROS_.pose.pose.orientation.y,
                    odomROS_.pose.pose.orientation.z,
                    odomROS_.pose.pose.orientation.w);
   tf::Matrix3x3 m1(q1);
   m1.getRPY(roll,pitch,yaw);

   odometry_.theta = RAD2DEG(yaw);

//    std::cout << "ODOM: " << odometry_ << std::endl;


    // ********************* Laser 2D *********************** //
    numLasers_ = laserROS_.ranges.size();
//    std::cout << " - LASER: " << numLasers_ << std::endl;
    if(numLasers_ > 0){
        angleLaserIncrement =  RAD2DEG(laserROS_.angle_increment);
        minAngleLaser_ = RAD2DEG(laserROS_.angle_min);
        maxAngleLaser_ = RAD2DEG(laserROS_.angle_max);
        minLaserRange_ = laserROS_.range_min;
        maxLaserRange_ = laserROS_.range_max;
//        std::cout << " angle: " << minAngleLaser_ << ' ' << maxAngleLaser_ << " inc " << angleLaserIncrement << std::endl;
//        std::cout << " range: " << minLaserRange_ << ' ' << maxLaserRange_ << std::endl;


        lasers_.resize(numLasers_);
        for(int i=0; i<laserROS_.ranges.size(); i++){
            lasers_[i] = laserROS_.ranges[numLasers_-i-1];
            if(lasers_[i]<0)
                lasers_[i] = maxLaserRange_;
        }
    }

    // ********************* Depth Point Cloud *********************** //
    int flag = 0;
    numDepthPoints_ = 0;
    for(int pc=0; pc<maxNumCams_; ++pc){
//        std::cout << "DEPTH " << pc << ' ' << depthCloudROS_[pc].header.frame_id << " seq " << depthCloudROS_[pc].header.seq << " width " << depthCloudROS_[pc].width << " height " << depthCloudROS_[pc].height << std::endl;
        numDepthPoints_ += depthCloudROS_[pc].width * depthCloudROS_[pc].height;
        if(depthCloudROS_[pc].width * depthCloudROS_[pc].height > 0)
            flag++;
    }
//    std::cout << "total " << numDepthPoints_ << std::endl;

//    std::cout << "DEPTH " << depthCloudROS1_.header.frame_id << " seq " << depthCloudROS1_.header.seq << " width " << depthCloudROS1_.width << " height " << depthCloudROS1_.height << std::endl;
//    numDepthPoints_ = depthCloudROS1_.width * depthCloudROS1_.height;
    if(flag == numCams_){

        depthCloud_.resize(numDepthPoints_);
        int p=0;

        for(int pc=0; pc<maxNumCams_; ++pc){

            if(depthCloudROS_[pc].width * depthCloudROS_[pc].height == 0)
                continue;


            sensor_msgs::PointCloud2 transformed_cloud;

            try
            {
                transform = tf_buffer_->lookupTransform("map", depthCloudROS_[pc].header.frame_id, depthCloudROS_[pc].header.stamp, ros::Duration(3.0));
                tf2::doTransform(depthCloudROS_[pc], transformed_cloud, transform);
            }
            catch (tf2::TransformException& ex)
            {
                ROS_WARN("%s", ex.what());
                ROS_WARN("%s", depthCloudROS_[pc].header.frame_id.c_str());
                continue;
            }

    //        sensor_msgs::PointCloud2Modifier modifier(depthCloudROS_);
    //        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

            sensor_msgs::PointCloud2Iterator<float> iter_x(transformed_cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(transformed_cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(transformed_cloud, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(transformed_cloud, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(transformed_cloud, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(transformed_cloud, "b");

            for(size_t i=0; i<depthCloudROS_[pc].width * depthCloudROS_[pc].height;
                ++i, ++p, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b){

                depthCloud_[p].x = *iter_x;
                depthCloud_[p].y = *iter_y;
                depthCloud_[p].z = *iter_z;
            }
        }

        seconds_ = laserROS_.header.stamp.sec + ((double)laserROS_.header.stamp.nsec)/1000000000.0;
//        std::cout << "seconds " << seconds_ << " " << seconds_-initialSeconds_ << std::endl;

        if(initialSeconds_==-1.0){
            initialSeconds_ = seconds_;
            seconds_ = 0;
        }else{
            seconds_ -= initialSeconds_;
        }
    }

    return true;
}

void RobotBase_ROS::receiveOdom(const nav_msgs::Odometry::ConstPtr &value)
{

//  STRUCTURE OF nav_msgs::Odometry

//# This represents an estimate of a position and velocity in free space.
//# The pose in this message should be specified in the coordinate frame given by header.frame_id.
//# The twist in this message should be specified in the coordinate frame given by the child_frame_id

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    odomROS_.header = value->header;

    //string child_frame_id
    odomROS_.child_frame_id = value->child_frame_id;

    //geometry_msgs/PoseWithCovariance pose
        //# This represents a pose in free space with uncertainty.
        //Pose pose
            //# A representation of pose in free space, composed of position and orientation.
            //Point position
                //# This contains the position of a point in free space
                //float64 x
                //float64 y
                //float64 z
            //Quaternion orientation
                //# This represents an orientation in free space in quaternion form.
                //float64 x
                //float64 y
                //float64 z
                //float64 w
        //# Row-major representation of the 6x6 covariance matrix
        //# The orientation parameters use a fixed-axis representation.
        //# In order, the parameters are:
        //# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        //float64[36] covariance
    odomROS_.pose = value->pose;

    //geometry_msgs/TwistWithCovariance twist
        //# This expresses velocity in free space with uncertainty.
        //Twist twist
            //# This expresses velocity in free space broken into its linear and angular parts.
            //Vector3  linear
                //float64 x
                //float64 y
                //float64 z
            //Vector3  angular
                //float64 x
                //float64 y
                //float64 z
        //# Row-major representation of the 6x6 covariance matrix
        //# The orientation parameters use a fixed-axis representation.
        //# In order, the parameters are:
        //# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        //float64[36] covariance
    odomROS_.twist = value->twist;
}

void RobotBase_ROS::receiveLaser(const sensor_msgs::LaserScan::ConstPtr &value)
{
//  STRUCTURE OF sensor_msgs::LaserScan

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    //             # timestamp in the header is the acquisition time of
    //             # the first ray in the scan.
    //             #
    //             # in frame frame_id, angles are measured around
    //             # the positive Z axis (counterclockwise, if Z is up)
    //             # with zero angle being forward along the x axis
    laserROS_.header = value->header;

    //float32 angle_min        # start angle of the scan [rad]
    //float32 angle_max        # end angle of the scan [rad]
    //float32 angle_increment  # angular distance between measurements [rad]
    laserROS_.angle_min = value->angle_min;
    laserROS_.angle_max = value->angle_max;
    laserROS_.angle_increment = value->angle_increment;

    //float32 time_increment   # time between measurements [seconds] - if your scanner
    //                         # is moving, this will be used in interpolating position
    //                         # of 3d points
    //float32 scan_time        # time between scans [seconds]
    laserROS_.time_increment = value->time_increment;
    laserROS_.scan_time = value->scan_time;

    //float32 range_min        # minimum range value [m]
    //float32 range_max        # maximum range value [m]
    laserROS_.range_min = value->range_min;
    laserROS_.range_max = value->range_max;

    //float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
    //float32[] intensities    # intensity data [device-specific units].  If your
    //                         # device does not provide intensities, please leave
    //                         # the array empty.
    laserROS_.ranges = value->ranges;
    laserROS_.intensities = value->intensities;
}

void RobotBase_ROS::receiveDepthCloud(const sensor_msgs::PointCloud2::ConstPtr &value, int num)
{
//  STRUCTURE OF sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2* depthCloudROS;
    if(num<0 || num>3)
        return;

    depthCloudROS = &(depthCloudROS_[num]);

    //# This message holds a collection of N-dimensional points, which may
    //# contain additional information such as normals, intensity, etc. The
    //# point data is stored as a binary blob, its layout described by the
    //# contents of the "fields" array.

    //# The point cloud data may be organized 2d (image-like) or 1d
    //# (unordered). Point clouds organized as 2d images may be produced by
    //# camera depth sensors such as stereo or time-of-flight.

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    depthCloudROS->header = value->header;

    //# 2D structure of the point cloud. If the cloud is unordered, height is
    //# 1 and width is the length of the point cloud.
    depthCloudROS->height = value->height;
    depthCloudROS->width = value->width;

    //# Describes the channels and their layout in the binary data blob.
    //PointField[] fields
    //    # This message holds the description of one point entry in the
    //    # PointCloud2 message format.
    //    uint8 INT8    = 1
    //    uint8 UINT8   = 2
    //    uint8 INT16   = 3
    //    uint8 UINT16  = 4
    //    uint8 INT32   = 5
    //    uint8 UINT32  = 6
    //    uint8 FLOAT32 = 7
    //    uint8 FLOAT64 = 8

    //    string name      # Name of field
    //    uint32 offset    # Offset from start of point struct
    //    uint8  datatype  # Datatype enumeration, see above
    //    uint32 count     # How many elements in the field
    depthCloudROS->fields = value->fields;

    depthCloudROS->is_bigendian = value->is_bigendian;  // # Is this data bigendian? (bool)
    depthCloudROS->point_step = value->point_step;      // # Length of a point in bytes (uint32)
    depthCloudROS->row_step = value->row_step;          // # Length of a row in bytes (uint32)

    //uint8[] data         # Actual point data, size is (row_step*height)
    depthCloudROS->data = value->data;

    depthCloudROS->is_dense = value->is_dense;      // # True if there are no invalid points (bool)
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void RobotBase_ROS::resumeMovement()
{
    ros::spinOnce();
    rate_->sleep();
}






