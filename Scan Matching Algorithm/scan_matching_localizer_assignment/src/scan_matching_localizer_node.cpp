/*
 * scan_matching_localizer_node.cpp
 *
 *  Created on: Apr 30, 2021
 *      Author: Daegyu Lee
 */

 // NOTE
 // 1. Please submit a report discussing the result of registration algorithm,
 // 2. Please feel the code where I wrote as "TODO:#".
 // 3. Import 2-D occupancy grid map using map_server(please refer to launch file.) 
 // 4. You should give a initial pose using RVIZ 2D Pose Estimate tool.
 // 5. If you have query things, feel free to send me a mail : "lee.dk@kaist.ac.kr"
 // 6. Run a bag file and compare the result.


 // headers in ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// headers in STL
#include <memory>
#include <cmath>
#include <type_traits>
#include <stdio.h>
#include <float.h>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <unordered_map>
#include <bits/stdc++.h>
#include <mutex>
#include <thread>

// PCL Libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pclomp/ndt_omp.h>

#include <laser_geometry/laser_geometry.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

class ScanLocalizer
{
public:
    ScanLocalizer(ros::NodeHandle& nh);
    ~ScanLocalizer();
    void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);
    void OccupancyGrid2DMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
    void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void ICPMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr& InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr& TargetData);
    void NDTMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr& InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr& TargetData);

    ros::NodeHandle nh_;
    ros::Subscriber subScan;
    ros::Subscriber subOccupancyGrid;
    ros::Subscriber subInitPose;

    ros::Publisher pubPointCloud;
    ros::Publisher pubStaticMapPointCloud;
    ros::Publisher pubTransformedCloud;
    ros::Publisher pubOdometry;


    nav_msgs::Odometry m_odomScan;

    pcl::PointCloud<pcl::PointXYZI>::Ptr m_StaticMap_ptr;
    bool bRvizInit;
    Eigen::Matrix4f prev_guess, init_guess;
};

ScanLocalizer::ScanLocalizer(ros::NodeHandle& nh) : nh_(nh), bRvizInit(false)
{
    subScan = nh_.subscribe("/scan", 1, &ScanLocalizer::LaserScanCallback, this);
    subOccupancyGrid = nh_.subscribe("/map", 1, &ScanLocalizer::OccupancyGrid2DMapCallback, this);
    subInitPose = nh.subscribe("/initialpose", 1, &ScanLocalizer::InitPoseCallback, this);

    pubPointCloud = nh_.advertise<sensor_msgs::PointCloud2>("/scan_to_pointcloud2", 1, true);
    pubStaticMapPointCloud = nh_.advertise<sensor_msgs::PointCloud2>("/map_to_pointcloud2", 1, true);
    pubTransformedCloud = nh_.advertise<sensor_msgs::PointCloud2>("/registered_points", 1, true);
    pubOdometry = nh_.advertise<nav_msgs::Odometry>("/odom", 1, true);
};

ScanLocalizer::~ScanLocalizer()
{
    ROS_INFO("ScanLocalizer destructor.");
}

void ScanLocalizer::InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z).toRotationMatrix();
    prev_guess.block(0, 0, 3, 3) = mat3;
    prev_guess(0, 3) = msg.pose.pose.position.x;
    prev_guess(1, 3) = msg.pose.pose.position.y;
    bRvizInit = true;
    ROS_INFO("INIT POSE");
}

void ScanLocalizer::LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    std::cout << "lasercallback" << std::endl;
    ROS_INFO("LASERCALLBACK1");
    if (msg->ranges.empty())
    {
        ROS_ERROR("Empty scan data");
        return;
    }

    // sensor_msgs::PointCloud2 cloud;
    // cloud.header = msg->header;

    // laser_geometry::LaserProjection projector;
    // tf::TransformListener tf;
    // projector.transformLaserScanToPointCloud(msg->header.frame_id, *msg, cloud, tf);

    // pcl::PointCloud<pcl::PointXYZI> scan;
    // pcl::fromROSMsg(cloud, scan);

    // Feel the code to convert the laser scan data to the pointcloud type(pcl::PointCloud<pcl::PointXYZI>)    
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        pcl::PointXYZI pointBuf;
        //TODO:#1
        // // feel the code here
        // pointBuf.x=scan[i].x;
        // pointBuf.y=scan[i].y;
        // pointBuf.z=scan[i].z;
        // // pointBuf.intensity=scan[i].intensity;
        // cloud_in_ptr->points.push_back(pointBuf);

        float x = msg->ranges[i] * std::cos(msg->angle_min + i * msg->angle_increment);
        float y = msg->ranges[i] * std::sin(msg->angle_min + i * msg->angle_increment);
        float z = 0;
        pointBuf.x = x;
        pointBuf.y = y;
        pointBuf.z = z;
        cloud_in_ptr->points.push_back(pointBuf);

    }
    // static laser_geometry::LaserProjection projector;
    // sensor_msgs::PointCloud2 pc2_dst;
    // projector.projectLaser(*msg, pc2_dst,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
    // pc2_dst.header.frame_id = "map";
    // pcl::fromROSMsg(pc2_dst, *cloud_in_ptr);
    // std::cout<<"here"<<std::endl;
    ROS_INFO("LASER CALLBACK2");
    sensor_msgs::PointCloud2 LaserToPointCloud2Msg;
    pcl::toROSMsg(*cloud_in_ptr, LaserToPointCloud2Msg);
    LaserToPointCloud2Msg.header = msg->header;
    pubPointCloud.publish(LaserToPointCloud2Msg);


    /*Scan matching algorithm*/
    if (cloud_in_ptr->points.empty() || m_StaticMap_ptr->points.empty())
        return;
    ROS_INFO("LASER CALLBACK3");

    // Implement below two algorithm and compare the result.
    // You can switch the registration algorithm between ICP and NDT_OMP
    // TODO:#2
    // for ICP(No multi-thread) uncomment
    //ICPMatching(cloud_in_ptr, m_StaticMap_ptr); //Too slow
    // for NDT_OMP(multi-thread) uncomment
    NDTMatching(cloud_in_ptr, m_StaticMap_ptr);
}

void ScanLocalizer::OccupancyGrid2DMapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    // Feel the code to convert the laser scan data to the pointcloud type(pcl::PointCloud<pcl::PointXYZI>)    
    ROS_INFO("MAP IS LOADED?");
    m_StaticMap_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
    for (unsigned int width = 0; width < msg->info.width; width++) {
        for (unsigned int height = 0; height < msg->info.height; height++) {

            //TODO:#3
            // feel the code here.
            //Convert occupied grid to the x-y coordinates to put the target(pcl::PointCloud<pcl::PointXYZI>)
            if (msg->data[height * msg->info.width + width] > 80) {
                float x = width * msg->info.resolution + msg->info.resolution / 2 + msg->info.origin.position.x;
                float y = height * msg->info.resolution + msg->info.resolution / 2 + msg->info.origin.position.y;
                float z = 0;

                pcl::PointXYZI pointBuf;
                pointBuf.x = x;
                pointBuf.y = y;
                pointBuf.z = z;
                m_StaticMap_ptr->points.push_back(pointBuf);
            }

        }
    }

    sensor_msgs::PointCloud2 StaticMapToPointCloud2Msg;
    pcl::toROSMsg(*m_StaticMap_ptr, StaticMapToPointCloud2Msg);
    StaticMapToPointCloud2Msg.header.frame_id = "map";
    StaticMapToPointCloud2Msg.header.stamp = msg->header.stamp;
    pubStaticMapPointCloud.publish(StaticMapToPointCloud2Msg);

    ROS_INFO("MAP IS LOADED");
}


void ScanLocalizer::ICPMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr& InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr& TargetData)
{
    std::cout << "icp start" << std::endl;
    // You should initialize the init pose using RVIZ 2D pose estimate tool.
    if (!bRvizInit)
    {
        init_guess = Eigen::Matrix4f::Identity();
    }
    else
    {
        init_guess = prev_guess;
    }
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    // set the MaximumIterations, InputSource, InputTarget
    // TODO:#4
    // feel the code here
    icp.setMaximumIterations(80);
    icp.setInputSource(InputData);
    icp.setInputTarget(TargetData);

    // Run registration algorithm, and put the transformation matrix of previous step.
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
    icp.align(*result, init_guess);

    if (icp.hasConverged())
    {
        std::cout << "converged." << std::endl
            << "The score is " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    }
    else
    {
        return;
    }

    init_guess.block<3, 3>(0, 0) = icp.getFinalTransformation().block<3, 3>(0, 0);
    init_guess.block<3, 1>(0, 3) = icp.getFinalTransformation().block<3, 1>(0, 3);

    //publish registered cloud 
    //Convert transformed(registered) pointcloud using ICP algorithm.
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    //TODO:#5
    //feel the code here --> convert pcl pointcloud to the ros msg. 
    pcl::transformPointCloud(*InputData, *final_cloud, init_guess);

    sensor_msgs::PointCloud2 FinalCloudToPointCloud2Msg;
    pcl::toROSMsg(*final_cloud, FinalCloudToPointCloud2Msg); // or pcl::toROSMsg(*result, FinalCloudToPointCloud2Msg);
    FinalCloudToPointCloud2Msg.header.frame_id = "map";
    // FinalCloudToPointCloud2Msg.header.stamp = msg->header.stamp;
    FinalCloudToPointCloud2Msg.header.stamp = ros::Time::now();
    pubTransformedCloud.publish(FinalCloudToPointCloud2Msg);// topic name: "/registered_points"

    //publish Odometry
    //TODO:#6
    //feel the code here --> Publish the result using nav_msgs/Odometry topic. 
    tf::StampedTransform tf_transform;
    geometry_msgs::TransformStamped odom_trans;
    ros::Time current_time = ros::Time::now();
    tf::TransformBroadcaster odom_broadcaster;
    Eigen::Quaterniond eigen_quat(init_guess.block<3, 3>(0, 0).cast<double>());
    Eigen::Vector3d eigen_trans(init_guess.block<3, 1>(0, 3).cast<double>());
    tf::Quaternion tf_quat;
    tf::Vector3 tf_trans;
    // we need to create quaternions
    tf::quaternionEigenToTF(eigen_quat, tf_quat);

    tf::vectorEigenToTF(eigen_trans, tf_trans);

    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(tf_quat, odom_quat);

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = tf_trans.getX();
    odom_trans.transform.translation.y = tf_trans.getY();
    odom_trans.transform.translation.z = tf_trans.getZ();
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    m_odomScan.header.stamp = current_time;
    m_odomScan.header.frame_id = "map"; 
    m_odomScan.pose.pose.position.x = tf_trans.getX();
    m_odomScan.pose.pose.position.y = tf_trans.getY();
    m_odomScan.pose.pose.position.z = tf_trans.getZ();
    m_odomScan.pose.pose.orientation = odom_quat;

    m_odomScan.child_frame_id = "base_link";
    double temp = 0.01;
    m_odomScan.twist.twist.linear.x = temp;
    m_odomScan.twist.twist.linear.y = temp;
    m_odomScan.twist.twist.angular.z = temp;
    // publish
    pubOdometry.publish(m_odomScan);
    prev_guess = init_guess;
}

void ScanLocalizer::NDTMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr& InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr& TargetData)
{
    if (!bRvizInit)
    {
        init_guess = Eigen::Matrix4f::Identity();
    }
    else
    {
        init_guess = prev_guess;
    }

    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setInputSource(InputData);
    ndt->setInputTarget(TargetData);
    ndt->setTransformationEpsilon(0.01);
    ndt->setMaximumIterations(45);
    ndt->setResolution(1.0);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setNumThreads(5);

    //TODO:#7
    //Feel the code here
    // 
    // all the code from above

    //1
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
    ndt->align(*result, init_guess);
    //2
    if (ndt->hasConverged())
    {
        std::cout << "converged." << std::endl
            << "The score is " << ndt->getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << ndt->getFinalTransformation() << std::endl;
    }
    else
    {
        return;
    }

    // 3
    init_guess.block<3, 3>(0, 0) = ndt->getFinalTransformation().block<3, 3>(0, 0);
    init_guess.block<3, 1>(0, 3) = ndt->getFinalTransformation().block<3, 1>(0, 3);

    // 4
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*InputData, *final_cloud, init_guess);
    sensor_msgs::PointCloud2 FinalCloudToPointCloud2Msg;
    pcl::toROSMsg(*result, FinalCloudToPointCloud2Msg);
    FinalCloudToPointCloud2Msg.header.frame_id = "map";
    // FinalCloudToPointCloud2Msg.header.stamp
    FinalCloudToPointCloud2Msg.header.stamp = ros::Time::now();
    pubTransformedCloud.publish(FinalCloudToPointCloud2Msg);

    // 5
    tf::StampedTransform tf_transform;
    geometry_msgs::TransformStamped odom_trans;
    ros::Time current_time = ros::Time::now();
    tf::TransformBroadcaster odom_broadcaster;
    Eigen::Quaterniond eigen_quat(init_guess.block<3, 3>(0, 0).cast<double>());
    Eigen::Vector3d eigen_trans(init_guess.block<3, 1>(0, 3).cast<double>());
    tf::Quaternion tf_quat;
    tf::Vector3 tf_trans;

    tf::quaternionEigenToTF(eigen_quat, tf_quat);
    tf::vectorEigenToTF(eigen_trans, tf_trans);

    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(tf_quat, odom_quat);

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = tf_trans.getX();
    odom_trans.transform.translation.y = tf_trans.getY();
    odom_trans.transform.translation.z = tf_trans.getZ();
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    m_odomScan.header.stamp = current_time;
    m_odomScan.header.frame_id = "map";
    m_odomScan.pose.pose.position.x = tf_trans.getX();
    m_odomScan.pose.pose.position.y = tf_trans.getY();
    m_odomScan.pose.pose.position.z = tf_trans.getZ();
    m_odomScan.pose.pose.orientation = odom_quat;

    m_odomScan.child_frame_id = "base_link";
    double temp = 0.01;
    m_odomScan.twist.twist.linear.x = temp;
    m_odomScan.twist.twist.linear.y = temp;
    m_odomScan.twist.twist.angular.z = temp;
    // publish
    pubOdometry.publish(m_odomScan);
    prev_guess = init_guess;
}

int main(int argc, char** argv)
{
    // node name initialization
    ros::init(argc, argv, "scan_matching_localizer_node");

    ros::NodeHandle nh;
    ScanLocalizer localizer(nh);

    ros::spin();

    return 0;
}
