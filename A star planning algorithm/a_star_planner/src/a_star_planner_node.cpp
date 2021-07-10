/*
 * a_star_planner_node.cpp
 *
 *  Created on: May 20, 2021
 *      Author: Daegyu Lee
 */


/* NOTE
  1. Do the assignment individually.
  2. Please submit a report on the KLMS. 
  3. Please feel the code where I wrote as "TODO:#".
  4. Play rosbag file named as "Gridmap_and_points.bag" and use occupancy grid map topic.
    - Gridmap_and_points.bag
         - /semantics/costmap_generator/occupancy_grid : nav_msgs/OccupancyGrid.msg
         - /velodyne_points : sensor_msgs/PointCloud2 --> **optional

  5. Use a [AStar.hpp / AStar.cpp] to implement A star algorithm.
  6. Set goal pose using RVIZ 2D Nav Goal tool.
  7. It is not implementing A* directly, but integrating, 
     but we hope you can understand the planning algorithm in the process of implementing the internal functions.
  8. If you have query things, feel free to send me a mail : "lee.dk@kaist.ac.kr" 
  
  ** optional
  In this project, we provided you an occupancy grid map topic.
  As we have learned from the class, we can build an occupancy grid map to consider a robot volume, kinematic constraint.
  If you can generate an occupancy grid map, an extra point will be given.
  If you implemented to build an occupancy grid map, please add an explanation for the code.
 
  *** HINT
  Graph-grid in the A* library is positive value.
  However, our goal point can be both negative and positive.
  As a result, we should adjust the position of the A* graph-grid to be located in the center to integrate A* algorithm.
*/

  
/* REPORT GUIGE
  1. What you have learned
  2. Screen capture with code explaining why you filled the line for TODO#.
  3. Discuss the A* algorithm and how you can utilize it for your project.
*/

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//tf
#include "tf/transform_datatypes.h"
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// ros
#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

#include "a_star_planner/AStar.hpp"


class AstarPlanner
{
    public:
        AstarPlanner(ros::NodeHandle& nh);        
        ~AstarPlanner();
        void CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg);
        void CallbackGoalRviz(const geometry_msgs::PoseStamped& msg);

        ros::NodeHandle nh_;
        
    private:
        ros::Subscriber subOccupancyGrid;
        // ros::Subscriber subLocalPath;
        ros::Subscriber subGoalPose;

        ros::Publisher pubAstarPath; 
        ros::Publisher pubCollisionPoints;

        nav_msgs::Path m_LocalPath;
        geometry_msgs::PoseStamped m_GoalPose;
        bool bNewGoalPose;
};

AstarPlanner::AstarPlanner(ros::NodeHandle& nh) : nh_(nh), bNewGoalPose(false)
{
    subOccupancyGrid = nh_.subscribe("/semantics/costmap_generator/occupancy_grid",1, &AstarPlanner::CallbackOccupancyGrid, this);
    subGoalPose = nh_.subscribe("/move_base_simple/goal",1, &AstarPlanner::CallbackGoalRviz, this);

    pubAstarPath = nh_.advertise<nav_msgs::Path>("/Path/LocalWaypoint/a_star", 1, true);
    pubCollisionPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud2/a_star_collision_points", 1, true);
};

AstarPlanner::~AstarPlanner() 
{    
    ROS_INFO("AstarPlanner destructor.");
}

void AstarPlanner::CallbackGoalRviz(const geometry_msgs::PoseStamped& msg)
{
    m_GoalPose = msg;
    bNewGoalPose = true;
}

void AstarPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
    std::cout<< "start" << std::endl;
    if(!bNewGoalPose)
        return;

    double target_x = m_GoalPose.pose.position.x; //longitudinal
    double target_y = m_GoalPose.pose.position.y; //lateral

    int row, col, rotate_row, rotate_col;
    int grid_x_size = msg.info.width;
    int grid_y_size = msg.info.height;
    

    AStar::Generator generator;
    // Set 2d map size.
    generator.setWorldSize({grid_x_size, grid_y_size});
    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(false);

    generator.clearCollisions();

    pcl::PointCloud<pcl::PointXYZI>::Ptr collision_point_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    // TODO #1 Fill the code
    // 1. set a origin and target in the grid map.
    // 2. Add collision using generator.addCollision()
    // 3. Visualize a collision points using "collisionCloudMsg"

    AStar::Vec2i origin;
    origin.x = 0;
    origin.y = 0;

    AStar::Vec2i target;
    double shiftx = 0.0;
    double shifty = 0.0;
    target.x = target_x;
    target.y = target_y;
    if (target.x < 0){
	origin.x -= target.x;
	shiftx = target.x;
	target.x = 0;
     }
    if (target.y < 0){
	origin.y -= target.y;
	shifty = target.y;
	target.y = 0;
     }


    for (unsigned int width = 0; width < grid_x_size; width++)
    {
        for (unsigned int height = 0; height < grid_y_size; height++)
        {
	    //float sens = msg.data[width + height * msg.info.resolution];
	    int sens = msg.data[width + height * msg.info.width];
	    //std::cout << sens << std::endl;
	    if (sens > 80){
		    std::cout << "obstacle" << std::endl;
		    pcl::PointXYZI pointBuf;
		    pointBuf.x = width * msg.info.resolution + msg.info.resolution / 2 + msg.info.origin.position.x;
		    pointBuf.y = height * msg.info.resolution + msg.info.resolution / 2 + msg.info.origin.position.y;
		    pointBuf.z = 0.0;
		    pointBuf.intensity = 1;
		    collision_point_ptr->push_back(pointBuf);

		    AStar::Vec2i temp;
		    temp = { width, height };
		    generator.addCollision(temp,5);
	     }
        }
    }   
    //visualize the collision points
    sensor_msgs::PointCloud2 collisionCloudMsg;
    pcl::toROSMsg(*collision_point_ptr, collisionCloudMsg);
    collisionCloudMsg.header.frame_id = "base_link";
    collisionCloudMsg.header.stamp = ros::Time::now();
    pubCollisionPoints.publish(collisionCloudMsg);

    // TODO #2: Fill the code
    // 1. Implement A* using generator.findPath({origin_grid_x, origin_grid_y}, {target_grid_x, target_grid_y})
    nav_msgs::Path AStartPathMsg;
    AStartPathMsg.header.stamp = ros::Time();
    AStartPathMsg.header.frame_id = "base_link";
    
    auto path = generator.findPath(origin, target);

    // TODO #3: Fill the code
    // 1. Publish the result using nav_msgs/Path.msg
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.seq = 0;

    for (auto& wp : path)
    {
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "base_link";

        pose_stamped.pose.position.x = wp.x + shiftx;
        pose_stamped.pose.position.y = wp.y + shifty;
        pose_stamped.pose.position.z = 0;

        AStartPathMsg.poses.push_back(pose_stamped);
        pose_stamped.header.seq++;
    }
    
    pubAstarPath.publish(AStartPathMsg);

}

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "a_star_planner");
    // for subscribe
    ros::NodeHandle nh;
    AstarPlanner planner(nh);

    ros::spin();
    return 0;

}
