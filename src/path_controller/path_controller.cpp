#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <chrono>

// These things will be used in callback
// and initialized in main
ros::ServiceClient rtabmap_client;
ros::Publisher map_pub;
ros::Publisher map_walls_pub;
ros::Publisher pose_publisher;

// todo make it a class
// todo add checks (if topic is not awailable or other)
// todo add debug info
// todo make some names be arguments (topics names here and other)
// todo it isn't building in cmake (only in clion) it's bag. because in catkin_ws lies old version
// todo make pull request with error description to footstep_planner
// todo correct startpose to be in map range if it isn't (or maybe return default path)
// todo fill proj map
// todo idea change footstep_planner to have status topic and subscribe to that
// todo if here invalid goal, change goal and send again
// todo if no map send map and etc.
// todo also check this http://wiki.ros.org/topic_tools/throttle
// maybe rewrite footstep planner to map_change doesn't trigger replanning
// search until first solution is very fast, but results are ugly
// I think that only footstep_planner node is slow here
// but why then map is publishing with little delay?


// fixme callback is so long to work ~ 0.8 sec always
// fixme but time between callback and and map appearance is much longer
// todo I think problem not in my node, but in planner node
// fixme now i saw callback time (4 seconds)
// todo play with rtabmap cells_grid_size and footstep_planner grid_size
// When goal is published
// Get tf of camera from rtabmap and make from it PoseWithCowarianceStamped
// Get proj_map from rtabmap
// Publish proj_map
// Publish pose as initialpose
void goal_callback(const geometry_msgs::PoseStamped goal)
{
    auto start = std::chrono::high_resolution_clock::now();
    // Getting pose from camera tf
    tf::TransformListener tf_listener;
    tf::StampedTransform camera_tf;

    geometry_msgs::PoseWithCovarianceStamped camera_pose;
    try
    {
        tf_listener.waitForTransform("/map", "/base_link", 
                                     ros::Time(0), ros::Duration(3.0));
        // todo I think odom - where object is now. rtabmap has params to set the requency of tf publishing
        // in demo there are no visual odometry and so odom still at start point
        // maybe we should monitor odometry in other place
        tf_listener.lookupTransform("/map", "/base_link",
                                     ros::Time(0), camera_tf);

        const tf::Vector3 & origin = camera_tf.getOrigin();
        const tf::Quaternion rotation = camera_tf.getRotation();

        camera_pose.header.frame_id = "map";
        camera_pose.pose.pose.position.x = origin.x();
        camera_pose.pose.pose.position.y = origin.y();

        // check here for more information
        // http://answers.ros.org/question/9772/quaternions-orientation-representation/
        camera_pose.pose.pose.orientation.z = rotation.getZ();
        camera_pose.pose.pose.orientation.w = rotation.getW();
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }

    // Getting map from rtabmap service (get_proj_map)
    nav_msgs::GetMap proj_map_srv;
    bool success = rtabmap_client.call(proj_map_srv);
    if (!success)
    {
        ROS_ERROR("Failed to get grid_map from rtabmap");
        return;
    }

    // Publish map
    map_pub.publish(proj_map_srv.response.map);
    // map_walls_pub.publish(proj_map_srv.response.map);

    // publish pose as geometry_msgs/PoseWithCovarianceStamped
    pose_publisher.publish(camera_pose);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    ROS_INFO("Callback time was %lf", elapsed.count());
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "path_controller");
    ros::NodeHandle n;

    rtabmap_client = n.serviceClient<nav_msgs::GetMap>("/rtabmap/get_proj_map");
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    map_walls_pub = n.advertise<nav_msgs::OccupancyGrid>("map_walls", 1, true);
    pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);

    ros::Subscriber goal_sub = n.subscribe("goal", 1, goal_callback);
    ros::spin();

    return 0;
}