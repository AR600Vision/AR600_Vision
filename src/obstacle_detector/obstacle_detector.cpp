//
// Created by user on 29.03.17.
//

#include <string>
#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

using namespace std::chrono;

const char response_topic[] = "obstacle_detector/obstacle";


int main(int argc, char** argv)
{
    //Инициализация ноды ROS
    ros::init(argc, argv, "ObstacleDetector");
    ros::NodeHandle n;
    ros::Rate rate(0.5);

    //ros::Publisher responsePublisher = n.advertise<>(response_topic, 1, true);
    ros::ServiceClient rtabmap_client = n.serviceClient<nav_msgs::GetMap>("/rtabmap/get_proj_map");

    ROS_INFO("%s", "AR-600/obstacle_detector started");

    auto start = steady_clock::now();

    while (ros::ok())
    {
        //Получение карты
        start = steady_clock::now();
        nav_msgs::GetMap proj_map_srv;
        bool success = rtabmap_client.call(proj_map_srv);
        if (!success)
        {
            ROS_ERROR("Failed to get grid_map from rtabmap");
        }

        duration<double> time_span = duration_cast<duration<double>>(steady_clock::now() - start);
        double seconds = time_span.count();

        //ROS_INFO("Duration: %lf", seconds);

        //Получение позиции
        tf::TransformListener listener;
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/odom", "/camera_link", ros::Time(0), ros::Duration(3));
            listener.lookupTransform("/odom", "/camera_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }


        tf::Vector3 position = transform.getOrigin();
        tf::Quaternion q = transform.getRotation();
        q.normalize();

        // Get angle
        tf::Matrix3x3 m(q);
        //m.getRPY(buffer[3], buffer[4], buffer[5]);
        ROS_INFO("COORD: %lf %lf %lf", position.x(), position.y(), position.z());

        rate.sleep();
    }

    return 0;
}