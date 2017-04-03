//
// Created by user on 29.03.17.
//

#include <string>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

#include "SimpleDraw/SimpleDraw.h"


using namespace std;
using namespace std::chrono;

const char response_topic[] = "obstacle_detector/obstacle";

SimpleDraw g(500, 500);
int cellToPixel = 4;

void DrawCell(int x, int y, Color color);
void process(ros::ServiceClient rtabmap_client);

int main(int argc, char** argv)
{

    //Инициализация ноды ROS
    ros::init(argc, argv, "ObstacleDetector");
    ros::NodeHandle n;
    ros::Rate rate(0.5);

    //ros::Publisher responsePublisher = n.advertise<>(response_topic, 1, true);
    ros::ServiceClient rtabmap_client = n.serviceClient<nav_msgs::GetMap>("/rtabmap/get_proj_map");

    ROS_INFO("%s", "AR-600/obstacle_detector started");

    g.Clear(Color::White());
    g.Update();

    while (ros::ok())
    {
        process(rtabmap_client);

        if(g.Tick())
            return 0;

        rate.sleep();
    }

    return 0;
}

void process(ros::ServiceClient rtabmap_client)
{
    //Получение карты
    nav_msgs::GetMap proj_map_srv;
    bool success = rtabmap_client.call(proj_map_srv);
    if (!success)
    {
        ROS_ERROR("Failed to get grid_map from rtabmap");
        return;
    }

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
        return;
    }


    tf::Vector3 position = transform.getOrigin();
    tf::Quaternion q = transform.getRotation();
    q.normalize();

    // Get angle
    tf::Matrix3x3 m(q);
    tfScalar yaw, pitch, roll;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("COORD: %lf %lf %lf", position.x(), position.y(), position.z());

    auto map = proj_map_srv.response.map;
    float resolution = map.info.resolution;
    uint32_t width = map.info.width;
    uint32_t height = map.info.height;
    geometry_msgs::Pose origin = map.info.origin;

    /*
     * -1 - неизвестно
     *  0 - нет
     *  100 - препятствие
     */
    vector<int8_t> data = map.data;

    //Рисуем карту
    for(int y = 0; y<height; y++)
    {
        for(int x = 0; x<width; x++)
        {
            int8_t cell = data[y * width + x];

            if(cell==0)
                DrawCell(x,y,Color::White());
            else if(cell==-1)
                DrawCell(x,y,Color(128,128,128));
            else
                DrawCell(x,y,Color::Black());

        }
    }

    //Рисуем положение робота
    int x_pos = (position.x() - origin.position.x) / resolution;
    int y_pos = (position.y() - origin.position.y) / resolution;

    DrawCell(x_pos, y_pos, Color(255,0,0));

    //Рисуем направление
    int x_t = x_pos + 50.0*cos(yaw);
    int y_t = y_pos + 50.0*sin(yaw);

    g.DrawLine(Color(255,0,0), x_pos*cellToPixel, y_pos*cellToPixel, x_t*cellToPixel, y_t*cellToPixel);
}

void DrawCell(int x, int y, Color color)
{
    int size = cellToPixel;
    g.FillRect(color, x*size, y*size, size, size);
}