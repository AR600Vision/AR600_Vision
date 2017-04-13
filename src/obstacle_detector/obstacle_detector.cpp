//
// Created by user on 29.03.17.
//

#include <string>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

#include "SimpleDraw/SimpleDraw.h"
#include "CoordConverter/CoordConverter.h"
#include "../../../../devel/include/ar600_vision/NearestObstacle.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

struct SearchAreaCoords
{
    /*
         (target)
     E ---- B ---- F
     |             |
     |             |
     |             |
     |             |
     |             |
     C ---- A ---- D
         (robot)
     */

    tf::Vector3 a, b, c, d, e, f;
};

//Препятствие
struct Obstacle
{
    float Dist;
    tf::Vector3 Coord;

    bool IsObstacle()
    {
        return !isinf(Dist);
    }
};

///////////////////////////////////////////////////////////////////////////////////////////////////

//Топик,в  который публикуются препятствия
ros::Publisher obstaclePublisher;

SimpleDraw g(500, 500);
int cellToPixel = 4;

//Карта
nav_msgs::OccupancyGrid proj_map;
std::mutex map_mutex;

const float area_width = 0.5;
const float area_height = 1.5;

///////////////////////////////////////////////////////////////////////////////////////////////////

//Функция обработки
void process(ros::ServiceClient rtabmap_client, nav_msgs::OccupancyGrid map);

//Получение карты
void getProjMap(ros::ServiceClient rtabmap_client);
void getProjMapThreead(ros::ServiceClient rtabmap_client);

//Функции поиска препятствий
SearchAreaCoords GetSearchArea(float yaw, tf::Vector3 robotPos, int length, int width);
Obstacle SearchInArea(tf::Vector3 pos, SearchAreaCoords area, nav_msgs::OccupancyGrid map);
Obstacle SearchInTriangle(tf::Vector3 pos, tf::Vector3 t0, tf::Vector3 t1, tf::Vector3 t2, nav_msgs::OccupancyGrid map);
bool CheckCell(int x, int y, nav_msgs::OccupancyGrid map);

//Функции рисования
void DrawLine(Vector2i a, Vector2i b, Color c);
void DrawMap(nav_msgs::OccupancyGrid map);
void DrawCell(int x, int y, Color color);

//Преобразует координаты в индексы на карте
//Vector2i PoseToIndex(tf::Vector3 pos, nav_msgs::OccupancyGrid map);

//Преобразует индексы на карте в нормалные координаты
//Vector2f IndexToPose(int x, int y, nav_msgs::OccupancyGrid map);
///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{

    //Инициализация ноды ROS
    ros::init(argc, argv, "ObstacleDetector");
    ros::NodeHandle n;
    ros::Rate rate(0.3);

    //ros::Publisher responsePublisher = n.advertise<>(response_topic, 1, true);
    ros::ServiceClient rtabmap_client = n.serviceClient<nav_msgs::GetMap>("/rtabmap/get_proj_map");
    obstaclePublisher = n.advertise<ar600_vision::NearestObstacle>("obstacle_detector/obstacle", 1, true);

    ROS_INFO("%s", "AR-600/obstacle_detector started");

    g.Clear(Color::White());
    g.Update();

    getProjMap(rtabmap_client);
    std::thread proj_thread(getProjMapThreead, rtabmap_client);
    proj_thread.detach();

    while (ros::ok())
    {
        //Получение карты
        map_mutex.lock();
        nav_msgs::OccupancyGrid copy_map  = proj_map;
        map_mutex.unlock();

        process(rtabmap_client, copy_map);

        if(g.Tick())
            return 0;

        //rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

void process(ros::ServiceClient rtabmap_client, nav_msgs::OccupancyGrid map)
{
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

    DrawMap(map);


    float resolution = map.info.resolution;

    int width = area_width / resolution;
    int length = area_height / resolution;

    //Получение угла поворота и координат
    tf::Quaternion q = transform.getRotation();
    q.normalize();

    // Get angle
    tf::Matrix3x3 m(q);
    tfScalar yaw, pitch, roll;
    m.getRPY(roll, pitch, yaw);

    //ROS_INFO("Pos: (%lf %lf), Yaw: %lf", transform.getOrigin().x(), transform.getOrigin().y(), yaw);


    //Целочисленные координаты  (индексы в карте)
    tf::Vector3 pose_index = CoordConverter::CoordToIndex(transform.getOrigin(), map);

    //Получение области, в которой искать препятствия
    auto area = GetSearchArea(yaw, pose_index, length, width);

    //Поиск препятствия
    auto obstacle = SearchInArea(pose_index, area, map);
    ar600_vision::NearestObstacle response;

    if(obstacle.IsObstacle())
    {
        g.DrawLine(Color(0, 0, 255),
                   pose_index.x() * cellToPixel, pose_index.y() * cellToPixel,
                   obstacle.Coord.x() * cellToPixel, obstacle.Coord.y() * cellToPixel);




        response.IsObstacle = true;
        response.ObstaclePose.position.x = obstacle.Coord.x();
        response.ObstaclePose.position.y = obstacle.Coord.y();
        ROS_INFO("Obstacle: (%lf, %lf), Dist: %lf", obstacle.Coord.x(), obstacle.Coord.y(), obstacle.Dist);
    }
    else
    {
        response.IsObstacle = false;
        response.ObstaclePose.position.x = 0;
        response.ObstaclePose.position.y = 0;
        ROS_INFO("No obstacle");
    }

    obstaclePublisher.publish(response);
}

//Получение карты
void getProjMap(ros::ServiceClient rtabmap_client)
{
    //Получение карты
    nav_msgs::GetMap proj_map_srv;
    bool success = rtabmap_client.call(proj_map_srv);
    if (!success)
    {
        ROS_ERROR("Failed to get grid_map from rtabmap");
        return;
    }

    map_mutex.lock();
    proj_map = proj_map_srv.response.map;
    map_mutex.unlock();
}

//Поток получениия карты
void getProjMapThreead(ros::ServiceClient rtabmap_client)
{
    while(true)
    {
        getProjMap(rtabmap_client);
    }
}

//////////////////////////// ПОИСК ПРЕПЯТСТВИЙ ////////////////////////////////////////////////////

//Возвращет координаты вершин области, в которой надо искать препятствия
SearchAreaCoords GetSearchArea(float yaw, tf::Vector3 a, int length, int width)
{

    //Положение цели (B)
    tf::Vector3 b(a.x() + length * cos(yaw), a.y() + length * sin(yaw), 0);

    tf::Vector3 ab = b - a;
    float v_l = ab.length(); //sqrt(ab.x()*ab.x() + ab(1)*ab(1));

    tf::Vector3 c (a.x() + ab.y() / v_l * width /2, a.y() + ab.x() / v_l * width / 2, 0);
    tf::Vector3 d (a.x() - ab.y() / v_l * width /2, a.y() + ab.x() / v_l * width / 2, 0);
    tf::Vector3 e = c + ab;
    tf::Vector3 f = d + ab;

    return SearchAreaCoords {a, b, c, d, e, f};
}


//Ищет препятствия в прямогуольной области (одновременно раскрашивает)
Obstacle SearchInArea(tf::Vector3 pos, SearchAreaCoords area, nav_msgs::OccupancyGrid map)
{
    auto o1 = SearchInTriangle(pos, area.c, area.d, area.e, map);
    auto o2 = SearchInTriangle(pos, area.d, area.e, area.f, map);

    return o1.Dist < o2.Dist ? o1 : o2;
}

//Ищет препятстhabr.ru/post/248159/
Obstacle SearchInTriangle(tf::Vector3 pos, tf::Vector3 t0, tf::Vector3 t1, tf::Vector3 t2, nav_msgs::OccupancyGrid map)
{
    Obstacle obstacle;
    obstacle.Dist = std::numeric_limits<float>::infinity();

    if(t0.y()>t1.y()) std::swap(t0, t1);
    if(t0.y()>t2.y()) std::swap(t0, t2);
    if(t1.y()>t2.y()) std::swap(t1, t2);

    int t0t2_height = t2.y() - t0.y();
    int t0t2_width = t2.x() - t0.x();

    int t0t1_height = t1.y() - t0.y();
    int t0t1_width = t1.x() - t0.x();

    int t1t2_height = t2.y() - t1.y();
    int t1t2_width = t2.x() - t1.x();

    //Можно потом сразу возвращать, если нашли препятствие,
    //но сейчас пройдем все, чтобы визуализировть
    bool isEmpty = true;

    for (int y = t0.y(); y < t2.y(); y++)
    {
        float t0t2_gain = (y - t0.y()) / (float)t0t2_height;
        int x_start = t0.x() + t0t2_width * t0t2_gain + 0.5;
        int x_end;

        //Нижняя половина
        if(y<t1.y() && t0t1_height != 0)
        {
            float t0t1_gain = (y - t0.y()) / (float) t0t1_height;
            x_end = t0.x() + t0t1_width * t0t1_gain + 0.5;
        }

        //Верхняя половина
        if(y >= t1.y() && t1t2_height != 0)
        {
            float t1t2_gain = (y - t1.y()) / (float)t1t2_height;
            x_end = t1.x() + t1t2_width * t1t2_gain + 0.5;
        }

        if(x_start > x_end) std::swap(x_start, x_end);

        for(int x = x_start; x<x_end; x++)
        {
            if(x<0 || x>=map.info.width || y<0 || y>=map.info.height)
                continue;

            bool emptyCell = CheckCell(x, y, map);

            if(!emptyCell)
            {
                //Vector2f obstacleCoord = IndexToPose(x, y, map);

                float dist = sqrt(pow((double) (x - pos.x()), 2.0f) + pow(float(y - pos.y()), 2.0f));

                if (dist < obstacle.Dist)
                {
                    obstacle.Dist = dist;
                    obstacle.Coord.setX(x);
                    obstacle.Coord.setY(y);
                }
            }
        }
    }

    return obstacle;
}

//Проверяет наличие препятствия в клетке
bool CheckCell(int x, int y, nav_msgs::OccupancyGrid map)
{
    int8_t cell = map.data[y * map.info.width + x];

    if (cell == 0)
    {
        DrawCell(x, y, Color(0, 255, 0));
        return true;
    }
    else if (cell == -1)
    {
        DrawCell(x, y, Color(255, 255, 0));
        return true;
    }
    else
    {
        DrawCell(x, y, Color(255, 0, 0));
        return false;
    }
}


////////////////////////////// РИСОВАНИЕ //////////////////////////////////////////////////////////

//Линия алгоритмом Брезенхэма
//https://habrahabr.ru/post/248153/
void DrawLine(Vector2i a, Vector2i b, Color c)
{
    int x0 = a(0), y0 = a(1);
    int x1 = b(0), y1 = b(1);

    bool steep = false;
    if (std::abs(x0-x1)<std::abs(y0-y1))
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0>x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int dx = x1-x0;
    int dy = y1-y0;
    int derror2 = std::abs(dy)*2;
    int error2 = 0;
    int y = y0;

    for (int x=x0; x<=x1; x++)
    {
        if (steep)
        {
            //image.set(y, x, color);
            DrawCell(y, x, c);

        }
        else
        {
            //image.set(x, y, color);
            DrawCell(x, y, c);
        }

        error2 += derror2;

        if (error2 > dx)
        {
            y += (y1>y0?1:-1);
            error2 -= dx*2;
        }
    }
}


//Рисует карту
void DrawMap(nav_msgs::OccupancyGrid map)
{
    for(int y = 0; y<map.info.height; y++)
    {
        for(int x = 0; x<map.info.width; x++)
        {
            int8_t cell = map.data[y * map.info.width + x];

            if(cell==0)
                DrawCell(x,y,Color::White());
            else if(cell==-1)
                DrawCell(x,y,Color(128,128,128));
            else
                DrawCell(x,y,Color::Black());

        }
    }
}

//Рисует "пиксель" на карте
void DrawCell(int x, int y, Color color)
{
    int size = cellToPixel;
    g.FillRect(color, x*size, y*size, size, size);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//Преобразует координаты в индексы на карте
/*Vector2i PoseToIndex(tf::Vector3 pos, nav_msgs::OccupancyGrid map)
{
    float resolution = map.info.resolution;
    geometry_msgs::Pose origin = map.info.origin;

    Vector2i pose_i;
    pose_i << (pos.x() - origin.position.x) / resolution,
            (pos.y() - origin.position.y) / resolution;

    return pose_i;
}

//Преобразует индексы на карте в нормалные координаты
Vector2f IndexToPose(int x, int y, nav_msgs::OccupancyGrid map)
{
    float resolution = map.info.resolution;
    geometry_msgs::Pose origin = map.info.origin;

    Vector2f pos;
    pos<<x*resolution + origin.position.x, y*resolution + origin.position.y;

    return pos;
}*/