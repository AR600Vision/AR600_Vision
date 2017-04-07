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

    Vector2i a, b, c, d, e, f;
};

//Препятствие
struct Obstacle
{
    float Dist;
    Vector2i Coord;

    bool IsObstacle()
    {
        return !isinf(Dist);
    }
};

const char response_topic[] = "obstacle_detector/obstacle";

SimpleDraw g(500, 500);
int cellToPixel = 4;

//Карта
nav_msgs::OccupancyGrid proj_map;
std::mutex map_mutex;

//Функция обработки
void process(ros::ServiceClient rtabmap_client, nav_msgs::OccupancyGrid map);

//Получение карты
void getProjMap(ros::ServiceClient rtabmap_client);
void getProjMapThreead(ros::ServiceClient rtabmap_client);

//Функции поиска препятствий
SearchAreaCoords GetSearchArea(float yaw, Vector2i robotPos, int length, int width);
Obstacle SearchInArea(Vector2i pos, SearchAreaCoords area, nav_msgs::OccupancyGrid map);
Obstacle SearchInTriangle(Vector2i pos, Vector2i t0, Vector2i t1, Vector2i t2, nav_msgs::OccupancyGrid map);
bool CheckCell(int x, int y, nav_msgs::OccupancyGrid map);

//Функции рисования
void DrawLine(Vector2i a, Vector2i b, Color c);
void DrawMap(nav_msgs::OccupancyGrid map);
void DrawCell(int x, int y, Color color);

Vector2i v2(int x, int y)
{
    Vector2i v;
    v << x, y;
    return v;
}

int main(int argc, char** argv)
{

    //Инициализация ноды ROS
    ros::init(argc, argv, "ObstacleDetector");
    ros::NodeHandle n;
    ros::Rate rate(0.3);

    //ros::Publisher responsePublisher = n.advertise<>(response_topic, 1, true);
    ros::ServiceClient rtabmap_client = n.serviceClient<nav_msgs::GetMap>("/rtabmap/get_proj_map");

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


    float resolution = map.info.resolution;
    geometry_msgs::Pose origin = map.info.origin;

    DrawMap(map);

    int width = 1 / resolution;
    int length = 1.5 / resolution;


    //Получение угла поворота и координат
    tf::Quaternion q = transform.getRotation();
    q.normalize();

    // Get angle
    tf::Matrix3x3 m(q);
    tfScalar yaw, pitch, roll;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO("Pos: (%lf %lf), Yaw: %lf", transform.getOrigin().x(), transform.getOrigin().y(), yaw);


    Vector2i pos;
    pos << (transform.getOrigin().x() - origin.position.x) / resolution,
         (transform.getOrigin().y() - origin.position.y) / resolution;

    //Получение области, в которой искать препятствия
    auto area = GetSearchArea(yaw, pos, length, width);

    //Поиск препятствия
    auto obstacle = SearchInArea(pos, area, map);
    if(obstacle.IsObstacle())
    {
        g.DrawLine(Color(0, 0, 255),
                   pos(0) * cellToPixel, pos(1) * cellToPixel,
                   obstacle.Coord(0) * cellToPixel, obstacle.Coord(1) * cellToPixel);
    }


    isRunning = false;
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

//Возвращет координаты вершин области, в которой надо искать препятствия
SearchAreaCoords GetSearchArea(float yaw, Vector2i a, int length, int width)
{

    //Положение цели (B)
    Vector2i b;
    b << a(0) + length*cos(yaw),
            a(1) + length*sin(yaw);


    Vector2i ab = b - a;
    float v_l = sqrt(ab(0)*ab(0) + ab(1)*ab(1));

    Vector2i c;
    c << a(0) + ab(1) / v_l * width/2,
            a(1) - ab(0) / v_l * width/2;

    Vector2i d;
    d << a(0) - ab(1) / v_l * width/2,
            a(1) + ab(0) / v_l * width/2;

    Vector2i e = c + ab;

    Vector2i f = d + ab;

    return SearchAreaCoords {a, b, c, d, e, f};
}


//Ищет препятствия в прямогуольной области (одновременно раскрашивает)
Obstacle SearchInArea(Vector2i pos, SearchAreaCoords area, nav_msgs::OccupancyGrid map)
{
    auto o1 = SearchInTriangle(pos, area.c, area.d, area.e, map);
    auto o2 = SearchInTriangle(pos, area.d, area.e, area.f, map);

    return o1.Dist < o2.Dist ? o1 : o2;
}

//Ищет препятстhabr.ru/post/248159/
Obstacle SearchInTriangle(Vector2i pos, Vector2i t0, Vector2i t1, Vector2i t2, nav_msgs::OccupancyGrid map)
{
    Obstacle obstacle;
    obstacle.Dist = std::numeric_limits<float>::infinity();

    if(t0(1)>t1(1)) std::swap(t0, t1);
    if(t0(1)>t2(1)) std::swap(t0, t2);
    if(t1(1)>t2(1)) std::swap(t1, t2);

    int t0t2_height = t2(1) - t0(1);
    int t0t2_width = t2(0) - t0(0);

    int t0t1_height = t1(1) - t0(1);
    int t0t1_width = t1(0) - t0(0);

    int t1t2_height = t2(1) - t1(1);
    int t1t2_width = t2(0) - t1(0);

    //Можно потом сразу возвращать, если нашли препятствие,
    //но сейчас пройдем все, чтобы визуализировть
    bool isEmpty = true;

    for (int y = t0(1); y < t2(1); y++)
    {
        float t0t2_gain = (y - t0(1)) / (float)t0t2_height;
        int x_start = t0(0) + t0t2_width * t0t2_gain + 0.5;
        int x_end;

        //Нижняя половина
        if(y<t1(1) && t0t1_height != 0)
        {
            float t0t1_gain = (y - t0(1)) / (float) t0t1_height;
            x_end = t0(0) + t0t1_width * t0t1_gain + 0.5;
        }

        //Верхняя половина
        if(y >= t1(1) && t1t2_height != 0)
        {
            float t1t2_gain = (y - t1(1)) / (float)t1t2_height;
            x_end = t1(0) + t1t2_width * t1t2_gain + 0.5;
        }

        if(x_start > x_end) std::swap(x_start, x_end);

        for(int x = x_start; x<x_end; x++)
        {
            if(x<0 || x>=map.info.width || y<0 || y>=map.info.height)
                continue;

            bool emptyCell = CheckCell(x, y, map);

            if(!emptyCell)
            {

                float dist = sqrt(pow((double) (x - pos(0)), 2.0f) + pow(float(y - pos(1)), 2.0f));

                if (dist < obstacle.Dist)
                {
                    obstacle.Dist = dist;
                    obstacle.Coord(0) = x;
                    obstacle.Coord(1) = y;
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
