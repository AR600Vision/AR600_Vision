/**
 * Нода ROS, отвечающий за задачу определения поверхности в точке шага робота
 *
 * Подробное описание реализуемых задач см. в класее StepsController
 *
 *  Принимаемые топики:
 *  - Облако точек с камеры глубины (/depth/points) - PointCloud2
 *  - Команды на постановку ног
 */

#include <ros/ros.h>

//TODO: Это  далеко не лучший способ подключить файл, наверняка Catking_Inlcude_Dirs могут лучше, но не работает
#include "../../../../devel/include/ar600_vision/StepsController.h"

// PCL specific includes
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>

#include <steps_controller/StepsController/StepsController.h>
#include <steps_controller/StepsController/StepRequest.h>

const char point_cloud_topic[] = "camera/depth/points";

StepsController::StepsController steps_controller;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input);                    //Принимает облако точек
bool CanStep(ar600_vision::StepsController::Request &req, ar600_vision::StepsController::Response &res);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    //Инициализация ноды ROS
    ros::init(argc, argv, "StepsController");
    ros::NodeHandle n;

    ROS_INFO("%s", "AR-600 steps controller started");

    //Подписываемся на топик с облаком точек
    ros::Subscriber sub = n.subscribe(point_cloud_topic ,1, pointcloud_callback);

    //Создаем сервис
    ros::ServiceServer steps_controller_service = n.advertiseService("steps_controller",CanStep);

    //ros::spin ();

    while (!steps_controller.wasStopped ())
    {
        steps_controller.spinOnce (100);
        ros::spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }


    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Принимает облако точек
void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Преобразование в PCL
    pcl::PCLPointCloud2 pointCloud2;
    pcl_conversions::toPCL(*input, pointCloud2);

    steps_controller.UpdateFrame(boost::make_shared<pcl::PCLPointCloud2>(pointCloud2));
}


/////////////////////////////////////////  РЕАЛИЗАЦИЯ СЕРВСИА //////////////////////////////////////////////////////////

/**
 * Этот сервис принимает запрос на проверку возможности наступить в точку
 * и возвращает, можно ли наступить в точку и какие параметры шага должны быть
 */
bool CanStep(ar600_vision::StepsController::Request &req, ar600_vision::StepsController::Response &res)
{
    ROS_INFO("Request: StepX=%f StepY=%f",req.StepX, req.StepY);

    StepsController::StepControllerRequest request;
    request.StepX = req.StepX;
    request.StepY = req.StepY;

    steps_controller.CalculateStep(request);

}