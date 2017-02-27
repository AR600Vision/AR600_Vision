/**
 * Нода ROS, отвечающий за задачу определения поверхности в точке шага робота
 *
 * Подробное описание реализуемых задач см. в класее StepsController
 *
 *  Принимаемые топики:
 *  - Облако точек с камеры глубины (/depth/points) - PointCloud2
 *  - tf
 *  - Команды на постановку ног
 *
 *  Запрос:
 *    - Xs, Ys, [Zs] - желаемые координаты шага
 *
 *  Ответ:
 *    - Xs, Ys, Zs   - оптимальные координаты шага
 *    - CanStep      - можно ли вообще сделать шаг
 */

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

//TODO: Это  далеко не лучший способ подключить файл, наверняка Catking_Inlcude_Dirs могут лучше, но не работает
#include "../../../../devel/include/ar600_vision/StepResponse.h"

// PCL specific includes
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>

#include <steps_controller/StepsController/StepsController.h>
#include <steps_controller/StepsController/StepRequest.h>

using namespace StepsController;

const char point_cloud_topic[] = "/rtabmap/cloud_map";
const char request_topic[]     = "steps_controller/step_request";
const char response_topic[]    = "steps_controller/response_topic";

StepsController::StepsController* steps_controller;
ros::Publisher responsePublisher;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Обработчики топиков
void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input);
void request_callback(const geometry_msgs::PoseStamped);
bool GetParams(ros::NodeHandle & nh, StepsParams & params);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    //Инициализация ноды ROS
    ros::init(argc, argv, "StepsController");

    ros::NodeHandle n;

    ROS_INFO("%s", "AR-600/steps_controller started");

    //Получение параметров и создание объекта расчета
    StepsParams params;
    if(!GetParams(n, params))
    {
        ROS_ERROR("Unable to read all required parameters");
        ROS_ERROR("AR-600/steps_controller stopping...");
        return -1;
    }

    steps_controller = new StepsController::StepsController(params);

    //Топики
    ros::Subscriber cloudSubscriber = n.subscribe(point_cloud_topic, 1, pointcloud_callback);
    ros::Subscriber requestSubscriber = n.subscribe(request_topic, 1, request_callback);
    responsePublisher = n.advertise<ar600_vision::StepResponse>(response_topic, 1, true);

    while (!steps_controller->wasStopped ())
    {
        steps_controller->spinOnce (100);
        ros::spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }


    delete steps_controller;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Принимает облако точек
void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Преобразование в PCL
    pcl::PCLPointCloud2 pointCloud2;
    pcl_conversions::toPCL(*input, pointCloud2);

    steps_controller->UpdateFrame(boost::make_shared<pcl::PCLPointCloud2>(pointCloud2));
}

void request_callback(const geometry_msgs::PoseStamped point)
{
    float x = point.pose.position.x;
    float y = point.pose.position.y;

    ROS_INFO("Request: StepX=%f StepY=%f",x, y);

    StepsController::StepControllerRequest request;
    request.StepX = x;
    request.StepY = y;

    auto result = steps_controller->CalculateStep(request);

    //Публикуем топик
    ar600_vision::StepResponse response;
    response.CanStep = result.CanStep;
    response.Pose.position.x = result.StepX;
    response.Pose.position.y = result.StepY;
    response.Pose.position.z = result.StepZ;
    responsePublisher.publish(response);
}




//Получает параметры из RosParam
bool GetParams(ros::NodeHandle & nh, StepsParams & params)
{
    bool isOk = true;
    ROS_INFO("Namespace: %s", nh.getNamespace().c_str());

    isOk &= nh.getParam("DownsampleLeafSize", params.DownsampleLeafSize);
    isOk &= nh.getParam("NormalSearchRadius", params.NormalSearchRadius);
    isOk &= nh.getParam("FootX", params.FootX);
    isOk &= nh.getParam("FootY", params.FootY);
    isOk &= nh.getParam("SearchX", params.SearchX);
    isOk &= nh.getParam("SearchY", params.SearchY);
    isOk &= nh.getParam("SearchZ", params.SearchZ);

    return isOk;
}
