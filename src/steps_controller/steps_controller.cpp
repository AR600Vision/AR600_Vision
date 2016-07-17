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

// PCL specific includes
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>

#include <steps_controller/StepsController/StepsController.h>

const char point_cloud_topic[] = "camera/depth/points";
const char point_cloud_name[] = "cloud";
const float downsample_leaf_size =  0.05;

#define POINT_TYPE pcl::PointXYZ

pcl::VoxelGrid<pcl::PCLPointCloud2> sor;                                                   //Для уменьшения плотности точек
pcl::NormalEstimation<POINT_TYPE, pcl::Normal> ne;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input);                    //Принимает облако точек

StepsController steps_controller;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    //Инициализация ноды ROS
    ros::init(argc, argv, "StepsController");
    ros::NodeHandle n;

    ROS_INFO("%s", "AR-600 steps controller started");

    //Создаем визуализатор
    //viewer = create_viz();

    //Подписываемся на топик с облаком точек
    ros::Subscriber sub = n.subscribe(point_cloud_topic ,1, pointcloud_callback);

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
