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


#include <cmath>

// PCL specific includes
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>

const char point_cloud_topic[] = "camera/depth/points";
const char point_cloud_name[] = "cloud";
const float downsample_leaf_size =  0.05;

#define POINT_TYPE pcl::PointXYZ

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;                            //Визуализатор
pcl::VoxelGrid<pcl::PCLPointCloud2> sor;                                                //Для уменьшения плотности точек
pcl::NormalEstimation<POINT_TYPE, pcl::Normal> ne;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boost::shared_ptr<pcl::visualization::PCLVisualizer> create_viz();                          //Создание вьювера
void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input);                    //Принимает облако точек

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    //Инициализация ноды ROS
    ros::init(argc, argv, "StepsController");
    ros::NodeHandle n;

    ROS_INFO("%s", "AR-600 steps controller started");

    //Создаем визуализатор
    viewer = create_viz();

    //Подписываемся на топик с облаком точек
    ros::Subscriber sub = n.subscribe(point_cloud_topic ,1, pointcloud_callback);

    //ros::spin ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        ros::spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }


    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Создание вьювера
boost::shared_ptr<pcl::visualization::PCLVisualizer> create_viz()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    return viewer;
}

//Принимает облако точек
void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    if(viewer== NULL)
    {
        ROS_ERROR("%s","Viewer isn't initialized");
        return;
    }

    //Преобразование в PCL
    /*pcl::PCLPointCloud2 pointCloud2;
    pcl_conversions::toPCL(*input, pointCloud2);

    //Уменьшение размера
    pcl::PCLPointCloud2::Ptr downsampled_cloud = downsample_cloud(boost::make_shared<pcl::PCLPointCloud2>(pointCloud2));

    //Преобразование в pcl::PointCloud<T>
    pcl::PointCloud<POINT_TYPE>::Ptr cloud(new pcl::PointCloud<POINT_TYPE>);
    pcl::fromPCLPointCloud2(*downsampled_cloud,*cloud);

    //Поворот
    pcl::PointCloud<POINT_TYPE>::Ptr transformed_cloud = rotate_cloud(cloud);

    //Расчет нормалей
    pcl::PointCloud<pcl::Normal>::Ptr normals = calulate_normals(transformed_cloud);

    //Обновляем облако в визуализаторе
    viewer->removeAllPointClouds(0);
    viewer->addPointCloud(transformed_cloud,point_cloud_name);
    viewer->addPointCloudNormals<POINT_TYPE, pcl::Normal>(transformed_cloud, normals, 1, 0.015, "normals", 0);*/
    
}
