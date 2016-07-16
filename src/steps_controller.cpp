/**
 * Нода ROS, отвечающий за задачу определения поверхности в точке шага робота
 *
 * Задачи:
 *  - По заданной координате шага и ряду других параметров, определять
 *    пригодна ли поверхность для осуществления шага
 *  - Поиск пригодной поверхности для шага в заданной окрестности
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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


const char point_cloud_topic[] = "camera/depth/points";
const char point_cloud_name[] = "cloud";
const float downsample_leaf_size =  0.05;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;                            //Визуализатор
pcl::VoxelGrid<pcl::PCLPointCloud2> sor;                                                //Для уменьшения плотности точек

boost::shared_ptr<pcl::visualization::PCLVisualizer> create_viz();                      //Создание вьювера
void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input);                //Принимает облако точек
pcl::PCLPointCloud2::Ptr downsample_cloud(pcl::PCLPointCloud2::Ptr cloud);              //Уменьшает количество точек в облаке

int main(int argc, char** argv)
{
    //Инициализация ноды ROS
    ros::init(argc, argv, "steps_controller");
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

    //Преобразование в pcl::PointCloud<T>
    pcl::PCLPointCloud2 pointCloud2;
    pcl_conversions::toPCL(*input, pointCloud2);

    //Уменьшение размера
    pcl::PCLPointCloud2::Ptr downsampled_cloud = downsample_cloud(boost::make_shared<pcl::PCLPointCloud2>(pointCloud2));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*downsampled_cloud,*cloud);

    //Обновляем облако в визуализаторе
    viewer->removePointCloud();
    viewer->addPointCloud(cloud,point_cloud_name);

    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
}


//Уменьшает количество точек в облаке
pcl::PCLPointCloud2::Ptr downsample_cloud(pcl::PCLPointCloud2::Ptr cloud)
{
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    sor.setInputCloud(cloud);
    sor.setLeafSize(downsample_leaf_size,downsample_leaf_size, downsample_leaf_size);
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}