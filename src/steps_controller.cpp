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
pcl::PCLPointCloud2::Ptr downsample_cloud(pcl::PCLPointCloud2::Ptr cloud);                  //Уменьшает количество точек в облаке
pcl::PointCloud<pcl::Normal>::Ptr calulate_normals(pcl::PointCloud<POINT_TYPE>::Ptr cloud); //Расчет нормалей
pcl::PointCloud<POINT_TYPE>::Ptr rotate_cloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud);      //Поворачивает облако

double deg_to_rad(float deg);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

    //Преобразование в PCL
    pcl::PCLPointCloud2 pointCloud2;
    pcl_conversions::toPCL(*input, pointCloud2);

    //Уменьшение размера
    pcl::PCLPointCloud2::Ptr downsampled_cloud = downsample_cloud(boost::make_shared<pcl::PCLPointCloud2>(pointCloud2));

    //Преобразование в pcl::PointCloud<T>
    pcl::PointCloud<POINT_TYPE>::Ptr cloud(new pcl::PointCloud<POINT_TYPE>);
    pcl::fromPCLPointCloud2(*downsampled_cloud,*cloud);

    //Поворот
    pcl::PointCloud<POINT_TYPE>::Ptr transformed_cloud = rotate_cloud(cloud);

    //Расчет нормалей
    //pcl::PointCloud<pcl::Normal>::Ptr normals = calulate_normals(transformed_cloud);

    //Обновляем облако в визуализаторе
    viewer->removeAllPointClouds(0);
    viewer->addPointCloud(transformed_cloud,point_cloud_name);
    //viewer->addPointCloudNormals<POINT_TYPE, pcl::Normal>(transformed_cloud, normals, 1, 0.015, "normals", 0);


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

pcl::PointCloud<pcl::Normal>::Ptr calulate_normals(pcl::PointCloud<POINT_TYPE>::Ptr cloud)
{
    ne.setInputCloud (cloud);
    pcl::search::KdTree<POINT_TYPE>::Ptr tree (new pcl::search::KdTree<POINT_TYPE> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.05);
    ne.compute (*cloud_normals1);

    return cloud_normals1;
}

/**
 * Вращает облако  и приводит его к нормальному положению:
 *   X - вперед
 *   Y - влево
 *   Z - ввврх
 *
 * ВНИМАНИЕ
 * только для непосредственных данных с кинекта, данные
 * RTABMap и других библиотек имеют другую ориентацию
 */
pcl::PointCloud<POINT_TYPE>::Ptr rotate_cloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    Eigen::Affine3f transforms = Eigen::Affine3f::Identity();
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(180), Eigen::Vector3f::UnitZ()));
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(-90), Eigen::Vector3f::UnitY()));
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(90), Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *transformed_cloud, transforms);

    return transformed_cloud;

}

//Преобразует градусы в радианы
double deg_to_rad(float deg)
{
    return deg / 180.0 * M_PI;
}