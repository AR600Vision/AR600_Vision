//
// Created by garrus on 17.07.16.
//

#include "StepsController.h"

const std::string StepsController::point_cloud_name = "point_cloud";

StepsController::StepsController()
{
    viewer = setup_vizualizer();

}

//Создает визуализатор
boost::shared_ptr<pcl::visualization::PCLVisualizer> StepsController::setup_vizualizer()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    //viewer->initCameraParameters ();
    viewer->setCameraPosition(0,0,0,1,0,0,0,0,1,0);

    return viewer;
}


/**
 * Принимает очередное изменения облака точек
 * и обрабатывает его
 */
void StepsController::UpdateFrame(pcl::PCLPointCloud2::Ptr pointCloud2)
{
    //Уменьшение размера
    pcl::PCLPointCloud2::Ptr downsampled_cloud = cloud_transforms.DownsampleCloud(pointCloud2);

    //Преобразование в pcl::PointCloud<T>
    pcl::PointCloud<POINT_TYPE>::Ptr cloud(new pcl::PointCloud<POINT_TYPE>);
    pcl::fromPCLPointCloud2(*downsampled_cloud,*cloud);

    //Поворот
    pcl::PointCloud<POINT_TYPE>::Ptr transformed_cloud = cloud_transforms.RotateCloud(cloud);

    //Расчет нормалей
    pcl::PointCloud<pcl::Normal>::Ptr normals = cloud_transforms.CalulateNormals(transformed_cloud);

    //Обновляем облако в визуализаторе
    viewer->removeAllPointClouds(0);
    viewer->addPointCloud(transformed_cloud,point_cloud_name.c_str());
    viewer->addPointCloudNormals<POINT_TYPE, pcl::Normal>(transformed_cloud, normals, 1, 0.015, "normals", 0);
}

//////////////////////////////// ОБЕРКТКИ НАД VIEWER ///////////////////////////////////////////////////////////////////

bool StepsController::wasStopped () const
{
    return viewer->wasStopped();
}

void StepsController::spinOnce (int time, bool force_redraw)
{
    viewer->spinOnce(time, force_redraw);
}