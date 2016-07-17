//
// Created by garrus on 17.07.16.
//

#include "CloudTransforms.h"

CloudTransforms::CloudTransforms()
{
    this->downsample_leaf_size=0.05f;
    this->normal_search_radius=0.05f;
}

CloudTransforms::CloudTransforms(float downsample_leaf_size, float normal_search_radius)
{
    this->downsample_leaf_size = downsample_leaf_size;
    this->normal_search_radius = normal_search_radius;
}

//Уменьшает количество точек в облаке
pcl::PCLPointCloud2::Ptr CloudTransforms::DownsampleCloud(pcl::PCLPointCloud2::Ptr cloud)
{
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    sor.setInputCloud(cloud);
    sor.setLeafSize(downsample_leaf_size,downsample_leaf_size, downsample_leaf_size);
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}

//Расчет нормалей
pcl::PointCloud<pcl::Normal>::Ptr CloudTransforms::CalulateNormals(pcl::PointCloud<POINT_TYPE>::Ptr cloud)
{
    ne.setInputCloud (cloud);
    pcl::search::KdTree<POINT_TYPE>::Ptr tree (new pcl::search::KdTree<POINT_TYPE> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (downsample_leaf_size);
    ne.compute (*cloud_normals1);
    return cloud_normals1;
}

//Поворот облака
pcl::PointCloud<POINT_TYPE>::Ptr CloudTransforms::RotateCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    Eigen::Affine3f transforms = Eigen::Affine3f::Identity();
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(180), Eigen::Vector3f::UnitZ()));
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(-90), Eigen::Vector3f::UnitY()));
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(90), Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *transformed_cloud, transforms);
    return transformed_cloud;
}

//Градусы в радианы, в cmath не нашел
double CloudTransforms::deg_to_rad(float deg)
{
    return deg / 180.0 * M_PI;
}