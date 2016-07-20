//
// Created by garrus on 17.07.16.
//

#include "CloudTransforms.h"

CloudTransforms::CloudTransforms()
{
    this->downsample_leaf_size=0.01f;
    this->normal_search_radius=0.05f;

    d_x = 0;
    d_y = 0;
    d_z = 1.5;
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
    ne.setRadiusSearch (normal_search_radius);
    ne.compute (*cloud_normals1);
    return cloud_normals1;
}

/**
 * Поворот облака
 *
 * X - вперед
 * Y - влево
 * Z - вверх*/
pcl::PointCloud<POINT_TYPE>::Ptr CloudTransforms::RotateCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    Eigen::Affine3f transforms = Eigen::Affine3f::Identity();
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(180), Eigen::Vector3f::UnitZ()));
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(-90 - 40), Eigen::Vector3f::UnitY()));
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(90), Eigen::Vector3f::UnitZ()));

    transforms.translation()<<d_x, d_y, d_z;

    pcl::transformPointCloud(*cloud, *transformed_cloud, transforms);
    return transformed_cloud;
}

/**
 * Обрезает облако
 * x,y,z - центр
 * depth, width, height - размеры
 *
 */
pcl::PointCloud<POINT_TYPE>::Ptr CloudTransforms::PassThroughFilter(pcl::PointCloud<POINT_TYPE>::Ptr cloud,
                                                   float x, float y, float z,
                                                   float depth, float width, float height)
{
    Eigen::Vector4f min, max;
    min[0] = x - depth/2;
    max[0] = x + depth/2;
    min[1] = y - width/2;
    max[1] = y + width/2;
    min[2] = z - height/2;
    max[2] = z + height/2;

    pcl::CropBox<POINT_TYPE> crop_box;
    crop_box.setMin(min);
    crop_box.setMax(max);

    //TODO: Убедится, что тут нет утечек памяти
    //pcl::PointCloud<POINT_TYPE>::Ptr cropped = boost::make_shared<pcl::PointCloud<POINT_TYPE> >(*cloud);
    pcl::PointCloud<POINT_TYPE>::Ptr cropped (new pcl::PointCloud<POINT_TYPE>());

    crop_box.setInputCloud(cloud);
    crop_box.filter(*cropped);

    return cropped;

}

//Градусы в радианы, в cmath не нашел
double CloudTransforms::deg_to_rad(float deg)
{
    return deg / 180.0 * M_PI;
}



//Делает из неорганизованного облака организованное
pcl::PointCloud<POINT_TYPE>::Ptr CloudTransforms::MakeOrganizedCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud, float step)
{
    //Ищем минимальные и максимальные координаты в облаке
    float min_x, max_x, min_y, max_y;
    POINT_TYPE point = cloud->at(0);
    min_x = point.x; max_x = point.x;
    min_y = point.y; max_y = point.y;

    for(int i = 1; i<cloud->size(); i++)
    {
        point = cloud->at(i);

        if(point.x < min_x)
            min_x = point.x;
        else if(point.x > max_x)
            max_x = point.x;

        if(point.y < min_y)
            min_y = point.y;
        else if(point.y > max_y)
            max_y = point.y;
    }

    //Вычисляем размеро облака
    float size_x = max_x - min_x;
    float size_y = max_x - min_y;

    int i_size_x = size_x / step;
    int i_size_y = size_y / step;

    //Создаем облако нужного размера
    const float nan = std::numeric_limits<float>::quiet_NaN();
    pcl::PointCloud<POINT_TYPE>::Ptr organized = boost::make_shared<pcl::PointCloud<POINT_TYPE> >(i_size_y, i_size_x, POINT_TYPE(nan, nan, nan));

    //Заполняем
    for(int i = 0; i<cloud->size(); i++)
    {
        point = cloud->at(i);

        int i_coord = (point.x-min_x)/step;
        int j_coord = (point.y-min_y)/step;

        POINT_TYPE p = organized->at(i_coord, j_coord);
        if(p.x!=nan && p.y==nan && p.z==nan)
            if(point.z > p.z)
                organized->at(i_coord,j_coord)= point;

        organized->at(i_coord,j_coord)= point;
    }

    /*pcl::PointCloud<POINT_TYPE>::Ptr organized = boost::make_shared<pcl::PointCloud<POINT_TYPE> >(100, 100);
    for(int i = 0; i<100; i++)
        for(int j = 0; j<100; j++)
            organized->at(i, j)=POINT_TYPE(i/100.0f, j/100.0f, 0);*/


    return organized;

}