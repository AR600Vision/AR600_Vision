//
// Created by garrus on 17.07.16.
//

#include "CloudTransforms.h"

CloudTransforms::CloudTransforms()
{
    /*this->downsample_leaf_size=0.01f;
    this->normal_search_radius=0.05f;

    d_x = 0;
    d_y = 0;
    d_z = 1.5;*/
}


//Уменьшает количество точек в облаке
pcl::PCLPointCloud2::Ptr CloudTransforms::DownsampleCloud(pcl::PCLPointCloud2::Ptr cloud, float downsample_leaf_size)
{
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    sor.setInputCloud(cloud);
    sor.setLeafSize(downsample_leaf_size, downsample_leaf_size, downsample_leaf_size);
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}

//Расчет нормалей
pcl::PointCloud<pcl::Normal>::Ptr CloudTransforms::CalulateNormals(pcl::PointCloud<POINT_TYPE>::Ptr cloud, float normal_search_radius)
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
pcl::PointCloud<POINT_TYPE>::Ptr CloudTransforms::TransformCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud, pcl::PointXYZ rotate, pcl::PointXYZ translate)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    Eigen::Affine3f transforms = Eigen::Affine3f::Identity();

    //TODO: Может быть для корректного вращения надо вращать после поворота для приведения осей
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(180 + rotate.z), Eigen::Vector3f::UnitZ()));
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(-90 + rotate.y), Eigen::Vector3f::UnitY()));
    transforms.rotate(Eigen::AngleAxis<float>(deg_to_rad(90 + rotate.x), Eigen::Vector3f::UnitZ()));

    transforms.translation()<<translate.x, translate.y, translate.z;

    pcl::transformPointCloud(*cloud, *transformed_cloud, transforms);
    return transformed_cloud;
}

/**
 * Обрезает облако
 * x,y,z - центр
 * depth, width, height - размеры
 *
 */
pcl::PointCloud<POINT_TYPE>::Ptr CloudTransforms::BoxFilter(pcl::PointCloud<POINT_TYPE>::Ptr cloud,
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

bool CloudTransforms::is_nan(POINT_TYPE point)
{
    return std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
}

//Делает из неорганизованного облака организованное
void CloudTransforms::MakeOrganizedCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud,
                                         pcl::PointCloud<pcl::Normal>::Ptr normals,
                                         float step,
                                         pcl::PointCloud<POINT_TYPE>::Ptr& organized,
                                         pcl::PointCloud<pcl::Normal>::Ptr& organized_normal)
{
    if(cloud->size()!=normals->size())
    {
        throw ("The number of points differs from the number of normals!");
    }

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
    organized = boost::make_shared<pcl::PointCloud<POINT_TYPE> >(i_size_y, i_size_x, POINT_TYPE(nan, nan, nan));
    organized_normal = boost::make_shared<pcl::PointCloud<pcl::Normal> >(i_size_y, i_size_x);

    //Заполняем
    for(int i = 0; i<cloud->size(); i++)
    {
        point = cloud->at(i);
        pcl::Normal normal = normals->at(i);

        int i_coord = (point.x-min_x)/step;
        int j_coord = (point.y-min_y)/step;

        POINT_TYPE p = organized->at(i_coord, j_coord);

        if(!is_nan(p))
        {
            if (point.z > p.z) {
                organized->at(i_coord, j_coord) = point;
                organized_normal->at(i_coord, j_coord) = normal;
            }
        }
        else
        {
            organized->at(i_coord, j_coord) = point;
            organized_normal->at(i_coord, j_coord) = normal;
        }
    }

}