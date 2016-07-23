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
    transforms.rotate(Eigen::AngleAxis<float>(Math::DegToRad(180 + rotate.z), Eigen::Vector3f::UnitZ()));
    transforms.rotate(Eigen::AngleAxis<float>(Math::DegToRad(-90 + rotate.y), Eigen::Vector3f::UnitY()));
    transforms.rotate(Eigen::AngleAxis<float>(Math::DegToRad(90 + rotate.x), Eigen::Vector3f::UnitZ()));

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


//Делает из неорганизованного облака организованное
void CloudTransforms::MakeOrganizedCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud,
                                         pcl::PointCloud<pcl::Normal>::Ptr normals,
                                         float step,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& organized,
                                         pcl::PointCloud<pcl::Normal>::Ptr& organized_normal)
{

    if(cloud->size()!=normals->size())
    {
        throw ("The number of points differs from the number of normals!");
    }

    //Нахождение мин. и макс. точек
    float x_min, x_max, y_min, y_max, z_min;

    POINT_TYPE first_point = cloud->at(0);
    x_min = x_max = first_point.x;
    y_min = y_max = first_point.y;
    z_min = first_point.z;

    for(int i = 0; i<cloud->size(); i++)
    {
        POINT_TYPE point = cloud->at(i);
        if(point.x>x_max)
            x_max = point.x;
        if(point.x<x_min)
            x_min = point.x;


        if(point.y>y_max)
            y_max=point.y;
        if(point.y<y_min)
            y_min = point.y;

        if(point.z<z_min)
            z_min = point.z;
    }

    //Определение размера облака
    float cloud_width = x_max - x_min;
    float cloud_height = y_max - y_min;

    int width = cloud_width / step +1;
    int height = cloud_height / step +1;

    float nan = std::numeric_limits<float>::quiet_NaN();
    pcl::PointXYZRGB null_point;
    null_point.x = nan;
    null_point.y = nan;
    null_point.z = z_min;


    organized = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(width, height, null_point);
    organized_normal = boost::make_shared<pcl::PointCloud<pcl::Normal> >(width, height);


    //Заносим точки в облако
    for(int i = 0; i<cloud->size(); i++)
    {
        POINT_TYPE point  = cloud->at(i);
        pcl::Normal normal = normals->at(i);

        //Вычисление индексов
        int column = (point.x - x_min)/step;
        int row = (point.y - y_min)/step;

        if(column<0 || column>=width)
        {
            std::cout<<"Overflow column = "<<column<<std::endl;
            continue;
        }

        if(row<0 || row>=height)
        {
            std::cout<<"Overflow row = "<<row<<std::endl;
            continue;
        }


        pcl::PointXYZRGB current_point  = organized->at(column, row);

        /* В облаке по одним (x;y) могут быть несколько точек
         * с разным z.
         * Надо взять самую верхнюю.
         */
        if(point.z>current_point.z)
        {
            organized->at(column, row).x = point.x;
            organized->at(column, row).y = point.y;
            organized->at(column, row).z = point.z;

            organized_normal->at(column, row) = normal;
        }
    }

}
