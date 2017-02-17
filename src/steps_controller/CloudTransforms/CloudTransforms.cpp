//
// Created by garrus on 17.07.16.
//

#include "CloudTransforms.h"

CloudTransforms::CloudTransforms()
{
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
    transforms.rotate(Eigen::AngleAxis<float>(Math::DegToRad(rotate.z), Eigen::Vector3f::UnitZ()));
    transforms.rotate(Eigen::AngleAxis<float>(Math::DegToRad(rotate.y), Eigen::Vector3f::UnitY()));
    transforms.rotate(Eigen::AngleAxis<float>(Math::DegToRad(rotate.x), Eigen::Vector3f::UnitZ()));

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
bool CloudTransforms::MakeOrganizedCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud,
                                         pcl::PointCloud<pcl::Normal>::Ptr normals,
                                         float step,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& organized,
                                         pcl::PointCloud<pcl::Normal>::Ptr& organized_normal,
                                         float search_x, float search_y,
                                         float center_x, float center_y)
{

    int size1 = cloud->size();
    int size2 = normals->size();

    if(cloud->size()!=normals->size())
    {
        throw ("The number of points differs from the number of normals!");
    }

    int size = cloud->size();
    if(cloud->size()==0)
        return false;

    //Нахождение мин. и макс. точек
    float z_min;

    POINT_TYPE first_point = cloud->at(0);
    z_min = first_point.z;

    for(int i = 0; i<cloud->size(); i++)
    {
        POINT_TYPE point = cloud->at(i);

        if(point.z<z_min)
            z_min = point.z;
    }

    //Определение размера облака
    float cloud_width = search_x;
    float cloud_height = search_y;

    int width = cloud_width / step +1;
    int height = cloud_height / step +1;

    float x_min = center_x - search_x/2;
    float y_min = center_y - search_y/2;

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

            //std::cout<<"column: "<<column<<", row: "<<row<<" ("<<point.x<<", "<<point.y<<", "<<point.z<<")\n";

            organized_normal->at(column, row) = normal;
        }

        /*for(int i = 0; i<organized->size(); i++)
        {
            pcl::PointXYZRGB point = organized->at(i);
            std::cout<<point.x << ' '<<point.y<<point.z<<'\n';
        }*/

    }

    return true;

}
