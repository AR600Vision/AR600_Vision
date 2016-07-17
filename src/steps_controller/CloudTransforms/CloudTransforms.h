/**
 * Этот класс реализует ряд операций над входным облаком,
 * чтобы сделать его пригодным для дальнейшего использования
 *
 *  - Downsampole (уменьшение количества точек)
 *  - Расчет нормалей
 *  - Вырезание участка
 *  - Поворот облака
 */

#ifndef AR600_VISION_CLOUDTRANSFORMS_H
#define AR600_VISION_CLOUDTRANSFORMS_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>

#include "settings.h"

class CloudTransforms
{
public:
    CloudTransforms();
    CloudTransforms(float downsample_leaf_size, float normal_search_radius);

    pcl::PCLPointCloud2::Ptr DownsampleCloud(pcl::PCLPointCloud2::Ptr cloud);                  //Уменьшает количество точек в облаке
    pcl::PointCloud<pcl::Normal>::Ptr CalulateNormals(pcl::PointCloud<POINT_TYPE>::Ptr cloud); //Расчет нормалей
    pcl::PointCloud<POINT_TYPE>::Ptr RotateCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud);      //Поворачивает облако

private:

    float downsample_leaf_size;
    float normal_search_radius;

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;                                                    //Для уменьшения плотности точек
    pcl::NormalEstimation<POINT_TYPE, pcl::Normal> ne;                                          //Для нахождения нормалей

    double deg_to_rad(float deg);                                                               //Градусы в радианы, в cmath не нашел

};


#endif //AR600_VISION_CLOUDTRANSFORMS_H
