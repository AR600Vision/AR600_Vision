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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include "settings.h"

class CloudTransforms
{
public:
    CloudTransforms();
    CloudTransforms(float downsample_leaf_size, float normal_search_radius);

    //Уменьшает количество точек в облаке
    pcl::PCLPointCloud2::Ptr DownsampleCloud(pcl::PCLPointCloud2::Ptr cloud);

    //Расчет нормалей
    pcl::PointCloud<pcl::Normal>::Ptr CalulateNormals(pcl::PointCloud<POINT_TYPE>::Ptr cloud);

    /**
     * Поворот облака
     *
     * X - вперед
     * Y - влево
     * Z - вверх*/
    pcl::PointCloud<POINT_TYPE>::Ptr RotateCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud);

    /**
     * Обрезает облако
     * x,y,z - центр
     * depth, width, height - размеры
     *
     */
    pcl::PointCloud<POINT_TYPE>::Ptr PassThroughFilter(pcl::PointCloud<POINT_TYPE>::Ptr cloud,
        float x, float y, float z,
        float depth, float width, float height);

    /**
     * Делает из неорганизованного облака организованное
     * Так же синхронно с точками сдвигает нормали. Дело в том, что проще
     * организовать уже найденные нормали, чем найти в организованном облаке, это
     * почему-то не работает
     */
    static void MakeOrganizedCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud,
                            pcl::PointCloud<pcl::Normal>::Ptr normals,
                            float step,
                            pcl::PointCloud<POINT_TYPE>::Ptr& organized,
                            pcl::PointCloud<pcl::Normal>::Ptr& organized_normal);

private:

    float downsample_leaf_size;
    float normal_search_radius;

    float d_x, d_y, d_z;

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;                                                    //Для уменьшения плотности точек
    pcl::NormalEstimation<POINT_TYPE, pcl::Normal> ne;                                          //Для нахождения нормалей

    double deg_to_rad(float deg);                                                               //Градусы в радианы, в cmath не нашел
    static bool is_nan(POINT_TYPE point);

};


#endif //AR600_VISION_CLOUDTRANSFORMS_H
