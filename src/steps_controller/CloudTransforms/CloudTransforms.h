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

#include <limits>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include "settings.h"
#include <steps_controller/Math/Math.h>

class CloudTransforms
{
public:
    CloudTransforms();

    //Уменьшает количество точек в облаке
    pcl::PCLPointCloud2::Ptr DownsampleCloud(pcl::PCLPointCloud2::Ptr cloud, float downsample_leaf_size);

    //Расчет нормалей
    pcl::PointCloud<pcl::Normal>::Ptr CalulateNormals(pcl::PointCloud<POINT_TYPE>::Ptr cloud, float normal_search_radius);

    /**
     * Поворот облака
     *
     * X - вперед
     * Y - влево
     * Z - вверх*/
    pcl::PointCloud<POINT_TYPE>::Ptr TransformCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud, pcl::PointXYZ rotate, pcl::PointXYZ translate);

    /**
     * Обрезает облако
     * x,y,z - центр
     * depth, width, height - размеры
     *
     */
    pcl::PointCloud<POINT_TYPE>::Ptr BoxFilter(pcl::PointCloud<POINT_TYPE>::Ptr cloud,
                                               float x, float y, float z,
                                               float depth, float width, float height);

    /**
     * Делает из неорганизованного облака организованное
     * Так же синхронно с точками сдвигает нормали. Дело в том, что проще
     * организовать уже найденные нормали, чем найти в организованном облаке, это
     * почему-то не работает
     * \param[in]  cloud - исходное облако точек
     * \param[in]  normals - исходное облако нормалей
     * \param[in]  step - требуемый шаг сетки
     * \param[out] organized - новое организованное облако точек
     * \param[out] organized_normals - новоое организованное облако нормалей
     * \param[in]  search_x - размер зоны поиска по X
     * \param[in]  search_y - размер зоны поиска по Y
     * \param[in]  center_x - X центр зоны поиска
     * \param[in]  center_y - Y центр зоны поиска
     */
    static bool MakeOrganizedCloud(pcl::PointCloud<POINT_TYPE>::Ptr cloud,
                                   pcl::PointCloud<pcl::Normal>::Ptr normals,
                                   float step,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& organized,
                                   pcl::PointCloud<pcl::Normal>::Ptr& organized_normal,
                                   float search_x, float search_y,
                                   float center_x, float center_y);

private:

    //TODO: насколько эффективно то, что объекты создаются один раз? Может быть это не сильно помогает и стоит сделать класс статическим
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;                                                    //Для уменьшения плотности точек
    pcl::NormalEstimation<POINT_TYPE, pcl::Normal> ne;                                          //Для нахождения нормалей



};


#endif //AR600_VISION_CLOUDTRANSFORMS_H
