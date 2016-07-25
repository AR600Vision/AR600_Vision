//
// Created by garrus on 24.07.16.
//

#ifndef AR600_VISION_FOOTTARGETFUNCTOR_H
#define AR600_VISION_FOOTTARGETFUNCTOR_H

#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <steps_controller/StepsController/StepsParams.h>
#include <steps_controller/Utils/DataAccessFunctor.h>
#include <steps_controller/Math/Math.h>


/**
 * Функктор целевой функции оптимизации
 * точки наступания
 *
 * Чтобы использовать его как простую функцию, но
 * он внутри себя производит все расчеты
 */
class FootTargetFunctor
{
public:

    /**
     * \param[in] normal_angles - углы откланения нормалей от вертикали
     * \param[in] cloud - облако точек
     * \param[in] step_params - параметры расчет шага (размеры стопы, точка наступания итп)
     * \param[in] step - шаг между точками в организованном облаке
     */
    FootTargetFunctor(boost::shared_ptr<float[]> normal_angles, pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud, StepsParams step_params, float step);

    /**
     * Расчет значения целевой функции
     *
     * Целевая функция зависиот от ряда параметров
     *  - среднее  высоты и откланения нормалей от вертикали
     *  - СКО высоты и откланения нормалей от вертикали
     *  - откланение от запланированной точки наступания (?)
     *
     *  \param[in] x - x-координата центра стопы относительно центра области поиска
     *  \param[in] y - y-координата центра стопы относительно центра области поиска
     */
    float operator()(float x, float y) const;

private:
    boost::shared_ptr<float[]> normal_angles;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    StepsParams step_params;
    float step;

    ArrayAccessFunctor angle_func;
    PointCloudAccessFunctor<pcl::PointXYZRGB> cloud_func;

    void overflow_check(int value, int min, int max, std::string name) const;
};

#endif //AR600_VISION_FOOTTARGETFUNCTOR_H
