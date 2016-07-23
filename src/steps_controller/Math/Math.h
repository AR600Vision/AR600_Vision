/**
 * Здесь содержаться некоторые математические
 * методы, которые используются в других частях
 * проекта
 */

#ifndef AR600_VISION_EXTENDEDMATH_H
#define AR600_VISION_EXTENDEDMATH_H

#include <cmath>
#include <pcl/point_types.h>
#include "settings.h"

class Math
{
public:

    //Находит косинус угол между двумя прямыми
    //Прямые заданы координатами направляющих векторов
    static double AngleBetweenLines(pcl::Normal a, pcl::Normal b);

    //Возвращает вертикальную нормаль
    static pcl::Normal Vertical();

    //Градусы в радианы, в cmath не нашел
    static double DegToRad(double deg);

    //Радианы в градусы
    static double RadToDeg(double rad);

    //Проверяет, что точка - nan
    template <class PointT>
    static bool IsPointNaN(PointT point)
    {
        return std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
    }

    static bool IsPointNaN(pcl::Normal point);
};


#endif //AR600_VISION_EXTENDEDMATH_H
