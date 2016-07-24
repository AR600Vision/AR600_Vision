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
#include <steps_controller/Utils/Utils.h>

#include <steps_controller/Utils/DataAccessFunctor.h>

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


    /**
     * Расчет матожидания (среднего)
     * func - объект извлечения данных
     * остальные параметры определяют область в двухмерном массове, в которой надо посчитать
     *
     * ВНИМАНИЕ: это НЕСТРОГИЕ границы [star_*; end_*]
     */
    static float Average(DataAccessFunctor* func, int start_x, int end_x, int start_y, int end_y);

    //Расчт дисперсии
    static float Dispetion(DataAccessFunctor* func, float average, int start_x, int end_x, int start_y, int end_y);
    static float Dispetion(DataAccessFunctor* func,  int start_x, int end_x, int start_y, int end_y);

};


#endif //AR600_VISION_EXTENDEDMATH_H
