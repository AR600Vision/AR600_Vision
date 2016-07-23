//
// Created by garrus on 21.07.16.
//

#include "Math.h"

//Находит косинус угол между двумя прямыми
double Math::AngleBetweenLines(pcl::Normal a, pcl::Normal b)
{
    if(IsPointNaN(a) || IsPointNaN(b))
        return std::numeric_limits<float>::quiet_NaN();


    return fabs(a.normal_x*b.normal_x + a.normal_y*b.normal_y + a.normal_z*b.normal_z)/
            (sqrt(a.normal_x*a.normal_x + a.normal_y*a.normal_y + a.normal_z*a.normal_z)
             * sqrt(b.normal_x*b.normal_x + b.normal_y*b.normal_y + b.normal_z*b.normal_z));
}


//Возвращает вертикальную нормаль
pcl::Normal Math::Vertical()
{
    return pcl::Normal(0,0,1);
}

//Градусы в радианы, в cmath не нашел
double Math::DegToRad(double deg)
{
    return deg / 180.0 * M_PI;
}

//Радианы в градусы
double Math::RadToDeg(double rad)
{
    return rad / M_PI * 180.0;
}

//Проверяет, что точка - nan
bool Math::IsPointNaN(pcl::Normal point)
{
    return std::isnan(point.normal_x) || std::isnan(point.normal_y) || std::isnan(point.normal_z);
}