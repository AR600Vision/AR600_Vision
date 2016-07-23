/**
 * Полезные функции
 *
 * В каждом проекте есть такой класс
 */

#ifndef AR600_VISION_UTILS_H
#define AR600_VISION_UTILS_H

#include <pcl/point_cloud.h>
//#include <boost/shared_ptr.hpp>

class Utils
{
public:

    /**
     * Заменяет NaN в облаке на адекватные значения
     * pcl vizualizer серьезно глючит, если в облаке
     * c PointXYZRGB есть точки с координатами NaN
     */

    template <class PointT>
    static void ReplaceNaNs(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, float replacement)
    {
        for(int i=0; i<cloud->size(); i++)
        {
            PointT p = cloud->at(i);

            if(std::isnan(p.x))
                cloud->at(i).x = replacement;

            if(std::isnan(p.y))
                cloud->at(i).y = replacement;

            if(std::isnan(p.z))
                cloud->at(i).z = replacement;
        }
    }

};


#endif //AR600_VISION_UTILS_H
