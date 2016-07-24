#ifndef AR600_VISION_DATAACCESSFUNCTOR_H
#define AR600_VISION_DATAACCESSFUNCTOR_H


/* Функторы для того, чтобы мат. операции могли
 * работать с любыми источниками данных.
 * (вместо лямбд)
 *
 *  Как я умею экономить 10 строк кода и не копипастить
 *  две простейшие функции путем нагромождения классов
 * 
 * Конкретно:
 *  - массив углов
 *  - PointCloud -> высоты точек
 */

//#include 

#include <pcl/point_cloud.h>

class DataAccessFunctor
{
public:
    virtual float operator()(int column, int row)=0;
};

//Массив float
class ArrayAccessFunctor: public DataAccessFunctor
{
public:
    ArrayAccessFunctor(boost::shared_ptr<float[]> array, int width, int height)
    {
        this->array = array;
        this->width = width;
        this->height = height;
    }

    float operator()(int column, int row)
    {
        return array[row * this->width + column];
    }

private:
    boost::shared_ptr<float[]> array;
    int width, height;
};

//Облако, надо взять Z
template <class PointT>
class PointCloudAccessFunctor: public  DataAccessFunctor
{
public:
    PointCloudAccessFunctor(boost::shared_ptr<pcl::PointCloud<PointT> > cloud)
    {
        this->cloud = cloud;
    }

    float operator()(int column, int row)
    {
        PointT p = cloud->at(column, row);
        if(std::isnan(p.x) || std::isnan(p.y))
            return std::numeric_limits<float>::quiet_NaN();

        return p.z;
    }

private:
    boost::shared_ptr<pcl::PointCloud<PointT> > cloud;
};


#endif //AR600_VISION_DATAACCESSFUNCTOR_H
