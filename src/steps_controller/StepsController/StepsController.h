/**
 * Этот класс реализует задачу определения поверхности в точке шага робота.
 * Основная реализация ноды steps_controller находится здесь.
 *
 * Задачи:
 *  - По заданной координате шага и ряду других параметров, определять
 *    пригодна ли поверхность для осуществления шага
 *  - Поиск пригодной поверхности для шага в заданной окрестности
 *
 */

#ifndef AR600_VISION_STEPS_CONTROLLER_H
#define AR600_VISION_STEPS_CONTROLLER_H

#include <string>

//PCL
#include <pcl/visualization/pcl_visualizer.h>

//Project
#include "settings.h"
#include <steps_controller/CloudTransforms/CloudTransforms.h>

class StepsController
{
public:
    StepsController();

    /**
     * Принимает очередное изменения облака точек
     * и обрабатывает его
     */
    void UpdateFrame(pcl::PCLPointCloud2::Ptr pointCloud2);

    /* Т.к. есть визуализатор, то необходимо обеспечить
     * обертки над методами, чтобы можно было организовать цикл
     * приложения
     */
    bool wasStopped () const;
    void spinOnce (int time = 1, bool force_redraw = false);

private:

    //Константы
    static const std::string point_cloud_name;

    //Основные параметры
    //TODO: сделать возможность настройки параметров

    //Необходимые объекты
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;                            //Визуализатор
    CloudTransforms cloud_transforms;                                                       //Для преобразований облака

    boost::shared_ptr<pcl::visualization::PCLVisualizer> setup_vizualizer();                //Создание визуализатор

};


#endif //AR600_VISION_STEPS_CONTROLLER_H
