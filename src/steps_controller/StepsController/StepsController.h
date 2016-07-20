/**
 * Этот класс реализует задачу определения поверхности в точке шага робота.
 * Основная реализация ноды steps_controller находится здесь.
 *
 * Задачи:
 *  - По заданной координате шага и ряду других параметров, определять
 *    пригодна ли поверхность для осуществления шага
 *  - Поиск пригодной поверхности для шага в заданной окрестности
 *
 *
 * ВНИМАНИЕ:
 *  UpdateFrame и CalculateStep благодаря ROS вызываются в одном потоке.
 *  При использовании без ROS требуется внешняя синхронизация
 */

#ifndef AR600_VISION_STEPS_CONTROLLER_H
#define AR600_VISION_STEPS_CONTROLLER_H

#include <string>

//PCL
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/integral_image_normal.h>

//Project
#include "settings.h"
#include <steps_controller/CloudTransforms/CloudTransforms.h>
#include "StepRequest.h"
#include "StepsParams.h"

namespace StepsController
{

    class StepsController {
    public:
        StepsController();

        /**
         * Принимает очередное изменения облака точек
         * и обрабатывает его
         */
        void UpdateFrame(pcl::PCLPointCloud2::Ptr pointCloud2);

        /**
         * Запрос на проверку области на пригодность шага
         */
        StepControllerResponse CalculateStep(StepControllerRequest request);

        /* Т.к. есть визуализатор, то необходимо обеспечить
         * обертки над методами, чтобы можно было организовать цикл
         * приложения
         */
        bool wasStopped() const;

        void spinOnce(int time = 1, bool force_redraw = false);

    private:

        //Константы
        static const std::string point_cloud_name;
        static const std::string search_cloud_name;
        static const std::string search_cube_name;
        static const std::string foot_cube_name;

        //Основные параметры
        StepsParams steps_params;

        //Необходимые объекты
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;                            //Визуализатор
        CloudTransforms cloud_transforms;                                                       //Для преобразований облака
        pcl::PointCloud<POINT_TYPE>::Ptr current_cloud;                                         //Сохраненное на последнем сообщении топика облако

        boost::shared_ptr<pcl::visualization::PCLVisualizer> setup_vizualizer();                //Создание визуализатор
        void draw_cube(float x, float y, float z,                                               //Обертка для более удобного рисования кубов
                       float depth, float width, float height,                                  //Задается координатами центра и размерами
                       float r, float g, float b,
                       const std::string &id="cube");



    };

}

#endif //AR600_VISION_STEPS_CONTROLLER_H
