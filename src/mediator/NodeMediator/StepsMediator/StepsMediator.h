//
// Created by garrus on 21.02.17.
//


/*!
 * \brief Взаимодействие с нодой расчета параметров шага
 * \author Markov Alex
 *
 * Отправляет запрос расчета шага в виде топика, принимает и сохраняет результат.
 * Позволяет обращаться к ноде в асинхронном режиме. См. базовый класс для подробностей
 * Извлекает параметры из массива и заносит результат в массив
 *
 * -------------------- ФОРМАТ ДАННЫХ --------------------
 * Полный формат данных см. в mediator_node.cpp
 * Здесь только данные, относящиеся к ноде
 *
 * Запрос:
 * ---|----|----|----|-----|-----|-----|----|----|
 * ...| dX | dY | dZ | dfX | dfY | dfZ | Xs | Ys |
 * ---|----|----|----|-----|-----|-----|----|----|
 *
 * dX, dY, dZ      - смещения корпуса робота/камеры относительно предыдущего запроса
 * dfX, dfY, dfZ   - поворот корпуса робота/камеры относительно предыдущего запроса (углы Эйлера)
 * Xs, Ys          - желаемые координаты шага
 *
 * Ответ:
 * ---|----|----|----|-----|-----|-----|----------|----|----|----|
 * ...| dX | dY | dZ | dfX | dfY | dfZ | can step | Xs | Ys | Zs |
 * ---|----|----|----|-----|-----|-----|----------|----|----|----|
 *
 * dX, dY, dZ      - аналогиично запросу, только вычисленные
 * dfX, dfY, dfZ   - при помощи RTABMap. В этой версии все 0
 * can step        - можно ли вообще совершить шаг в окрестности этой точки
 * Xs, Ys, Zs      - Уточненные координаты шага
 *
 */


#ifndef AR600_VISION_STEPSMEDIATOR_H
#define AR600_VISION_STEPSMEDIATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "../NodeMediatorBase.h"
#include <exception>

//TODO: Это  далеко не лучший способ подключить файл, наверняка Catking_Inlcude_Dirs могут лучше, но не работает
#include "../../../../../../devel/include/ar600_vision/StepResponse.h"

class StepsMediator : public NodeMediatorBase
{
public:

    /*!
     * Конструктор
     * @param nh дескриптор ноды ROS, чтобы подписвываться и публиковать
     * @param maxBufferSize размер буфера отпрвки, который будет выделен
     */
    StepsMediator(ros::NodeHandle & nh, int maxBufferSize);

    //Расчет шага закончен
    void ResultCallback(ar600_vision::StepResponse str);

protected:

    /*!
     * Преобразует массив double в нужные параметры
     * и отправляет в топик
     * @param buffer
     * @param count
     */
    virtual void _SendRequest(const double* buffer, int count) override;

private:
    ros::Publisher step_publisher;
    ros::Subscriber step_subscriber;

};


#endif //AR600_VISION_STEPSMEDIATOR_H
