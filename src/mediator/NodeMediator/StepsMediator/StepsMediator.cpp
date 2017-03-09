//
// Created by garrus on 21.02.17.
//

#include "StepsMediator.h"

const char request_topic[]     = "steps_controller/step_request";
const char response_topic[]    = "steps_controller/step_response";


StepsMediator::StepsMediator(ros::NodeHandle & nh) :
        NodeMediatorBase(7)
{
    step_publisher = nh.advertise<geometry_msgs::PoseStamped>(request_topic, 100);
    step_subscriber = nh.subscribe(response_topic, 1, &StepsMediator::ResultCallback, this);
    stepCnt=0;
}

void StepsMediator::SendRequest(double xsl, double ysl, double zsl, double xsr, double ysr, double zsr)
{
    std::lock_guard<std::mutex> l(Mutex);
    std::cout<<"xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";


    if(!isDone)
        return;

    //Надо считать
    publish(xsl, ysl, zsl);
    _xs = xsr;
    _ys = ysr;
    _zs = zsr;

    stepCnt++;
    isDone = false;
}

//расчет шага закончен
void StepsMediator::ResultCallback(ar600_vision::StepResponse step)
{

    if(stepCnt==1)
    {
        //Посчитали первую, запускаем вторую
        publish(_xs, _ys, _zs);

        //Сохраняем пааметры левой ноги
        _xs = step.Pose.position.x;
        _ys = step.Pose.position.y;
        _zs = step.Pose.position.z;
        canStep = step.CanStep;

        stepCnt++;
        return;
    }

    if(BufferMaxSize<7)
    {
        ROS_ERROR("StepsMediator: Inner buffer is too small to contains all result data");
        throw std::overflow_error("Inner buffer is too small to contains all result data");
    }

    Mutex.lock();

    //Левая нога
    SendBuffer[0]=_xs;
    SendBuffer[1]=_ys;
    SendBuffer[2]=_zs;

    //Правая нога
    SendBuffer[3]=step.Pose.position.x;
    SendBuffer[4]=step.Pose.position.y;
    SendBuffer[5]=step.Pose.position.z;

    SendBuffer[6]= (step.CanStep & canStep) ? 10 : 0;
    std::cout<<"===========================================================\n";

    stepCnt=0;
    isDone = true;

    Mutex.unlock();
}

//Публикует в топик
void StepsMediator::publish(float x, float y, float z)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    ROS_INFO("steps_controller is called");
    step_publisher.publish(pose);
}


