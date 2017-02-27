//
// Created by garrus on 20.02.17.
//


#include "NodeMediatorBase.h"
#include <iostream>


NodeMediatorBase::NodeMediatorBase(int maxBufferSize):
        _bufferMutex(),
        BufferMaxSize(maxBufferSize)
{
    //Инициализация буфферов и прочего
    SendBuffer = new double[BufferMaxSize];
    _isCalcFinished = true;
    DataSize = 0;
    memset(SendBuffer,  0, sizeof(double)*BufferMaxSize);

    calc_start = std::chrono::system_clock::now();
    response_time_limit = 10.0;
}

NodeMediatorBase::~NodeMediatorBase()
{
    if(SendBuffer!= nullptr)
        delete[] SendBuffer;
}

//Отправка запроса ноде
void NodeMediatorBase::SendRequest(const double* buffer, int count)
{
    std::lock_guard<std::mutex> l(_bufferMutex);
    bool isRviz = buffer[0]==1;

    auto now = std::chrono::system_clock::now();
    double response_time = (now - calc_start).count();
    bool response_timeout = response_time > response_time_limit;

    if((_isCalcFinished || response_timeout) && !isRviz)
    {
        //Отрезаем rviz
        calc_start = std::chrono::high_resolution_clock::now();
        _SendRequest(buffer+1, count-1);
        _isCalcFinished = false;
    }
}

//Чтение результата
int NodeMediatorBase::ReadResponse(double* buffer, int maxCount)
{
    std::lock_guard<std::mutex> l(_bufferMutex);

    if(DataSize>maxCount)
        throw std::overflow_error("Output buffer is smaller then data size");

    memcpy(buffer, SendBuffer, sizeof(double)*DataSize);
    std::cout<<"\n";


    return DataSize;
}

//Данные от ноды получены
void NodeMediatorBase::Done(std::function<void(double*, int & count, int maxCount)> setter)
{
    std::lock_guard<std::mutex> l(_bufferMutex);
    setter(SendBuffer, DataSize, BufferMaxSize);
    _isCalcFinished = true;

}