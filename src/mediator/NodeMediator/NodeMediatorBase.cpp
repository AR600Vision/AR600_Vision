//
// Created by garrus on 20.02.17.
//


#include "NodeMediatorBase.h"

NodeMediatorBase::NodeMediatorBase(int maxBufferSize):
        Mutex(),
        BufferMaxSize(maxBufferSize)
{
    //Инициализация буфферов и прочего
    SendBuffer = new double[BufferMaxSize];
    isDone = true;
    memset(SendBuffer,  0, sizeof(double)*BufferMaxSize);
}

NodeMediatorBase::~NodeMediatorBase()
{
    if(SendBuffer!= nullptr)
        delete[] SendBuffer;
}

//Чтение результата
int NodeMediatorBase::ReadResponse(double* buffer, int maxCount)
{
    std::lock_guard<std::mutex> l(Mutex);

    if(BufferMaxSize + 1 > maxCount)
    {
        ROS_ERROR("Buffer is too small to copy response");
        return -1;
    }

    //Копируем данные
    if(buffer!=nullptr)
    {
        memcpy(buffer, SendBuffer, sizeof(double) * BufferMaxSize);
        buffer[BufferMaxSize] = isDone;
    }

    return BufferMaxSize + 1;
}

//Завершен ли расчет
bool NodeMediatorBase::IsDone()
{
    return isDone;
}
