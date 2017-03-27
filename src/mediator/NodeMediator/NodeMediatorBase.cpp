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
bool NodeMediatorBase::ReadResponse(double* buffer)
{
    std::lock_guard<std::mutex> l(Mutex);

    //Копируем данные
    memcpy(buffer, SendBuffer, sizeof(double) * BufferMaxSize);

    return isDone;
}
