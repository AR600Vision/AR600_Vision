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
    _isCalcFinished = false;
    _firstCall = true;
    DataSize = 0;
    memset(SendBuffer,  0, sizeof(double)*BufferMaxSize);
}

NodeMediatorBase::~NodeMediatorBase()
{
    if(SendBuffer!= nullptr)
        delete[] SendBuffer;
}

//Отправка запроса ноде
bool NodeMediatorBase::SendRequest(const double* buffer, int count)
{
    std::lock_guard<std::mutex> l(_bufferMutex);

    if(_isCalcFinished || _firstCall)
    {
        bool isOk = _SendRequest(buffer, count);
        _firstCall = false;

        if(isOk)
            _isCalcFinished = false;

        return isOk;
    }
    else
        return true;
}

//Чтение результата
int NodeMediatorBase::ReadResponse(double* buffer, int maxCount)
{
    std::lock_guard<std::mutex> l(_bufferMutex);

    if(DataSize + 1 > maxCount)
        return -1;

    //Устанавливаем флаг завершенности расчета

    //Копируем данные
    if(buffer!=nullptr)
        memcpy(buffer, SendBuffer, sizeof(double)*DataSize);

    buffer[BufferMaxSize] = (double)_isCalcFinished;

    return BufferMaxSize + 1;
}

//Данные от ноды получены
void NodeMediatorBase::Done(std::function<void(double*, int & count, const int maxCount)> setter)
{
    std::lock_guard<std::mutex> l(_bufferMutex);
    setter(SendBuffer, DataSize, BufferMaxSize);
    _isCalcFinished = true;

}
