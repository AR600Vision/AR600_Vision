/*!
 *  \brief     Mediator node for AR600 CV project
 *  \authors    Titov Alex, Markov Alex
 *  \date      2017
 *  \warning   Improper use can crash your application
 *  \copyright GNU Public License.
 *
 *  Принимает запросы по UDP от ФРУНДа, асинхронно вызывает
 *  ноды, возвращает ответы
 *
 *  -------------------- ФОРМАТ ДАННЫХ --------------------
 *  данные пеередаются в виде массива double
 *
 *  Запрос:
 *  |------ ... ---|------ ... ---|--- ... ---|------ ... ---|
 *  | data 1       | data 2       |           | data n       |
 *  |------ ... ---|------ ... ---|--- ... ---|------ ... ---|
 *
 *  data1..n   Данные запроса к i-й ноде
 *
 *  Ответ:
 *  |--------|--------|------ ... ---|--------|------ ... ---|------|--- ... ---|--------|------ ... ---|
 *  | status | done 1 | data 1       | done m | data 2       | done |           | done m | data m       |
 *  |--------|--------|------ ... ---|--------|------ ... ---|------|--- ... ---|--------|------ ... ---|
 *
 *  status        Код ошибки
 *  done1..m      Расчет завершен (0/1)
 *  data1..m      Результат расчета i-й нодой
 *
 *  КОДЫ ОШИБОК:
 *   0     Без ошибок
 *   1     Неверный формат пакета
 *   2     Запрошенная нода не найдена
 *   3     Неизвестная внутренняя ошибка в ноде
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <thread>


#include "NodeMediator/NodeMediatorBase.h"
#include "NodeMediator/StepsMediator/StepsMediator.h"
#include "NodeMediator/PathMediator/PathMediator.h"

/*!
 * Коды ошибок
 */
enum ERRORS
{
    NO_ERROR,
    INVALID_PACKAGE_FORMAT,
    MEDIATOR_ERROR,
    NODE_INTERNAL_ERROR
};

bool isReceive;

/*!
 * Функция слушания
 * @param port порт слушания
 * @param maxBufferSize максимальный размер буфера приема, отправки
 * @param mediators список обработчиков
 */
void receiveFunc(int port, int maxBufferSize, std::vector<NodeMediatorBase*> & mediators);

/*!
 * Отправляет код ошибки
 * @param sock_desc
 * @param si_frund
 */
void sendError(int sock_desc, sockaddr_in si_frund, ERRORS error);

bool SendRequests(const std::vector<NodeMediatorBase *> &mediators, sockaddr_in si_frund, int sock_desc, const double *buffer, ssize_t recvSize);
int ReadResponse(std::vector<NodeMediatorBase *> &mediators, sockaddr_in si_frund, int sock_desc, double *buffer, int maxBufferSize);

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "ar600e_receiver_node");
    ros::NodeHandle nh;

    ROS_INFO("Receiver node started...");

    //Заполняем список медиаторов
    std::vector<NodeMediatorBase*> mediators =
    {
        new StepsMediator(nh)
    };


    isReceive = true;
    std::thread listenThread(receiveFunc, 12833, 1000, std::ref(mediators));
    listenThread.detach();

    ros::spin();
    return 0;
}

//Функция слушания
void receiveFunc(int port, int maxBufferSize, std::vector<NodeMediatorBase*> & mediators)
{
    sockaddr_in si_frund;
    sockaddr_in si_me;
    int elementsWrited;

    /// Create socket
    int sock_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_desc == -1) {
        ROS_ERROR("Can't create socket");
        return;
    }

    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    if(bind(sock_desc, (sockaddr*)&si_me, sizeof(si_me)) == -1)
    {
        ROS_ERROR("Can't bind socket");
        return;
    }

    //Буфер для данных
    //Пока я использую один буфер, но можно и два
    double* buffer = new double[maxBufferSize];

    ROS_INFO("UDP listen thread started...");

    //Цикл приема запросов
    while (isReceive)
    {
        //Принимаем данные
        socklen_t slen_req = sizeof(si_frund);
        ssize_t recvSize = recvfrom(sock_desc, buffer, maxBufferSize * sizeof(double), 0, (sockaddr *) &si_frund,
                                    &slen_req);

        //Проверка корректности данных
        if (recvSize == -1) {
            ROS_ERROR("Error receiving data.Error code: %d", errno);
            continue;
        }

        if (recvSize % sizeof(double) != 0) {
            ROS_ERROR("Received data isn't array of doubles");
            continue;
        }

        recvSize /= sizeof(double);

        ROS_INFO("Received data");

        //Разделяем запрос на части, распределяем между медиаторами,
        //вызываем, собираем результат обратно в массив
        //если в какой-то ноде на чтении или записи что-то кинект
        //исключение, то весь цикл вызова-сбора прервется

        try
        {
            //Разделяем запрос между медиаторами
            if(!SendRequests(mediators, si_frund, sock_desc, buffer, recvSize))
                continue;

            //TODO: Timeout

            elementsWrited  = ReadResponse(mediators,  si_frund, sock_desc, buffer, maxBufferSize);
            if(elementsWrited==-1)
                continue;

            ROS_INFO("Response readed");

        }
        catch(std::exception ex)
        {
            ROS_ERROR("Internal error in node: %s",ex.what());
            sendError(sock_desc, si_frund, NODE_INTERNAL_ERROR);
            continue;
        }

        buffer[1] = (double)NO_ERROR;
        socklen_t slen_res = sizeof(si_frund);
        if(sendto(sock_desc, buffer, elementsWrited * sizeof(double), 0, (sockaddr *)&si_frund, (socklen_t)slen_res)!=-1)
            ROS_INFO("Sent response");
        else
            ROS_ERROR("Error sending response. Error code: %d", errno);

    }
}



//Раздаеляет запрос на запросы к отдельным медиаторам
bool SendRequests(const std::vector<NodeMediatorBase *> &mediators, sockaddr_in si_frund, int sock_desc, const double *buffer, ssize_t recvSize)
{
    int elementsRead = 0;
    for (int i = 0; i < mediators.size(); i++)
    {
        int requiredRequestLength = mediators[i]->RequestLength();
        if(recvSize - elementsRead < requiredRequestLength)
        {
            ROS_ERROR("Not enough paramters");
            sendError(sock_desc, si_frund, INVALID_PACKAGE_FORMAT);
            return false;
        }

        if(!mediators[i]->SendRequest(buffer+elementsRead, requiredRequestLength))
        {
            ROS_ERROR("Not enough paramters");
            sendError(sock_desc, si_frund, INVALID_PACKAGE_FORMAT);
            return false;
        }

        elementsRead+=requiredRequestLength;
    }

    return true;
}

//Собираем результаты
int ReadResponse(std::vector<NodeMediatorBase *> &mediators, sockaddr_in si_frund, int sock_desc, double *buffer, int maxBufferSize)
{
    int elementsWrited = 1;       //Начинаем с 1, потому что там статус
    for(int i = 0; i<mediators.size(); i++)
    {
        int size = mediators[i]->ReadResponse(buffer + elementsWrited, maxBufferSize - elementsWrited);
        if (size == -1) {
            ROS_ERROR("Buffer is too small to contains all response data");
            sendError(sock_desc, si_frund, MEDIATOR_ERROR);
            return -1;
        }

        elementsWrited += size;
    }

    return elementsWrited;
}

//Отправляет код ошибки
void sendError(int sock_desc, sockaddr_in si_frund, ERRORS error)
{
    socklen_t slen_res = sizeof(si_frund);
    double errorDouble = (double)error;
    sendto(sock_desc, &errorDouble, sizeof(double), 0, (sockaddr *)&si_frund, (socklen_t)slen_res);
}
