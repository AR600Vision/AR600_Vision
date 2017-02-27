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
 *  |---------|------|------ ... ---|
 *  | node id | rviz | data         |
 *  |---------|------|------ ... ---|
 *
 *  node id      номер вызываемой ноды
 *  rviz         входные данные беруться из запроса (0) или из RVIZ (1) (или еще откуда-нибудь)
 *
 *  Ответ:
 *  |---------|--------|--------|--------------|------ ... ---|
 *  | node id | status | actual | elapsed time | data         |
 *  |---------|--------|--------|--------------|------ ... ---|
 *
 *  node id       Номер ноды, которая отвечает (такой же, как в запросе)
 *  status        Код ошибки
 *  actual        Является ли эти данные ответом на текущий (1) или один из предыдущих (0)
 *                запросов. В этой версии ответ в пределах одного запроса невозможен, всегда 0
 *  elapsed time  Время, прошедшее с момента запроса. В этой версии всегда 0.
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
    NODE_NOT_FOUND,
    NODE_INTERNAL_ERROR
};

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

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "ar600e_receiver_node");
    ros::NodeHandle nh;

    ROS_INFO("Receiver node started...");

    //Заполняем список медиаторов
    std::vector<NodeMediatorBase*> mediators =
    {
        new StepsMediator(nh, 1000),
        new PathMediator(nh, 1000)
    };


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
    int sendCount;

    ROS_INFO("UDP listen thread started...");

    //Цикл приема запросов
    while (ros::ok())
    {
        //Принимаем данные
        socklen_t slen_req = sizeof(si_frund);
        ssize_t recvSize = recvfrom(sock_desc, buffer, maxBufferSize * sizeof(double), 0, (sockaddr *)&si_frund, &slen_req);

        //Проверка корректности данных
        if(recvSize == -1)
        {
            ROS_ERROR("Error receiving data.Error code: %d",errno);
            continue;
        }

        if(recvSize % sizeof(double) != 0)
        {
            ROS_ERROR("Received data isn't array of doubles");
            continue;
        }

        recvSize /= sizeof(double);

        ROS_INFO("Received data");

        if(recvSize <3)
        {
            ROS_ERROR("Input request has invalid package format");
            sendError(sock_desc, si_frund, INVALID_PACKAGE_FORMAT);
            continue;
        }

        int commandId = buffer[0];
        if(commandId <0 || commandId >= mediators.size())
        {
            ROS_ERROR("Command id %d is out of range", commandId);
            sendError(sock_desc, si_frund, NODE_NOT_FOUND);
            continue;
        }

        ROS_INFO("Starting NodeMediator #%d",commandId);

        //Обработка
        try
        {
            //+1 - отрезаем nodeId
            mediators[commandId]->SendRequest(buffer+1, recvSize-1);

            //TODO: таймоут

            //+4 потому что отрезаем nodeId, status, is actual, elapsed time
            sendCount = mediators[commandId]->ReadResponse(buffer + 4, maxBufferSize);

        }
        catch(std::exception ex)
        {
            ROS_ERROR("Internal error in node: %s",ex.what());
            sendError(sock_desc, si_frund, NODE_INTERNAL_ERROR);
        }

        //Заполняем служебную информацию
        buffer[0] = commandId;
        buffer[1] = (double)NO_ERROR;
        buffer[2] = 0; //Не 0 только при реализации таймоута
        buffer[3] = 0; //Не поддерживается

        socklen_t slen_res = sizeof(si_frund);
        if(sendto(sock_desc, buffer, (sendCount + 4) * sizeof(double), 0, (sockaddr *)&si_frund, (socklen_t)slen_res)!=-1)
            ROS_INFO("Sent response");
        else
            ROS_ERROR("Error sending response. Error code: %d", errno);

    }
}

//Отправляет код ошибки
void sendError(int sock_desc, sockaddr_in si_frund, ERRORS error)
{
    socklen_t slen_res = sizeof(si_frund);
    double errorDouble = (double)error;
    sendto(sock_desc, &errorDouble, sizeof(double), 0, (sockaddr *)&si_frund, (socklen_t)slen_res);
}
