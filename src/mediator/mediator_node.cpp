/*!
 *  \brief     Mediator node for AR600 CV project
 *  \authors    Titov Alex, Markov Alex
 *  \date      2017
 *  \warning   Improper use can crash your application
 *  \copyright GNU Public License.
 *
 *  Принимает запросы по UDP от ФРУНДа, асинхронно вызывает
 *  ноды, возвращает ответы
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


StepsMediator* stepsMediator;

bool isReceive;

/*!
 * Функция слушания
 * @param port порт слушания
 * @param maxBufferSize максимальный размер буфера приема, отправки
 * @param mediators список обработчиков
 */
void receiveFunc(int port, int maxBufferSize);


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ar600e_receiver_node");
    ros::NodeHandle nh;

    stepsMediator = new StepsMediator(nh);

    ROS_INFO("Receiver node started...");


    isReceive = true;
    std::thread listenThread(receiveFunc, 12833, 10);
    listenThread.detach();

    ros::spin();
    return 0;
}

//Функция слушания
void receiveFunc(int port, int maxBufferSize)
{
    sockaddr_in si_frund;
    sockaddr_in si_me;

    //Количество отсылаемых элементов
    int sendSize = 6 + 1 + 1;

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

        for(int i = 0; i<recvSize; i++)
        {
            std::cout<<buffer[i]<<" ";
        }

        std::cout<<"\n";

        //Отправка запросов
        if(buffer[6]==1)
        {
            stepsMediator->SendRequest(buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
            stepsMediator->ReadResponse(buffer, maxBufferSize);
        }


        socklen_t slen_res = sizeof(si_frund);
        if(sendto(sock_desc, buffer, sendSize * sizeof(double), 0, (sockaddr *)&si_frund, (socklen_t)slen_res)!=-1)
        {
            for(int i = 0; i<recvSize; i++)
            {
                std::cout<<buffer[i]<<" ";
            }

            std::cout<<"\n";
            ROS_INFO("Sent response");
        }
        else
            ROS_ERROR("Error sending response. Error code: %d", errno);


    }
}