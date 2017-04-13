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
#include <fstream>
#include <chrono>
#include <tf/transform_listener.h>

#include "NodeMediator/NodeMediatorBase.h"
#include "NodeMediator/StepsMediator/StepsMediator.h"
#include "NodeMediator/PathMediator/PathMediator.h"
#include "NodeMediator/ObstacleMediator/ObstacleMediator.h"

using namespace std;
using namespace std::chrono;

StepsMediator* stepsMediator;
ObstacleMediator* obstacleMediator;;

bool isReceive;
steady_clock::time_point start;

void receiveFunc(int port, int maxBufferSize);
int receive(int socket, double* buffer, int maxBufferSize, sockaddr_in* si_frund);
double getSeconds();

float fakeTime = 0;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ar600e_receiver_node");
    ros::NodeHandle nh;

    stepsMediator = new StepsMediator(nh);
    obstacleMediator = new ObstacleMediator(nh);

    ROS_INFO("Receiver node started...");

    start = steady_clock::now();

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
    int sendSize = 6 + 1 + 1; //wtf???

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


    //Дебажный вывод лога
    std::ofstream f;

    f.open("/home/user/log.txt", std::ios_base::out | std::ios_base::trunc);
    f << "TIME\txl\tyl\tzl\txr\tyr\tzr\tcan_step\n";

    //Цикл приема запросов
    while (isReceive)
    {

        ssize_t recvSize;
        if ((recvSize = receive(sock_desc, buffer, maxBufferSize, &si_frund)) == -1)
        {
            std::cout << "Cant recv" << std::endl;
            return;

        }


        /*
         Формат запроса
         |-----|-----|-----|-----|-----|-----|--------------|
         | xsl | ysl | zsL | xsr | ysr | zsr | should check |
         |-----|-----|-----|-----|-----|-----|--------------|
                           |                 |
             Координаты    |  Координаты     | Надо ли
             шага лев      |  шага прав      | проверять шаг

         Формат ответа
         |-----|-----|-----|-----|-----|-----|--------------|-------------|
         | xsl | ysl | zsL | xsr | ysr | zsr | can step     | is obstacle |
         |-----|-----|-----|-----|-----|-----|--------------|-------------|
                           |                 |              |
             Координаты    |  Координаты     | Можно ли     | Есть ли
             шага лев      |  шага прав      | делать шаг   | препятствие

         */

        ROS_INFO("Received data");

        for(int i = 0; i<recvSize; i++)
        {
            std::cout<<buffer[i]<<" ";
        }

        std::cout<<"\n";

        //Обработка запросов
        /*if(buffer[6]==1)
        {
            stepsMediator->SendRequest(buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
            bool isDone = stepsMediator->ReadResponse(buffer);
        }*/

        buffer[0]=1;
        buffer[1]=2;
        buffer[2]=3;
        buffer[3]=4;
        buffer[4]=5;
        buffer[5]=6;
        buffer[6]=7;

        //Обработка препятствий
        bool isObstacle = obstacleMediator->IsObstacle();
        buffer[7] = isObstacle;


        //Отправка запрсоов
        socklen_t slen_res = sizeof(si_frund);
        if(sendto(sock_desc, buffer, sendSize * sizeof(double), 0, (sockaddr *)&si_frund, (socklen_t)slen_res)!=-1)
        {
            f<<fakeTime*0.005<<"  ";
            for(int i = 0; i<sendSize; i++)
            {
                std::cout<<buffer[i]<<"\t";
                f<<buffer[i]<<"  ";
            }

            std::cout<<"\n";
            f<<"\n";
            ROS_INFO("Sent response");
        }
        else
            ROS_ERROR("Error sending response. Error code: %d", errno);

        f.flush();
        fakeTime++;
    }
}


int receive(int socket, double* buffer, int maxBufferSize, sockaddr_in* si_frund)
{
    //Принимаем данные
    socklen_t slen_req = sizeof(si_frund);
    ssize_t recvSize = recvfrom(socket, buffer, maxBufferSize * sizeof(double), 0, (sockaddr *) si_frund,
                                &slen_req);

    //Проверка корректности данных
    if (recvSize == -1) {
        ROS_ERROR("Error receiving data.Error code: %d", errno);
        return -1;
    }

    if (recvSize % sizeof(double) != 0) {
        ROS_ERROR("Received data isn't array of doubles");
        return -1;
    }

   return recvSize / sizeof(double);
}


//Возвращает время от начала программы
double getSeconds()
{
    duration<double> time_span = duration_cast<duration<double>>(steady_clock::now() - start);
    return time_span.count();
}
