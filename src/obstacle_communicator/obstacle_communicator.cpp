/*
 * 5005
 * "1" - запустить решение
 * "2" - выключить
 * "3" - остановка
 *
 * Лайт-версия омбена сообщениями
 * о препятствии, потому что ФРУНД пока не умеет
 * останавливаться по
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sys/socket.h>
#include <arpa/inet.h>


class CommandSender
{
public:
    CommandSender(const char* ip, int port)
    {
        if((_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
        {
            ROS_ERROR("Can't create socket");
            return;
        }

        memset((char *) &si_other, 0, sizeof(si_other));
        si_other.sin_family = AF_INET;
        si_other.sin_port = htons(port);
        if(inet_aton(ip , &si_other.sin_addr)==0)
        {
            ROS_ERROR("Invalid ip address");
            return;
        }
    }

    bool SendCommand(const char command)
    {
        int slen=sizeof(si_other);
        if(sendto(_socket, &command, 1 , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            ROS_ERROR("Error sending, error code: %d", errno);
            return false;
        }

        return true;
    }

private:
    sockaddr_in si_other;
    int _socket;
};

ros::Subscriber obstacle_subscriber;
CommandSender control_sender("192.168.1.10", 5005);

void ObstacleChanged(std_msgs::Bool isObstacle);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObstacleCommunicator");
    ros::NodeHandle nh;

    ROS_INFO("ObstacleCommunicator node started");

    obstacle_subscriber = nh.subscribe("obstacle_detector/obstacle_changed", 1, &ObstacleChanged);
    ros::spin();

    return 0;
}


void ObstacleChanged(std_msgs::Bool isObstacle)
{
    if(isObstacle.data)
    {
        //Послать команду на воспроизведение фразы
        //и на остановку
        ROS_INFO("Obstacle detected");

        if(!control_sender.SendCommand('3'))
            ROS_ERROR("Error sending command to control program");
    }
}
