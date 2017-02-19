/*!
 *  \brief     Mediator node for AR600 CV project
 *  \author    Titov Alex
 *  \date      2017
 *  \warning   Improper use can crash your application
 *  \copyright GNU Public License.
 */
// TODO A callbacks should stay as simple and fast as possible
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>


/// Storage for response, with sinchronization
class NavigationBuffer
{
public:
    NavigationBuffer()
    {
        path = nav_msgs::Path(); // todo add some poses to here
        calc_finished = true;
    }

    void SetData(const nav_msgs::Path new_data)
    {
        m.lock();
        path = new_data;
        calc_finished = true;
        m.unlock();
    }

    nav_msgs::Path GetData()
    {
        std::lock_guard <std::mutex> l(m);
        return path;
    }

    // lock guard http://stackoverflow.com/questions/16522889/mutex-when-returning-object-value
    bool isCalcFinished()
    {
        std::lock_guard <std::mutex> l(m);
        return calc_finished;
    }

    // http://stackoverflow.com/questions/2650458/does-access-write-to-boolean-object-needs-synchronization
    void startCalc()
    {
        m.lock();
        calc_finished = false;
        m.unlock();
    }
private:
    nav_msgs::Path path;
    bool calc_finished;
    std::mutex m;
};

/// Contains callback and buffer
/// Converts request from double array to required format and sends it to processing node
/// Converts response to double vector to send to FRUND
class NavigationCommunicator
{
public:
    NavigationCommunicator(ros::NodeHandle & n)
    {
        path_subscriber = n.subscribe("/footstep_planner/path", 1, &NavigationCommunicator::callback, this);
        goal_publisher = n.advertise<geometry_msgs::PoseStamped>("goal", 1, true);
    }

    /// Make request from double array and send it to processing node
    /// Here communicator parses input and get what he want
    void SendReqest(double * reqest_buffer)
    {
        int rviz_control = int(reqest_buffer[0]);
        if (buffer.isCalcFinished() && !rviz_control)
        {
            geometry_msgs::PoseStamped goal;
            goal.pose.position.x = reqest_buffer[1];
            goal.pose.position.y = reqest_buffer[2];
            // fixme work with orientation. while now it's zero

            goal_publisher.publish(goal);
        }
    }

    /// Converts response to double vector
    std::vector<double> GetData()
    {
        nav_msgs::Path path = buffer.GetData();
        int steps_count = path.poses.size();
        std::vector<double> response_buffer(2*steps_count+1);

        response_buffer[0] = steps_count;
        for (int i = 0; i < steps_count; i++)
        {
            response_buffer[1 + 2*i] = path.poses[i].pose.position.x;
            response_buffer[1 + 2*i + 1] = path.poses[i].pose.position.y;
            // TODO are we need their orientation or something like that?
        }
    }

    void callback(nav_msgs::Path path)
    {
        buffer.SetData(path);
    }
private:
    ros::Publisher goal_publisher;
    ros::Subscriber path_subscriber;
    NavigationBuffer buffer;
};

/// Initializes communicators
/// Sends and recvs data from FRUND via sockets
/// Transfers data to communicators
struct MediatorNode
{
public:
    MediatorNode() : nav(n), spinner(2)
    {

    }
    ~MediatorNode()
    {
        ros::waitForShutdown();
    }

    /// Connect and start spinner
    bool prepare()
    {
        if (!connect("127.0.0.1", 50000))
            return false;

        spinner.start();
        return true;
    }


    void operate()
    {
        while(ros::ok())
        {
            /// Recv data from socket
            double request_buffer[100];
            socklen_t slen_req = sizeof(si_frund);
            recvfrom(sock_desc, request_buffer, 4*sizeof(double), 0, (sockaddr *)&si_frund, (socklen_t *)&slen_req);

            /// Give data to communicators
            nav.SendReqest(request_buffer);
            // TODO viz.SendReqest(request_buffer);

            /// Get response from communicators
            std::vector<double> response_buffer = nav.GetData();
            // TODO add viz response buffer here and split them here into one, then send via sockets

            int size = response_buffer.size()*sizeof(double);
            socklen_t slen_res = sizeof(si_frund);
            sendto(sock_desc, response_buffer.data(), size, 0, (sockaddr *)&si_frund, (socklen_t)slen_res);
        }
    }

private:
    bool connect(char * ip, int port)
    {
        /// Create socket
        sock_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_desc == -1)
        {
            ROS_ERROR("Can't create socket");
            return false;
        }

        si_frund.sin_family = AF_INET;
        si_frund.sin_port = htons(port);
        si_frund.sin_addr.s_addr = inet_addr(ip);
        return true;
    }

    NavigationCommunicator nav;
    struct sockaddr_in si_frund;
    int sock_desc;
    ros::NodeHandle n;
    ros::AsyncSpinner spinner;
};


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ar600e_receiver_node");

    MediatorNode n;
    if (!n.prepare()) // TODO make it a cycle maybe
        return 1;

    ROS_INFO("Receiver node was started successfully");

    n.operate();
}
