#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>

using namespace std;

int main(int argc, char** argv)
{
    struct sockaddr_in  si_other;
    int slen;
    int s;

    if((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
    {
        cout<<"can't create socket";
        return  -1;
    }

    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(12833);
    if(inet_aton("127.0.0.1" , &si_other.sin_addr)==-0)
    {
        cout<<"inet_aton failed";
        return -1;
    }

    double sendBuffer[]={0, 0, 0, 0, 0, 0, 0, 0, 1, -1};
    double recvBuffer[1000];

    slen=sizeof(si_other);
    if(sendto(s, sendBuffer,sizeof(sendBuffer) , 0 , (struct sockaddr *) &si_other, slen)==-1)
    {
        cout<<"Error sending, error code: "<<errno;
        return -1;
    }

    int recvSize = recvfrom(s, recvBuffer, 1000, 0, (struct sockaddr*)&si_other, (socklen_t*)&slen);
    if(recvSize==-1)
    {
        cout<<"Error receiving, error code: "<<errno;
        return -1;
    }

    int count = recvSize/sizeof(double);
    cout<<"Received: "<<count <<":\n";
    for(int i = 0; i<count; i++)
        std::cout<<recvBuffer[i]<<" ";

    return 0;
}