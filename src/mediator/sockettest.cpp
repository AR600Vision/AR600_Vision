#include <iostream>
#include <iomanip>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>

using namespace std;

void test(sockaddr_in & si_other, int s);


int main(int argc, char** argv)
{
    struct sockaddr_in  si_other;
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

    test(si_other, s);

    return 0;
}

void test(sockaddr_in & si_other, int s)
{
    int slen;
    double sendBuffer[]={0, 0, 0, 0, 0, 0};
    double recvBuffer[1000];

    slen=sizeof(si_other);
    if(sendto(s, sendBuffer,sizeof(sendBuffer) , 0 , (struct sockaddr *) &si_other, slen)==-1)
    {
        cout<<"Error sending, error code: "<<errno;
        return;
    }

    int recvSize = recvfrom(s, recvBuffer, 1000, 0, (struct sockaddr*)&si_other, (socklen_t*)&slen);
    if(recvSize==-1)
    {
        cout<<"Error receiving, error code: "<<errno;
        return;
    }

    recvSize/=sizeof(double);

    cout<<"Received: "<<recvSize <<":\n";

    cout << setw(5) << setprecision(2) << recvBuffer[0] << " | "<<setw(5)<<setprecision(2)<<recvBuffer[1]<<" | ";
    for(int i = 2; i<recvSize; i++)
        cout<<setw(5)<<setprecision(2)<<recvBuffer[i]<<" ";

    std::cout<<"\n";
}