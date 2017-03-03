#include <iostream>
#include <iomanip>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <omp.h>
#include <unistd.h>

using namespace std;

int test(sockaddr_in & si_other, int s, double* sendBuffer, int sendBufferSize, double* recvBuffer, int recvBufferSize);


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

    double sendBuffer[]={0, 0, 0, 0, 0, 0};
    double recvBuffer[1000];

    double time0 = omp_get_wtime();
    double time;

    for(int i = 0; i<1000; i++)
    {
        int recvSize = test(si_other, s, sendBuffer, sizeof(sendBuffer), recvBuffer, sizeof(recvBuffer));

        if(recvSize!=-1 && recvBuffer[1]==1)
        {
            time = omp_get_wtime()-time0;

            cout << setw(5) << setprecision(2) << recvBuffer[0] << " | "<<setw(5)<<setprecision(2)<<recvBuffer[1]<<" | ";
            for(int i = 2; i<recvSize; i++)
                cout<<setw(5)<<setprecision(2)<<recvBuffer[i]<<" ";

            cout<<" | "<<setw(5)<<setprecision(2)<< time<<"\n";
        }

        usleep(5000);
    }

    return 0;
}

int test(sockaddr_in & si_other, int s, double* sendBuffer, int sendBufferSize, double* recvBuffer, int recvBufferSize)
{
    int slen;

    slen=sizeof(si_other);
    if(sendto(s, sendBuffer, sendBufferSize , 0 , (struct sockaddr *) &si_other, slen)==-1)
    {
        cout<<"Error sending, error code: "<<errno;
        return -1;
    }

    int recvSize = recvfrom(s, recvBuffer, recvBufferSize, 0, (struct sockaddr*)&si_other, (socklen_t*)&slen);
    if(recvSize==-1)
    {
        cout<<"Error receiving, error code: "<<errno;
        return -1;
    }

    recvSize/=sizeof(double);

    return recvSize;
}