#include <string>
#include <iostream>
#include "tcpconnector.h"

//#include <unistd.h>
//#include <pthread.h>


#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include<string.h>
#include<stdlib.h>
#include<stdio.h>

#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

#define command "ip link set can0 type can bitrate 500000 triple-sampling on"
#define up  "ip link set can0 up"
#define down "ip link set can0 down"


//////////////////////////////////////////////////////
void sendRequest(int argc, char *argv[])
{
    // 需要去除命令名的路径和扩展名 //
    std::string cmdName(argv[0]);

    if (cmdName.rfind('/'))
    {
        cmdName = cmdName.substr(cmdName.rfind('/') + 1, cmdName.npos);
    }

    if (cmdName.rfind('.'))
    {
        cmdName = cmdName.substr(0, cmdName.rfind('.'));
    }

    // 添加命令的所有参数 //
    for (int i = 1; i < argc; ++i)
    {
        cmdName = cmdName + " " + argv[i];
    }

    TCPConnector* connector = new TCPConnector();
    TCPStream* stream=NULL;
    do
    {
        try
        {
//            stream = connector->connect("127.0.0.1", 5866);
//            stream = connector->connect("192.168.3.99", 5866);
            stream = connector->connect("192.168.1.100", 5866);
        }
        catch (std::exception &)
        {
            std::cout << "failed to connect server, will retry in 1 second" << std::endl;
            sleep(5);
        }
    }while (!stream);

    stream->send(cmdName.c_str(), cmdName.size());
    std::cout<<"sent - "<<cmdName<<std::endl;

    int len;
    char line[256];
    len = stream->receive(line, sizeof(line));
    line[len] = 0;
    std::cout<<"received - "<<line<<std::endl;

    delete stream;
}

int s;
unsigned  int nbytes;
struct sockaddr_can addr;
struct ifreq ifr;
struct can_frame sendf, recvf;
int flag_num = 0;
char message[1024] = {0};
unsigned int len;
int ret = 0,flag = 0;
int i = 0;
int canInit(void)
{
    system(down);
    system(command);
    usleep(50);
    system(up);
    usleep(50);


    s = socket(PF_CAN,SOCK_RAW,CAN_RAW);
    strcpy((char *)(ifr.ifr_name),"can0");
    ioctl(s,SIOCGIFINDEX,&ifr);
    printf("can0 can_ifindex = %x\n",ifr.ifr_ifindex);


    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s,(struct sockaddr*)&addr,sizeof(addr));
    if(ret < 0)
    {
        printf("in bind error \n");
        exit(1);
    }
    return 0;
}



char *pparg[10];

char *start[] =
{
    "start",
    "en",
    "hm",
    "rc",
    "am",
    "st"
};



int main()
{
    canInit();
    while(1)
    {
        printf("Select 1 : send can frame\n");
        printf("Select 2 : receive can  frame\n");
        printf("Select 3 : terminate this program\n");
        printf("please input:");
        scanf("%d",&flag_num);
        getchar();
        switch(flag_num)
        {
            case 1:
                memset(&sendf, 0, sizeof(struct can_frame));
                sendf.can_id = 0x123;
                memset(message,'\0',sizeof(message));
                printf("Please input data not exceeding 8 characters:");
                scanf("%[^\n]",message);
                strcpy((char *)sendf.data,message);
                printf("frame.data = %s\n",sendf.data);
                sendf.can_dlc = strlen((char *)sendf.data);
                nbytes = sendto(s,&sendf,sizeof(struct can_frame),0,(struct sockaddr*)&addr,sizeof(addr));
                printf("Send a CAN frame from interface %d num %d\n",ifr.ifr_ifindex,nbytes);
                break;
            case 2:
                while(1)
                {
                    memset(&recvf, 0, sizeof(struct can_frame));
                    nbytes = recvfrom(s,&recvf,sizeof(struct can_frame),0,(struct sockaddr *)&addr,&len);
                    printf("receive a can frame from interface %d\n",addr.can_ifindex);
                    printf(	"--can_id = %x\n"
                        "--can_dle =%x\n"
                        "--data=",recvf.can_id,recvf.can_dlc);
                    for (i = 0; i < recvf.can_dlc; ++i)
                        printf("0x%x   ", recvf.data[i]);
                    printf("\n");
                    pparg[0] = start[recvf.data[0]];
                    sendRequest(1, pparg);
                }
                break;
            case 3:
                flag = 1;
                break;
            default:
                printf("\nplease select correct number\n");
        }
        if (flag == 1)
            break;
    }
    system(down);
    return 0;
}

