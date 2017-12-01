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

class CanSocket
{
    public:


        CanSocket()
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
        }

        ~CanSocket()
        {
            system(down);
        }
};
int main()
{
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
                sendf.can_dlc = strlen(sendf.data);
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
                        printf("%c", recvf.data[i]);
                    printf("\n");
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

    return 0;
}
