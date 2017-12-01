#include <stdio.h>
#include <stdlib.h>	//system
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>

/*tone freqency*/
int low[]= {0, 262,294,330,349,392,440,494};
int mid[]= {0, 523,578,659,698,784,880,988};
int hig[]= {0, 1046,1175,1318,1397,1568,1760,1976};

/*tone1, happy birthday to you*/
char* music1[]={
	"m5","m5","h5","h3","h1","m7","m6","m6","0","0","h4","h4","h3","h1","h2","h1","h1","m5","m5","m6","m5","h1","m7","m7","m5","m5",
	"m6","m5","h2","h1","h1","m5","m5","h5","h3","h1","m7","m6","m6","0","0","h4","h4","h3","h1","h2","h1","h1","m5","m5","m6","m5","h1","m7","m7","m5","m5",
	"m6","m5","h2","h1","h1","m5","m5","h5","h3","h1","m7","m6","m6","0","0","h4","h4","h3","h1","h2",
	"h1","h1","m5","m5","h5","h3","h1","m7","m6","m6","0","0","h4","h4","h3","h1","h2","h1","h1","h1","-1"
};

/*tone2, wen wen de xing fu*/
char* music2[]={
	"0","l5","l6","m3","0","0","m3","m5","m3","m3","m2","m2","m1","m2","m3","m2","l6","l6","0","0","l6","l6","m1","m4","m4","m4","m4","m5","m3","m2","m2","0","m2","m2","m1",
	"m2","m3","m3","m2","m3","m3","0","l5","l5","m5","0","0","m5","m5","m5","m5","m5","m5","m3","l6","m3","m2","m1","m1","0","0","m2","m2","m1","m2","m2","m2","m2","m2","l6","m2","m2","l6","l6",
	"m1","m2","m1","m1","m6","h1","h2","m6","h2","h1","m6","0","0","m5","h2","h1","h2","h2","h2","h1","m5","m5","0","0","m5","h2","h3","h2","h1","m6","0","0","h2","h2","h3","h5",
	"h5","h3","h3","h3","h3","m6","h1","h2","m6","h2","h1","m6","m6","h2","h2","h2","h1","h3","h5","h3","h3","0","m6","h1","h2","m6","h2","h3","h2","0","0","h1","h1","m6","m6",
	"h2","h1","h1","h1","h1","0","m5","m6","h1","h1","m6","h2","m6","m6","0","m5","m6","h1","h1","m5","h3","h3","0","m5","m6","h1","h1","m6","h2","h2","m5","h2","h5",
	"h3","h3","h3","0","l5","l6","m1","m1","l6","m2","m2","0","m2","m3","m4","m4","m3","m3","m2","m1","0","m1","m2","m3","m3","m2","m1","l6","0","h1","h2","h3",
	"h3","h3","h2","0","l5","l6",
	"m3","0","0","m3","m5","m3","m3","m2","m2","m1","m2","m3","m2","l6","l6","0","0","l6","l6","m1","m4","m4","m4","m4","m5","m3","m2","m2","0","m2","m2","m1",
	"m2","m3","m3","m2","m3","m3","0","l5","l5","m5","0","0","m5","m5","m5","m5","m5","m5","m3","l6","m3","m2","m1","m1","0","0","m2","m2","m1","m2","m2","m2","m2","m2","l6","m2","m2","l6","l6",
	"m1","m2","m1","m1","m6","h1","h2","m6","h2","h1","m6","0","0","m5","h2","h1","h2","h2","h2","h1","m5","m5","0","0","m5","h2","h3","h2","h1","m6","0","0","h2","h2","h3","h5",
	"h5","h3","h3","h3","h3","m6","h1","h2","m6","h2","h1","m6","m6","h2","h2","h2","h1","h3","h5","h3","h3","0","m6","h1","h2","m6","h2","h3","h2","0","0","h1","h1","m6","m6",
	"h2","h1","h1","h1","h1","m6","h1",
	"h2","m6","h2","h1","m6","0","0","m5","h2","h1","h2","h2","h2","h1","m5","m5","0","0","m5","h2","h3","h2","h1","m6","0","0","h2","h2","h3","h5",
	"h5","h3","h3","h3","h3","m6","h1","h2","m6","h2","h1","m6","m6","h2","h2","h3","h2","h3","h5","h3","h3","0","m6","h1","h2","h3","h2","h1","m6","0","0","h1","h1","m6","m6",
	"h2","h1","h1","h1","h1","m6","h1","h2","h1","h2","h1","m6","m6","0","0","0","0","m6","h1",
	"h2","h2","h3","h2","h1","m6","h1","h1","h1","h1","-1"
};

/*length,16 means 1/16,8 means 1/8,4 means 1/4*/
int length1[]={
	8,8,4,4,4,4,4,4,4,4,8,8,4,4,4,4,4,8,8,4,4,4,4,4,8,8,
	4,4,4,4,4,8,8,4,4,4,4,4,4,4,4,8,8,4,4,4,4,4,8,8,4,4,4,4,4,8,8,
	4,4,4,4,4,8,8,4,4,4,4,4,4,4,4,8,8,4,4,4,
	4,4,8,8,4,4,4,4,4,4,4,4,8,8,4,4,4,4,4,4,-1	
};
int length2[]={
	8,16,16,4,8,16,16,16,16,16,16,16,16,16,16,8,8,4,4,16,16,16,16,8,8,16,8,16,16,8,16,16,16,16,16,
	8,8,16,8,16,4,8,16,16,4,8,16,16,16,16,16,16,16,16,8,16,8,16,4,4,16,16,16,16,8,8,8,16,16,16,8,16,8,8,
	8,8,4,4,8,8,8,8,16,8,16,8,16,16,8,8,8,8,16,8,16,4,8,16,16,8,8,16,8,16,8,16,16,8,16,16,
	16,8,16,4,4,8,8,8,8,16,8,16,4,8,8,8,8,16,8,16,4,8,16,16,8,8,16,8,16,8,16,16,16,8,16,
	16,8,16,4,4,16,16,16,16,8,8,8,8,4,16,16,16,16,8,8,4,4,16,16,16,16,8,8,4,8,8,8,8,
	4,4,4,16,16,16,16,8,8,4,4,16,16,16,16,8,8,8,8,4,16,16,16,16,8,8,4,4,16,16,16,16,
	4,4,4,8,16,16,
	4,8,16,16,16,16,16,16,16,16,16,16,8,8,4,4,16,16,16,16,8,8,16,8,16,16,8,16,16,16,16,16,
	8,8,16,8,16,4,8,16,16,4,8,16,16,16,16,16,16,16,16,8,16,8,16,4,4,16,16,16,16,8,8,8,16,16,16,8,16,8,8,
	8,8,4,4,8,8,8,8,16,8,16,8,16,16,8,8,8,8,16,8,16,4,8,16,16,8,8,16,8,16,8,16,16,8,16,16,
	16,8,16,4,4,8,8,8,8,16,8,16,4,8,8,8,8,16,8,16,4,8,16,16,8,8,16,8,16,8,16,16,16,8,16,
	16,8,16,4,4,8,8,
	8,8,16,8,16,8,16,16,8,8,8,8,16,8,16,4,8,16,16,8,8,16,8,16,8,16,16,8,16,16,
	16,8,16,4,4,8,8,8,8,16,8,16,4,8,8,8,8,16,8,16,4,8,16,16,8,8,16,8,16,8,16,16,16,8,16,
	16,8,16,4,4,8,8,8,8,8,8,4,4,4,4,4,8,16,16,
	4,8,8,8,4,8,4,4,4,4,-1
	
};

#define DEV_NAME "/dev/buzzer"

int main(int argc, char *argv[])
{
	int fd;
	int i = 0;
	int *which = NULL;
	char **music = music1;
	int *length = length1;

	fd=open(DEV_NAME, O_RDWR);
	if(fd<0) {
		perror("can not open device");
		exit(1);
	}


	while(1)
	{
	 	if(music[i][0] == '-')i=0;	
		/*skip 0 pause*/
		if(music[i][0] != '0')
		{
			if(music[i][0] == 'l')
			{
				which = low;
			}
			else if(music[i][0] == 'm')
			{
				which = mid;
			}
			else if(music[i][0] == 'h')
			{
				which = hig;
			}
			ioctl(fd, 1, which[music[i][1] - '0']);
	 		usleep(3200000/length[i]);
		}
		i++;
	}

	close(fd);
	return 0;
}