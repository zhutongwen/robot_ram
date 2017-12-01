#include<sys/types.h>
#include<fcntl.h>
#include<sys/stat.h>
#include<unistd.h>
#include<stdio.h>
#include<string.h>
#include<errno.h>
#include<stdlib.h>
#define BUF_SIZE	 256

/* ./eeprom -w  forlinx
   ./eeprom -r  */

int main(int argc, char **argv)
{
	int ret = 0;
	char arr[BUF_SIZE] = {'0'};
	char buf[BUF_SIZE] = {'0'};
	
	int fd = open("/dev/eeprom", O_RDWR);
	if (fd < 0) {
		perror("open eeprom device:");
		exit(1);
	}
	
	if(argc < 2)
	{
		printf("please input correct argument\n");
		exit(1);
	}	

	if(!strcmp(argv[1],"-w"))
	{
		if(argc != 3)
		{
			printf("please input like this : ./eeprom -w  forlinx\n");
			exit(1);
		}
		strcpy(arr,argv[2]);

		ret = write(fd,arr,strlen(arr));
		if(ret < 0)
		{
			fprintf(stderr,"error eeprom_write: %s\n",strerror(errno));
			exit(1);
		}
//		printf("write ret = %d\n",ret);

//		ret = lseek(fd,30,SEEK_SET);
//		ret = write(fd,arr,strlen(arr) + 1);
//		if(ret < 0)
//		{
//			fprintf(stderr,"error eeprom_write: %s\n",strerror(errno));
//			exit(1);
//		}
//		printf("write ret = %d\n",ret);
	}
	if(!strcmp(argv[1],"-r"))
	{
		if(argc != 2)
		{
			printf("please input like this : ./eeprom -r\n");
			exit(1);
		}
		ret =  read(fd,buf,BUF_SIZE);
		if(ret < 0)
		{
			fprintf(stderr,"error eeprom_read: %s\n",strerror(errno));
			exit(1);
		}
		printf("the value in  buf is  %s\n",buf);

		ret = lseek(fd,6,SEEK_SET);	
		ret =  read(fd,buf,BUF_SIZE);
		if(ret < 0)
		{
			fprintf(stderr,"error eeprom_read: %s\n",strerror(errno));
			exit(1);
		}
		printf("lseek to the seventh bytes in buf is %s\n",buf);
	}
	close(fd);

	return 0;
	

}
