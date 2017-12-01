#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/input.h>
#define NOKEY 0

int main()
{
	int keys_fd;	
        char ret[2]; 
	struct input_event t;
	char *dev;

        setvbuf(stdout, (char *)NULL, _IONBF, 0);//disable stdio out buffer;
	dev = getenv("KEYPAD_DEV");
      
        keys_fd = open(dev, O_RDONLY);
	if(keys_fd<=0)
	{
                printf("open %s device error!\n",dev);
		return 0;
	}

	while(1)
	{	
                if(read(keys_fd,&t,sizeof(t))==sizeof(t)) {
		    if(t.type==EV_KEY)
			if(t.value==0 || t.value==1)
			{
			//	printf("%d \n", t.code);
				switch(t.code)
				{
					case 256:
			    		printf("key256 %s\n",(t.value)?"Released":"Pressed");
			    	break;
			    	
			    	case 257:
			    		printf("key257 %s\n",(t.value)?"Released":"Pressed");
			    	break;
			    	
			    	case 258:
			    		printf("key258 %s\n",(t.value)?"Released":"Pressed");
			    	break;
			    	
			    	case 259:
			    		printf("key259 %s\n",(t.value)?"Released":"Pressed");
			    	break;
			    	
			    	case 260:
			    		printf("key260 %s\n",(t.value)?"Released":"Pressed");
			    	break;

					case 261:
						printf("key261 %s\n",(t.value)?"Released":"Pressed");
					break;
			    	
			    	default:
			    		break;
			    }
			}
		}
	}	
	close(keys_fd);
	
        return 0;
}

