#include <stdio.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> /* File control definitions */
#include <errno.h>
#include <termios.h> /* POSIX terminal control definitions */


#include "main.h"

int open_port(void)
{
	int fd; 

	fd = open("/dev/rfcomm0", O_RDWR | O_NOCTTY);
	if (fd <= 0) {
		perror("open_port: Unable to open /dev/rfcomm0 -");
		return -1;
	}

	int flags = 0;
	flags = fcntl(fd,F_GETFL,0);
	flags &= ~O_NONBLOCK;//设置成阻塞模式
	fcntl(fd,F_SETFL,flags);

	struct termios options;  
	tcgetattr(fd, &options); 

	options.c_oflag = 0; 
	options.c_iflag = 0; 
	options.c_cflag = 0;
	options.c_lflag = 0;

	options.c_iflag |= IGNPAR;//无奇偶检验位  

	options.c_cflag |= CLOCAL|CREAD;//设置控制模式状态，本地连接，接收使能  
	options.c_cflag &= ~CSIZE;//字符长度，设置数据位之前一定要屏掉这个位  
	options.c_cflag |= CS8;//8位数据长度  	

	cfsetspeed(&options, B9600);//设置波特率  

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);  

	return fd;
}

#define BUFSIZE	256

int main(void)
{
	int fd = 0;
	unsigned char buff[BUFSIZE] = {0};
	int  n = 0;

	fd = open_port();
	if(fd < 0)
		return -1;

	while(1) {
		//n = read(fd,buff,BUFSIZE);
		//for(int i=0; i<n; i++)
		//	printf("%.2X ", buff[i]);
		n=read(fd,buff,1);
		if(n!=1) break;
		printf("%.2X ", buff[0]);
		fflush(stdout);
	}

	close(fd);

	return 0;
}


