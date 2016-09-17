#include <stdio.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> /* File control definitions */
#include <errno.h>
#include <termios.h> /* POSIX terminal control definitions */

#include "main.h"

#define BUFSIZE	256


struct serial_buff_t  serial_buff;
struct serial_buff_t* serial_buff_p = &serial_buff;

struct cmd_buff_t  cmd_buff;
struct cmd_buff_t* cmd_buff_p = &cmd_buff;


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

void serial_buff_reset(void)
{
	memset(serial_buff_p, 0, sizeof(struct serial_buff_t));
	serial_buff_p->index = ENVLEN-1;
}

int serial_proc(unsigned char * buff)
{
	struct env_buff_t *env_p = (struct env_buff_t *)buff;

	if(env_p->head != 0xff || env_p->len != ENVLEN || (env_p->alen & env_p->len))
		return -1;

	unsigned short sum = 0;
	for(int i = 0; i < ENVLEN - 2; i++)
		sum += buff[i];
	if(env_p->sum != sum){
		printf("env_proc check sum error !!!\r\n");
		return -1;
	}

/*
        int bat_voltage;
        int bal_angle;
        int bal_kp;
        int bal_kd;
        int vel_kp;
        int vel_ki;
        int enc_filte;
*/

	printf("bat_v %-4d bal_a %-4d bal_p %-4d bal_d %-4d vel_p %-4d vel_i %-4d enc_f %-4d mpu_c %-6d mbal_a %-6d mbal_g %-8d mbal_t %-8d enc_l %-3d enc_r %-3d mot_l %-6d mot_r %-6d\r\n",
		env_p->env.bat_voltage, env_p->env.bal_angle, env_p->env.bal_kp, env_p->env.bal_kd, env_p->env.vel_kp, env_p->env.vel_ki, env_p->env.enc_filte, 
		env_p->env.mpu_count,env_p->env.mpu_bal_angle,env_p->env.mpu_bal_gypo,env_p->env.mpu_turn_gypo,
		env_p->env.enc_left,env_p->env.enc_right,env_p->env.moto_left,env_p->env.moto_right
	);
	fflush(stdout);

	serial_buff_reset();

	return 0;
}

void cmd_buff_reset(void)
{
	memset(cmd_buff_p, 0, sizeof(struct cmd_buff_t));
}

/*
struct cmd_t{
        int bal_angle;
        int bal_kp;
        int bal_kd;
        int vel_kp;
        int vel_ki;
        int enc_filte;
        int turn_kp;
        int turn_ki;
        int turn_cmd;
};
*/

void set_cmd(void)
{
	cmd_buff_p->cmd.bal_angle = 1;
	cmd_buff_p->cmd.bal_kp	  = 301;
	cmd_buff_p->cmd.bal_kd	  = 1001;
	cmd_buff_p->cmd.vel_kp	  = 81;
	cmd_buff_p->cmd.vel_ki	  = 401;
	cmd_buff_p->cmd.enc_filte = 801;
	cmd_buff_p->cmd.turn_kp	  = 0;
	cmd_buff_p->cmd.turn_ki	  = 0;
	cmd_buff_p->cmd.turn_cmd  = 0;
}

void make_cmd(void)
{
	set_cmd();

	cmd_buff_p->head = 0xff;
	cmd_buff_p->len  = CMDLEN;
	cmd_buff_p->alen = ~cmd_buff_p->len;

	unsigned char* buff = (unsigned char *)cmd_buff_p;
	unsigned short sum  = 0;
	for(int i=0;i<CMDLEN-2;i++)
		sum += buff[i];
	cmd_buff_p->sum = sum;
}

int write_cmd(int fd)
{
	unsigned char* buff = (unsigned char *)cmd_buff_p;
	int ret = 0;

	make_cmd();

	ret = write(fd,buff,CMDLEN);

	if(ret == CMDLEN){
		return 0;
	}
	else{
		perror("write cmd to serial");
		return -1;
	}
}

int main(void)
{
	int fd = 0;
	unsigned char buff[1] = {0};

	cmd_buff_reset();
	serial_buff_reset();

	fd = open_port();
	if(fd < 0)
		return -1;

	if(write_cmd(fd))
		return -1;

	sleep(1);

	if(write_cmd(fd))
		return -1;

	while(1) {
		if(read(fd,buff,1)!=1)	break;

		//printf("%.2X ", buff[0]);
		//fflush(stdout);

		serial_buff_p->index += 1;
		serial_buff_p->index %= ENVLEN;
		serial_buff_p->buff[serial_buff_p->index] = buff[0];
		serial_buff_p->buff[serial_buff_p->index+ENVLEN] = buff[0];
		serial_proc(&(serial_buff_p->buff[serial_buff_p->index+1]));
	}

	close(fd);

	return 0;
}


