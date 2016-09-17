#ifndef _MAIN_H_
#define _MAIN_H_

struct env_t{
	int env_lock;
	int bat_voltage;
	int bal_angle;
	int bal_kp;
	int bal_kd;
	int vel_kp;
	int vel_ki;
	int enc_filte;
	int mpu_count;
	int mpu_bal_angle;
	int mpu_bal_gypo;
	int mpu_turn_gypo;
	int enc_left;
	int enc_right;
	int moto_left;
	int moto_right;
	int cmd_forward;
	int cmd_back;
	int cmd_left;
	int cmd_right;
};

struct env_buff_t{
	unsigned short head;
	unsigned short len;
	struct env_t env;
	unsigned short alen;
	unsigned short sum;
};

#define ENVLEN	sizeof(struct env_buff_t)

extern struct env_t* ENV;

struct serial_buff_t{
	unsigned int index;
	unsigned char buff[2*ENVLEN];
};

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

struct cmd_buff_t{
	unsigned short head;
	unsigned short len;
	struct cmd_t cmd;
	unsigned short alen;
	unsigned short sum;
};

#define CMDLEN	sizeof(struct cmd_buff_t)

struct usart_buff_t{
	unsigned int  index;
	unsigned char buff[2*CMDLEN];
};

#endif

