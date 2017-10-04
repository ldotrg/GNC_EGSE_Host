#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include "stdint.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>

#define BILLION 			1000000000L
#define INLINE                        	__attribute__((always_inline))
#define CAN_BUFFER_SIZE 8

void clock_get_hw_time(struct timespec *ts)
{
  clock_gettime(CLOCK_MONOTONIC, ts);
}

static struct timespec ts;
double get_curr_time(void) 
{
    clock_get_hw_time(&ts);
    return ts.tv_sec + (double)ts.tv_nsec/(double)BILLION;
}

#define TVC_SIZE 6
#define RCS_SIZE 4
#define II_VALUE_CONTROL_SIZE 3
#define ORDANCE_SIZE 3
/* 
*
* EGSE->DM 20pps, RX cmd format from devices. (egse data verify server)
*
*/
struct egse2dm {
	uint8_t III_TVC_1[TVC_SIZE];
	uint8_t III_TVC_2[TVC_SIZE];
	uint8_t III_valve_control_1[II_VALUE_CONTROL_SIZE];
	uint8_t III_valve_control_2[II_VALUE_CONTROL_SIZE];
	uint8_t RCS[RCS_SIZE];
	uint8_t ordance_faring[ORDANCE_SIZE];
	uint8_t ordance_separation[ORDANCE_SIZE];
	uint8_t II_TVC_1[TVC_SIZE];
	uint8_t II_TVC_2[TVC_SIZE];
	uint8_t II_valve_control_1[II_VALUE_CONTROL_SIZE];
	uint8_t II_valve_control_2[II_VALUE_CONTROL_SIZE];
} __attribute__((packed));

int egse2dm_cmd_init(struct egse2dm *cmd)
{
	uint32_t i;
	for (i = 0; i < II_VALUE_CONTROL_SIZE; ++i)
	{
		cmd->III_valve_control_1[i] = 0x33U;
		cmd->III_valve_control_2[i] = 0x44U;
		cmd->II_valve_control_1[i] = 0xaaU;
		cmd->II_valve_control_2[i] = 0xbbU;
	}

	for (i = 0; i < TVC_SIZE; ++i)
	{
		cmd->III_TVC_1[i] = 0x11U;
		cmd->III_TVC_2[i] = 0x22U;
		cmd->II_TVC_1[i] = 0x88U;
		cmd->II_TVC_2[i] = 0x99U;
	}

	for (i = 0; i < RCS_SIZE; ++i)
	{
		cmd->RCS[i] = 0x55U;
	}

	for (i = 0; i < ORDANCE_SIZE; ++i)
	{
		cmd->ordance_faring[i] = 0x66U;
		cmd->ordance_separation[i] = 0x77U;
	}
	return 0;
}



int main(int argc,char **argv)
{
	int can_socket, i;
	int nbytes = 0;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;
	char *ifname;
	struct egse2dm egse_cmd;
	uint8_t *p_egse_cmd;
	uint32_t sent_len_max = CAN_BUFFER_SIZE;
	uint32_t data_len, cur_len = 0;

	p_egse_cmd = (uint8_t *)&egse_cmd;


	if (argc != 2)
	{
		printf("usage : sendCAN <CAN interface>\n");
		argv[1] = "can1";
		printf("Use sendCan can1\n");
	}

	ifname = argv[1];
 
	if((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(can_socket, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex; 

	if(bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
	{
		perror("Error in socket bind");
		return -2;
	}

	egse2dm_cmd_init(&egse_cmd);
	frame.can_id  = 0x123;
	frame.can_dlc = 8;
	data_len = sizeof(struct egse2dm);
	printf("[dungru:%d:%s] egse2dm cmd size: %d\n", __LINE__, __FUNCTION__, data_len);
	while (1) {
		uint32_t sent_len;
		sent_len = (data_len - cur_len) >=  sent_len_max ?
				   sent_len_max : (data_len - cur_len);
		if (sent_len > 0) {
			memset(&frame.data, 0, sent_len_max);
			for (i = 0; i < sent_len; i++, p_egse_cmd++) {
				frame.data[i] = *p_egse_cmd;
			}
			nbytes += write(can_socket, &frame, sizeof(struct can_frame));
			if (nbytes < 0)
				goto error;

			cur_len += sent_len;
		} else
			break;
	}

	printf("using %s to write %d bytes\n", ifname, nbytes);
	printf("Before send to kernel space %lf\n", get_curr_time());
error:
	return 0;
}
