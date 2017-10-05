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

struct egse2dm_header_t {
	uint32_t payload_len;
	uint32_t crc;
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

int32_t can_data_send_scatter(int fd, uint8_t *payload, uint32_t data_len)
{
	uint32_t sent_len = 0, cur_len = 0, nbytes = 0, i = 0;
	uint32_t sent_len_max = CAN_BUFFER_SIZE;
	struct can_frame frame;
	int ret;

	frame.can_id  = 0x123;
	frame.can_dlc = 8;
	while (1) {
		sent_len = (data_len - cur_len) >=  sent_len_max ?
				   sent_len_max : (data_len - cur_len);
		if (sent_len > 0) {
			memset(&frame.data, 0, sent_len_max);
			for (i = 0; i < sent_len; i++, payload++) {
				frame.data[i] = *payload;
			}
			ret = write(fd, &frame, sizeof(struct can_frame));
			if (ret < 0)
				goto error;
			cur_len += sent_len;
			nbytes += ret;
		} else {
			ret = nbytes;
			break;
		}
	}
error:
	return ret;
}

uint32_t crc32(uint32_t crc, const char *buf, uint32_t len)
{
	uint32_t	crc_30_00;
	uint32_t	crc_31;
	uint32_t	buf_value;
	
        if (buf == 0) return 0L;

	buf_value =  *((uint32_t *) buf);
	crc_30_00 = ((crc >> 1) ^ (buf_value & 0x7fffffff)); 
	crc_31	  = (crc & 0x1) ^ 
				((crc & (0x1 << 4)) >> 4) ^ 
				((crc & (0x1 << 8)) >> 8) ^ 
				((crc & (0x1 << 12)) >> 12) ^ 
				((crc & (0x1 << 17)) >> 17) ^ 
				((crc & (0x1 << 20)) >> 20) ^ 
				((crc & (0x1 << 23)) >> 23) ^ 
				((crc & (0x1 << 28)) >> 28) ^ 
				((buf_value & (0x1 << 31)) >> 31);
				
	return	( ( (crc_31 << 31) & 0x80000000 ) | (crc_30_00 & 0x7fffffff ));				
}

int main(int argc,char **argv)
{
	int can_socket;
	int nbytes = 0;
	struct sockaddr_can addr;
	struct ifreq ifr;
	char *ifname;
	struct egse2dm egse_cmd;
	uint8_t *p_egse_cmd;
	uint32_t data_len, crc;
	uint8_t *tx_buffer;
	uint32_t buf_offset = 0;
	uint32_t egse2dm_full_size;

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

	egse2dm_cmd_init((struct egse2dm *)p_egse_cmd);
	data_len = sizeof(struct egse2dm);
	/* start to do crc procedure */
	crc = nbytes = 0;
        while (nbytes < data_len)
        {
        	crc = crc32(crc, (char *)(p_egse_cmd + nbytes), 4);
        	nbytes += 4;
        }
	printf("TX CAN CRC without header: 0x%x\n", crc);

	egse2dm_full_size = sizeof(struct egse2dm_header_t) + sizeof(struct egse2dm);
	tx_buffer = calloc(1, egse2dm_full_size);

	buf_offset = 0;
	memcpy(tx_buffer + buf_offset, &data_len, 4);
	buf_offset += 4;
	memcpy(tx_buffer + buf_offset, &crc, 4);
	buf_offset += 4;
	memcpy(tx_buffer + buf_offset, p_egse_cmd, sizeof(struct egse2dm));

	printf("Using %s send egse2dm with header size: %d\n", ifname, egse2dm_full_size);
	nbytes = can_data_send_scatter(can_socket, tx_buffer, egse2dm_full_size);
	printf("[%lf] TX CAN total %d bytes has been send. \n", get_curr_time(), nbytes);
	free(tx_buffer);
	return 0;
}
