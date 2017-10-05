#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h> 

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <pthread.h>
#include <stdint.h>
#include "rs422_serialport.h"
#include <arpa/inet.h>
#include "gpio_sync_timer/DIInterrupt.h"

#define RX_COUNTDOWN		(7)
#define CAN_RX_BUFF_SIZE        (RX_COUNTDOWN * 8)
#define THREADS_NUM		(7)
#define USER_BAUD_RATE           (B921600)
#define IMU_SIZE (sizeof(struct IMU_filtered_data_t))
#define GPSR_SIZE (sizeof(struct NSPO_GPSR_SCI_TLM_t))

struct egse2dm_header_t {
	uint32_t payload_len;
	uint32_t crc;
} __attribute__((packed));

/*
As the mutex lock is stored in global (static) memory it can be 
    initialized with PTHREAD_MUTEX_INITIALIZER.
    If we had allocated space for the mutex on the heap, 
    then we would have used pthread_mutex_init(ptr, NULL)
*/
pthread_mutex_t     imu01_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t     imu02_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t     rate_tbl_x_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t     rate_tbl_y_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t     rate_tbl_z_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t     gpsr01_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t     gpsr02_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_cond_t      imu01_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t      imu02_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t      rate_tbl_x_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t      rate_tbl_y_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t      rate_tbl_z_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t      gpsr01_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t      gpsr02_cond = PTHREAD_COND_INITIALIZER;


struct IMU_filtered_data_t imu_data;
struct ProAxeSE_data_t ratetable;
struct NSPO_GPSR_SCI_TLM_t gpsr_data;

struct thread_info_t rs422_tx_info[THREADS_NUM] ={
	/*thread_name,    port_name,     rs422_fd, payload_size, freq, go_flag, payload,           mutex,             cond*/
	{"imu01",        "/dev/ttyAP0",  -1, IMU_SIZE,     10,    0, (void *) &imu_data,           &imu01_mutex,      &imu01_cond},
	{"rate_tbl_x",   "/dev/ttyAP1",  -1, 4,            50,    0, (void *) &(ratetable.rate.x), &rate_tbl_x_mutex, &rate_tbl_x_cond},
	{"rate_tbl_y",   "/dev/ttyAP2",  -1, 4,            50,    0, (void *) &(ratetable.rate.y), &rate_tbl_y_mutex, &rate_tbl_y_cond},
	{"rate_tbl_z",   "/dev/ttyAP3",  -1, 4,            50,    0, (void *) &(ratetable.rate.z), &rate_tbl_z_mutex, &rate_tbl_z_cond},
	{"imu02",        "/dev/ttyAP4",  -1, IMU_SIZE,     10,    0, (void *) &imu_data,           &imu02_mutex,      &imu02_cond},
	{"gpsr01",       "/dev/ttyAP5",  -1, GPSR_SIZE,    1,     0, (void *) &gpsr_data,          &gpsr01_mutex,     &gpsr01_cond},
	{"gpsr02",       "/dev/ttyAP6",  -1, GPSR_SIZE,    1,     0, (void *) &gpsr_data,          &gpsr02_mutex,     &gpsr02_cond}
};

void dump_thread_info(struct thread_info_t *info)
{
	printf("%s\n", info->thread_name);
	printf("%s\n", info->port_name);
	printf("fd: %d\n", info->rs422_fd);
	printf("size: %d\n", info->payload_size);
	printf("freq: %d\n", info->freq);
	printf("go_flag: %d\n", info->go_flag);
	hex_dump("data", info->payload, info->payload_size);
}


void *rs422_tx_downlink_thread(void *arg)
{
	struct thread_info_t *info = NULL;
	uint32_t frame_full_size;
	uint8_t *tx_buffer;
	uint32_t crc, total;
	uint32_t header_offset = 0;
	struct data_frame_header_t frame;
	uint32_t buf_offset;
	uint32_t wdlen;

	info = (struct thread_info_t *) arg;
	printf("Thread name: %s\n", info->thread_name);
	info->rs422_fd  = open_port(info->port_name);
	if (info->rs422_fd  < 0) {
		fprintf(stderr, "open port %s error\n", info->port_name);
		exit(EXIT_FAILURE);
	}
	// Set the configureation of the port 
	set_interface_attribs(info->rs422_fd, USER_BAUD_RATE, 0);
	frame_full_size = info->payload_size + sizeof(struct data_frame_header_t);
	frame.payload_len = info->payload_size;

	/* start to do crc procedure */
	crc = total = 0;
        while (total < frame.payload_len)
        {
        	crc = crc32(crc, (char *)(arg + total), 4);
        	total += 4;
        }

	printf("  Total_size_with_payload_crc_handle total=%u \n", total);
	printf("  pure_payload CRC crc=0x%08x \n", crc);
	frame.crc = crc;
	printf("%s: frame_full_size = %d\n", __FUNCTION__,frame_full_size);
	//dump_thread_info(info);
	while (1) {
		pthread_mutex_lock(info->mutex);
		while (~(info->go_flag & 0x1))
			pthread_cond_wait(info->cond, info->mutex);
		info->go_flag = 0;
		pthread_mutex_unlock(info->mutex);
		
		header_offset = 0;
		tx_buffer = calloc(1, frame_full_size);

		memcpy(tx_buffer + header_offset, &frame.payload_len, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, &frame.crc, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, arg, frame.payload_len);
		int i = 0;
		for (i = 0; i < info->freq; ++i)
		{
			buf_offset = 0;
			while(buf_offset < frame_full_size) {
				wdlen = write(info->rs422_fd, tx_buffer + buf_offset, frame_full_size - buf_offset);
				if (wdlen < 0)
					fprintf(stderr, "ERROR status %d\n", wdlen);
				buf_offset += wdlen;
			}
		}
		//hex_dump("gpsr02", tx_buffer, frame_full_size);
		free(tx_buffer);
	}
}


void *gpio_ttl_thread(void *arg)
{
	int *ret = NULL;
	gpio_ttl_init_wrapper();
	return ret;
}

int main(int argc,char **argv)
{
	int socket_can;
	int rx_nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;
	char ifname[] = "can0";
	int setflag = 0;
	int  status = 0;
	int rx_count = RX_COUNTDOWN;
	uint8_t rx_buff[CAN_RX_BUFF_SIZE];
	uint32_t buf_offset = 0;
	struct egse2dm_header_t *egse2dm_cmd_header;
	int32_t idx = 0, ret = 0;
	pthread_t threads_id[THREADS_NUM];
	pthread_t gpio_thread_id;
	if((socket_can = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}
	strcpy(ifr.ifr_name, ifname);
	ioctl(socket_can, SIOCGIFINDEX, &ifr);
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex; 

	if(bind(socket_can, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

	
	setflag = setflag | O_NONBLOCK;
	status = fcntl(socket_can, F_SETFL, setflag);
	fcntl(socket_can, F_GETFL, 0);

	can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT |CAN_ERR_BUSOFF;
	status = setsockopt(socket_can, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
	if (status != 0)
		printf("setsockopt fail\n");
	printf("using %s to read\n", ifname);

	/* GPIO Create*/
	ret = pthread_create(&gpio_thread_id, NULL, gpio_ttl_thread, NULL);
	if (ret) {
		printf("ERROR; return code from gpio_thread_id is %d\n", ret);
		exit(-1);
	}
	/*Data Pattern init*/
	imu_pattern_init(&imu_data);
	rate_table_pattern_init(&ratetable);
	gpsr_pattern_init((void *)&gpsr_data);
#if 1
	for (idx = 0; idx < THREADS_NUM; idx++) {
		printf("In main: creating thread %d\n", idx);
		ret = pthread_create(threads_id + idx, NULL, rs422_tx_downlink_thread, (void *) &rs422_tx_info[idx]);
		if (ret) {
			printf("ERROR; return code from pthread_create() is %d\n", ret);
			exit(-1);
		}
	}
#else /*debug purpose*/
	idx = 0;
	ret = pthread_create(threads_id + idx, NULL, rs422_tx_downlink_thread, (void *) &rs422_tx_info[idx]);
	if (ret) {
		printf("ERROR; return code from pthread_create() is %d\n", ret);
		exit(-1);
	}
#endif

	while(1)
	{
		struct timespec ts;
		rx_nbytes = read(socket_can, &frame, sizeof(frame));
		if (rx_nbytes > 0) {
			rx_count--;
			if (frame.can_id & CAN_ERR_FLAG) {
				fprintf(stderr, "error frame\n");
			}
			for (idx= 0; idx < CAN_MAX_DLEN; ++idx) {
				rx_buff[buf_offset] = frame.data[idx];
				buf_offset ++;
			}
			if (rx_count == 0) {
				clock_gettime(CLOCK_MONOTONIC, &ts);
				egse2dm_cmd_header = (struct egse2dm_header_t *)rx_buff;
				if(crc_checker(egse2dm_cmd_header->crc, (char *)(rx_buff + sizeof(struct egse2dm_header_t)), 
				               egse2dm_cmd_header->payload_len) == 0) {
					fprintf(stderr, "[%lld.%.9ld] CRC ERROR !!!!\n", (long long)ts.tv_sec, ts.tv_nsec);
					exit(EXIT_FAILURE);
				}
				printf("[%lld.%.9ld]CAN RX done and CRC PASS!! \n",(long long)ts.tv_sec, ts.tv_nsec);
				for (idx= 0; idx < THREADS_NUM; ++idx)
				{
					pthread_mutex_lock(rs422_tx_info[idx].mutex);
					rs422_tx_info[idx].go_flag = 1;
					pthread_mutex_unlock(rs422_tx_info[idx].mutex);
				}
				rx_count = RX_COUNTDOWN;
				buf_offset = 0;
				//hex_dump("rx hex", rx_buff, CAN_RX_BUFF_SIZE);
				for (idx= 0; idx < THREADS_NUM; ++idx)
				{
					pthread_cond_signal(rs422_tx_info[idx].cond);
				}
			}
		}
	}
	/* wait for all threads to complete */
	for (idx = 0; idx < THREADS_NUM; idx++) {
		pthread_join(threads_id[idx], NULL);
	}
	pthread_join(gpio_thread_id, NULL);

	return 0;
}

static __attribute__((constructor)) void init(void)
{

}

static __attribute__((destructor)) void finish(void)
{
	int idx;
	for (idx = 0; idx < THREADS_NUM; ++idx)
	{
		close(rs422_tx_info[idx].rs422_fd);
	}
}