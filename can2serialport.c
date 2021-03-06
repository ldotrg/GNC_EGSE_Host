#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h> 
#include <sched.h>
#include <pthread.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <stdint.h>
#include "rs422_serialport.h"
#include <arpa/inet.h>
#include "gpio_sync_timer/DIInterrupt.h"
#include <semaphore.h>

#define CFG_SINGLE_THREAD_DEBUG   0
#define CFG_DEBUG_THREAD_IDX      4
#define CFG_RS422_THREAD_ENABLE_MASK (1 << IMU01 |  \
				      1 << RATE_X | \
				      1 << RATE_Y | \
				      1 << RATE_Z | \
				      1 << IMU02  | \
				      1 << GPSR01 | \
				      1 << GPSR02) 

#define CFG_GPIO_ENABLE           0
#define CFG_HEADER_APPEND_ENABLE  1

#define RX_COUNTDOWN		(7)
#define CAN_RX_BUFF_SIZE        (RX_COUNTDOWN * 8)
#define USER_BAUD_RATE          (B921600)
#define IMU_SIZE (sizeof(struct IMU_filtered_data_t))
#define GPSR_SIZE (sizeof(struct NSPO_GPSR_SCI_TLM_t))
#define BILLION 			1000000000L
#define FTRACE_TIME_STAMP(id) do { syscall(id);} while (0)

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

struct thread_info_t rs422_tx_info[THREADS_NUM] = {
	/*thread_idx, thread_name,    port_name,     rs422_fd, payload_size, freq, go_flag, payload,           syscall_id, mutex,             cond,            core_id*/
	{IMU01 , "imu01",        "/dev/ttyAP0",  -1, IMU_SIZE,     10,    0, (void *) &imu_data,           501,        &imu01_mutex,      &imu01_cond,      2},
	{RATE_X , "rate_tbl_x",   "/dev/ttyAP1",  -1, 4,            50,    0, (void *) &(ratetable.rate.x), 502,        &rate_tbl_x_mutex, &rate_tbl_x_cond, 2},
	{RATE_Y , "rate_tbl_y",   "/dev/ttyAP2",  -1, 4,            50,    0, (void *) &(ratetable.rate.y), 503,        &rate_tbl_y_mutex, &rate_tbl_y_cond, 3},
	{RATE_Z , "rate_tbl_z",   "/dev/ttyAP7",  -1, 4,            50,    0, (void *) &(ratetable.rate.z), 504,        &rate_tbl_z_mutex, &rate_tbl_z_cond, 3},
	{IMU02 , "imu02",        "/dev/ttyAP4",  -1, IMU_SIZE,     10,    0, (void *) &imu_data,           505,        &imu02_mutex,      &imu02_cond,      4},
	{GPSR01 , "gpsr01",       "/dev/ttyAP5",  -1, GPSR_SIZE,    1,     0, (void *) &gpsr_data,          506,        &gpsr01_mutex,     &gpsr01_cond,     4},
	{GPSR02 , "gpsr02",       "/dev/ttyAP6",  -1, GPSR_SIZE,    1,     0, (void *) &gpsr_data,          507,        &gpsr02_mutex,     &gpsr02_cond,     5}
};

static struct timespec ts;
void clock_get_hw_time(struct timespec *ts)
{
	clock_gettime(CLOCK_MONOTONIC, ts);
}

double get_curr_time(void)
{
	clock_get_hw_time(&ts);
	return ts.tv_sec + (double)ts.tv_nsec/(double)BILLION;
}

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
	uint32_t *rate;
	struct IMU_filtered_data_t *imu_data;
	struct NSPO_GPSR_SCI_TLM_t *gpsr_data;


	info = (struct thread_info_t *) arg;
	// hex_dump("arg", (uint8_t *)arg, sizeof(struct thread_info_t));
	// hex_dump("info", (uint8_t *)info, sizeof(struct thread_info_t));
	printf("Thread name: %s\n", info->thread_name);
#if 1
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(info->core_id , &cpuset);
	pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
#endif
	info->rs422_fd  = open_port(info->port_name);
	if (info->rs422_fd  < 0) {
		fprintf(stderr, "open port %s error\n", info->port_name);
		exit(EXIT_FAILURE);
	}
	// Set the configureation of the port 
	set_interface_attribs(info->rs422_fd, USER_BAUD_RATE, 0);
	frame_full_size = info->payload_size + (CFG_HEADER_APPEND_ENABLE ? sizeof(struct data_frame_header_t) : 0);
	frame.payload_len = info->payload_size;
	/* paylod */
	switch(info->thread_idx) {
		case IMU01:
			imu_data = info->payload;
			break;
		case RATE_X:
			rate = info->payload;
			*rate = (*rate + 1 ) % 360;
			break;
		case RATE_Y:
			rate = info->payload;
			*rate = (*rate + 1 ) % 360;
			break;
		case RATE_Z:
			rate = info->payload;
			*rate = (*rate + 1 ) % 360;
			break;
		case IMU02:
			imu_data = info->payload;
			break;
		case GPSR01:
			gpsr_data = info->payload;
			break;
		case GPSR02:
			gpsr_data = info->payload;
			break;
		default:
			fprintf(stderr, "There is no %s thread !!!!\n", info->thread_name);
			exit(EXIT_FAILURE);
	}

	/* start to do crc procedure */
	crc = total = 0;
        while (total < frame.payload_len)
        {
        	crc = crc32(crc, (char *)(info->payload + total), 4);
        	total += 4;
        }

	printf("  pure_payload CRC crc=0x%08x \n", crc);
	frame.crc = crc;
	frame.seq_no = 0;
	printf("[%s] frame_full_size = %d\n", info->thread_name,frame_full_size);
	//dump_thread_info(info);
	while (1) {
		pthread_mutex_lock(info->mutex);
		while (info->go_flag == 0) {
			pthread_cond_wait(info->cond, info->mutex);
		}
		info->go_flag = 0;
		pthread_mutex_unlock(info->mutex);
		int i = 0;
		tx_buffer = calloc(1, frame_full_size);
		for (i = 0; i < info->freq; i++)
		{
			header_offset = 0;
#if CFG_HEADER_APPEND_ENABLE
			memcpy(tx_buffer + header_offset, &frame.payload_len, 4);
			header_offset += 4;
			
			memcpy(tx_buffer + header_offset, &frame.crc, 4);
			header_offset += 4;

			frame.seq_no += 1;
			//fprintf(stderr,"[%s] seq_no: %d\n", info->thread_name,frame.seq_no);
			memcpy(tx_buffer + header_offset, &frame.seq_no, 4);
			header_offset += 4;
#endif	/* CFG_HEADER_APPEND_ENABLE */	
			memcpy(tx_buffer + header_offset, (uint8_t *)info->payload, frame.payload_len);

			buf_offset = 0;
			while(buf_offset < frame_full_size) {
				wdlen = write(info->rs422_fd, tx_buffer + buf_offset, frame_full_size - buf_offset);
				if (wdlen < 0)
					fprintf(stderr, "ERROR status %d\n", wdlen);
				buf_offset += wdlen;
			}
			//hex_dump("gpsr02", tx_buffer, frame_full_size);
		}
		free(tx_buffer);
		//FTRACE_TIME_STAMP(info->syscall_id);
	}
}

#if CFG_GPIO_ENABLE
void *gpio_ttl_thread(void *arg)
{

	int *ret = NULL;
	gpio_ttl_init_wrapper();
	return ret;

}
#endif /* CFG_GPIO_ENABLE */
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
	uint32_t rx_pktcnt = 0;
#if 1
	int core_id = 1;
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);       //clears the cpuset
	CPU_SET(core_id , &cpuset); //set CPU 1 on cpuset
	/*
	* cpu affinity for the calling thread
	* first parameter is the pid, 0 = calling thread
	* second parameter is the size of your cpuset
	* third param is the cpuset in which your thread will be
	* placed. Each bit represents a CPU
	*/
	pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
	//sched_setaffinity(0, sizeof(cpuset), &cpuset);
#endif
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
	/*Data Pattern init*/
	imu_pattern_init(&imu_data);
	rate_table_pattern_init(&ratetable);
	gpsr_pattern_init((void *)&gpsr_data);
	printf("RS422 Enable MASK: 0x%x",CFG_RS422_THREAD_ENABLE_MASK);
	for (idx = 0; idx < THREADS_NUM; idx++) {
		if (~((1 << idx) & CFG_RS422_THREAD_ENABLE_MASK))
			continue;
		printf("In main: creating thread %d\n", idx);
		ret = pthread_create(threads_id + idx, NULL, rs422_tx_downlink_thread, (void *) &rs422_tx_info[idx]);
		if (ret) {
			printf("ERROR; return code from pthread_create() is %d\n", ret);
			exit(EXIT_FAILURE);
		}

	}
#if (CFG_GPIO_ENABLE == 1)
	/* GPIO Create*/
	pthread_t gpio_thread_id;
	ret = pthread_create(&gpio_thread_id, NULL, gpio_ttl_thread, NULL);
	if (ret) {
		printf("ERROR; return code from gpio_thread_id is %d\n", ret);
		exit(EXIT_FAILURE);
	}
#endif /* CFG_GPIO_ENABLE */

	while(1)
	{
		rx_nbytes = read(socket_can, &frame, sizeof(frame));
		if (rx_nbytes > 0) {
#if 1
			if (rx_count == RX_COUNTDOWN )
				FTRACE_TIME_STAMP(511);
#endif
			rx_count--;
			if (frame.can_id & CAN_ERR_FLAG) {
				fprintf(stderr, "error frame\n");
				exit(EXIT_FAILURE);
			}
			for (idx= 0; idx < CAN_MAX_DLEN; ++idx) {
				rx_buff[buf_offset] = frame.data[idx];
				buf_offset++;
			}
			if (rx_count == 0) {
				//FTRACE_TIME_STAMP(511);
				egse2dm_cmd_header = (struct egse2dm_header_t *)rx_buff;
				if(crc_checker(egse2dm_cmd_header->crc, (char *)(rx_buff + sizeof(struct egse2dm_header_t)), 
						egse2dm_cmd_header->payload_len) == 0) {
					fprintf(stderr, "[%lf] CRC ERROR !!!!\n", get_curr_time());
					exit(EXIT_FAILURE);
				}
				rx_pktcnt++;
				//printf("[%lf:%d]CAN RX CRC PASS!! \n",get_curr_time() , rx_pktcnt);
				for (idx = 0; idx < THREADS_NUM; ++idx)
				{
					if (~((1 << idx) & CFG_RS422_THREAD_ENABLE_MASK))
						continue;
					pthread_mutex_lock(rs422_tx_info[idx].mutex);
					rs422_tx_info[idx].go_flag = 1;
					pthread_mutex_unlock(rs422_tx_info[idx].mutex);
				}
				rx_count = RX_COUNTDOWN;
				buf_offset = 0;
				//hex_dump("rx hex", rx_buff, CAN_RX_BUFF_SIZE);
				for (idx= 0; idx < THREADS_NUM; ++idx) {
					if (~((1 << idx) & CFG_RS422_THREAD_ENABLE_MASK))
						continue;
					pthread_cond_signal(rs422_tx_info[idx].cond);
				}
			}
		}
	}
	/* wait for all threads to complete */
	for (idx = 0; idx < THREADS_NUM; idx++) {
		if (~((1 << idx) & CFG_RS422_THREAD_ENABLE_MASK))
			continue;
		pthread_join(threads_id[idx], NULL);
	}
#if (CFG_GPIO_ENABLE == 1)
	pthread_join(gpio_thread_id, NULL);
#endif
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
		if (~((1 << idx) & CFG_RS422_THREAD_ENABLE_MASK))
			continue;
		close(rs422_tx_info[idx].rs422_fd);
	}
}