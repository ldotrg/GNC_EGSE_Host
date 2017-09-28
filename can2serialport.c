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

#define RX_COUNTDOWN		(6)
#define THREADS_NUM		(7)
#define IMU_FREQ_PER_RX_CAN      (10)
#define RATE_TBL_FREQ_PER_RX_CAN (50)
#define USER_BAUD_RATE           (B921600)
#if 0
#define RATE_TABLE_TXDELAY
#define IMU_TXDELAY
#else
#define RATE_TABLE_TXDELAY do { \
			usleep(400); \
		} while (0)
#define IMU_TXDELAY do { \
			usleep(1500); \
		} while (0)
#endif
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

uint8_t imu01_go = 0;
uint8_t imu02_go = 0;
uint8_t rate_tbl_x_go = 0;
uint8_t rate_tbl_y_go = 0;
uint8_t rate_tbl_z_go = 0;
uint8_t gpsr01_go = 0;
uint8_t gpsr02_go = 0;

int imu01_rs422_fd;
int imu02_rs422_fd;
int rate_tbl_x_rs422_fd;
int rate_tbl_y_rs422_fd;
int rate_tbl_z_rs422_fd;
int gpsr01_rs422_fd;
int gpsr02_rs422_fd;

struct IMU_filtered_data_t imu_data;
struct ProAxeSE_data_t ratetable;
struct NSPO_GPSR_SCI_TLM_t gpsr_data;

static void clean_up_handler (void *arg)
{
	pthread_mutex_t *mtx;
	mtx = (pthread_mutex_t *) arg;
	pthread_mutex_unlock(mtx);
}

void *imu01_thread(void *arg)
{
	uint32_t frame_full_size;
	int  status = 0;
	uint8_t *tx_buffer;
	uint32_t crc, total;
	uint32_t header_offset = 0;
	struct data_frame_header_t frame;

	imu01_rs422_fd = open_port(SERIAL_PORT(0));
	if (imu01_rs422_fd < 0) {
		printf("open_port error!!\n");
		exit(EXIT_FAILURE);
	}
	// Set the configureation of the port 
	set_interface_attribs(imu01_rs422_fd, USER_BAUD_RATE, 0);

	frame_full_size = sizeof(struct IMU_filtered_data_t) + sizeof(struct data_frame_header_t);
	frame.payload_len = frame_full_size - sizeof(struct data_frame_header_t);

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
	printf("%s: frame_full_size = %d\n", __FUNCTION__, frame_full_size);

	while (1) {
		pthread_mutex_lock(&imu01_mutex);
		while (!imu01_go)
			pthread_cond_wait (&imu01_cond, &imu01_mutex);
		imu01_go = 0;
		pthread_mutex_unlock(&imu01_mutex);
		
		header_offset = 0;
		tx_buffer = calloc(1,frame_full_size);

		memcpy(tx_buffer + header_offset, &frame.payload_len, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, &frame.crc, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, arg, frame.payload_len);
		int i = 0;
		for (i = 0; i < IMU_FREQ_PER_RX_CAN; ++i)
		{
			status = write(imu01_rs422_fd, tx_buffer, frame_full_size);
			IMU_TXDELAY;
		}
		if (status < 0)
			printf("ERROR status %d\n", status);
		//hex_dump("imu01", tx_buffer, frame_full_size);
		free(tx_buffer);
	}
}

void *rate_table_x_thread(void *arg)
{
	
	uint32_t frame_full_size;
	int  status = 0;
	uint8_t *tx_buffer;
	uint32_t crc, total;
	uint32_t header_offset = 0;
	struct data_frame_header_t frame;

	rate_tbl_x_rs422_fd = open_port(SERIAL_PORT(1));
	if (rate_tbl_x_rs422_fd < 0) {
		printf("open_port error!!\n");
		exit(EXIT_FAILURE);
	}
	// Set the configureation of the port 
	set_interface_attribs(rate_tbl_x_rs422_fd, USER_BAUD_RATE, 0);

	frame_full_size = sizeof(int32_t) + sizeof(struct data_frame_header_t);
	frame.payload_len = frame_full_size - sizeof(struct data_frame_header_t);

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

	while (1) {
		pthread_mutex_lock(&rate_tbl_x_mutex);
		while (!rate_tbl_x_go)
			pthread_cond_wait (&rate_tbl_x_cond, &rate_tbl_x_mutex);
		rate_tbl_x_go = 0;
		pthread_mutex_unlock(&rate_tbl_x_mutex);
		
		header_offset = 0;
		tx_buffer = calloc(1,frame_full_size);

		memcpy(tx_buffer + header_offset, &frame.payload_len, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, &frame.crc, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, arg, frame.payload_len);
		int i = 0;
		for (i = 0; i < RATE_TBL_FREQ_PER_RX_CAN; ++i)
		{
			status = write(rate_tbl_x_rs422_fd, tx_buffer, frame_full_size);
			RATE_TABLE_TXDELAY;
		}
		if (status < 0)
			printf("ERROR status %d\n", status);
		//hex_dump("rate_table_x", tx_buffer, frame_full_size);
		free(tx_buffer);
	}
}

void *rate_table_y_thread(void *arg)
{
	uint32_t frame_full_size;
	int  status = 0;
	uint8_t *tx_buffer;
	uint32_t crc, total;
	uint32_t header_offset = 0;
	struct data_frame_header_t frame;

	rate_tbl_y_rs422_fd = open_port(SERIAL_PORT(2));
	if (rate_tbl_y_rs422_fd < 0) {
		printf("open_port error!!\n");
		exit(EXIT_FAILURE);
	}
	// Set the configureation of the port 
	set_interface_attribs(rate_tbl_y_rs422_fd, USER_BAUD_RATE, 0);

	frame_full_size = sizeof(int32_t) + sizeof(struct data_frame_header_t);
	frame.payload_len = frame_full_size - sizeof(struct data_frame_header_t);

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

	while (1) {
		pthread_mutex_lock(&rate_tbl_y_mutex);
		while (!rate_tbl_y_go)
			pthread_cond_wait (&rate_tbl_y_cond, &rate_tbl_y_mutex);
		rate_tbl_y_go = 0;
		pthread_mutex_unlock(&rate_tbl_y_mutex);
		
		header_offset = 0;
		tx_buffer = calloc(1,frame_full_size);

		memcpy(tx_buffer + header_offset, &frame.payload_len, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, &frame.crc, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, arg, frame.payload_len);
		int i = 0;
		for (i = 0; i < RATE_TBL_FREQ_PER_RX_CAN; ++i)
		{
			status = write(rate_tbl_y_rs422_fd, tx_buffer, frame_full_size);
			RATE_TABLE_TXDELAY;
		}
		if (status < 0)
			printf("ERROR status %d\n", status);
		//hex_dump("rate_table_y", tx_buffer, frame_full_size);
		free(tx_buffer);
	}
}

void *rate_table_z_thread(void *arg)
{
	uint32_t frame_full_size;
	int  status = 0;
	uint8_t *tx_buffer;
	uint32_t crc, total;
	uint32_t header_offset = 0;
	struct data_frame_header_t frame;

	rate_tbl_z_rs422_fd = open_port(SERIAL_PORT(3));
	if (rate_tbl_z_rs422_fd < 0) {
		printf("open_port error!!\n");
		exit(EXIT_FAILURE);
	}
	// Set the configureation of the port 
	set_interface_attribs(rate_tbl_z_rs422_fd, USER_BAUD_RATE, 0);

	frame_full_size = sizeof(int32_t) + sizeof(struct data_frame_header_t);
	frame.payload_len = frame_full_size - sizeof(struct data_frame_header_t);

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

	while (1) {
		pthread_mutex_lock(&rate_tbl_z_mutex);
		while (!rate_tbl_z_go)
			pthread_cond_wait (&rate_tbl_z_cond, &rate_tbl_z_mutex);
		rate_tbl_z_go = 0;
		pthread_mutex_unlock(&rate_tbl_z_mutex);
		
		header_offset = 0;
		tx_buffer = calloc(1,frame_full_size);

		memcpy(tx_buffer + header_offset, &frame.payload_len, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, &frame.crc, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, arg, frame.payload_len);
		int i = 0;
		for (i = 0; i < RATE_TBL_FREQ_PER_RX_CAN; ++i)
		{
			status = write(rate_tbl_z_rs422_fd, tx_buffer, frame_full_size);
			RATE_TABLE_TXDELAY;
		}
		if (status < 0)
			printf("ERROR status %d\n", status);
		//hex_dump("rate_table_z", tx_buffer, frame_full_size);
		free(tx_buffer);
	}
}

void *imu02_thread(void *arg)
{
	uint32_t frame_full_size;
	int  status = 0;
	uint8_t *tx_buffer;
	uint32_t crc, total;
	uint32_t header_offset = 0;
	struct data_frame_header_t frame;

	imu02_rs422_fd = open_port(SERIAL_PORT(4));
	if (imu02_rs422_fd < 0) {
		printf("open_port error!!\n");
		exit(EXIT_FAILURE);
	}
	// Set the configureation of the port 
	set_interface_attribs(imu02_rs422_fd, USER_BAUD_RATE, 0);

	frame_full_size = sizeof(struct IMU_filtered_data_t) + sizeof(struct data_frame_header_t);
	frame.payload_len = frame_full_size - sizeof(struct data_frame_header_t);

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

	while (1) {
		pthread_mutex_lock(&imu02_mutex);
		while (!imu02_go)
			pthread_cond_wait(&imu02_cond, &imu02_mutex);
		imu02_go = 0;
		pthread_mutex_unlock(&imu02_mutex);
		
		header_offset = 0;
		tx_buffer = calloc(1,frame_full_size);

		memcpy(tx_buffer + header_offset, &frame.payload_len, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, &frame.crc, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, arg, frame.payload_len);
		int i = 0;
		for (i = 0; i < IMU_FREQ_PER_RX_CAN; ++i)
		{
			status = write(imu02_rs422_fd, tx_buffer, frame_full_size);
			IMU_TXDELAY;
		}
		if (status < 0)
			printf("ERROR status %d\n", status);
		//hex_dump("imu02", tx_buffer, frame_full_size);
		free(tx_buffer);
	}
}

void *gpsr01_thread(void *arg)
{
	uint32_t frame_full_size;
	int  status = 0;
	uint8_t *tx_buffer;
	uint32_t crc, total;
	uint32_t header_offset = 0;
	struct data_frame_header_t frame;

	gpsr01_rs422_fd = open_port(SERIAL_PORT(5));
	if (gpsr01_rs422_fd < 0) {
		printf("open_port error!!\n");
		exit(EXIT_FAILURE);
	}
	// Set the configureation of the port 
	printf("%s\n", SERIAL_PORT(5));
	set_interface_attribs(gpsr01_rs422_fd, USER_BAUD_RATE, 0);

	frame_full_size = sizeof(struct NSPO_GPSR_SCI_TLM_t) + sizeof(struct data_frame_header_t);
	//frame_full_size = sizeof(struct IMU_filtered_data_t) + sizeof(struct data_frame_header_t);

	frame.payload_len = frame_full_size - sizeof(struct data_frame_header_t);

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

	while (1) {
		pthread_mutex_lock(&gpsr01_mutex);
		while (!gpsr01_go)
			pthread_cond_wait(&gpsr01_cond, &gpsr01_mutex);
		gpsr01_go = 0;
		pthread_mutex_unlock(&gpsr01_mutex);
		
		header_offset = 0;
		tx_buffer = calloc(1,frame_full_size);

		memcpy(tx_buffer + header_offset, &frame.payload_len, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, &frame.crc, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, arg, frame.payload_len);
		
		status = write(gpsr01_rs422_fd, tx_buffer, frame_full_size);
		//status = data_send_scatter(gpsr01_rs422_fd, tx_buffer, frame_full_size);
		if (status < 0)
			printf("ERROR status %d\n", status);
		//hex_dump("gpsr01", tx_buffer, frame_full_size);
		free(tx_buffer);
	}
}

void *gpsr02_thread(void *arg)
{
	uint32_t frame_full_size;
	int  status = 0;
	uint8_t *tx_buffer;
	uint32_t crc, total;
	uint32_t header_offset = 0;
	struct data_frame_header_t frame;

	gpsr02_rs422_fd = open_port(SERIAL_PORT(6));
	if (gpsr02_rs422_fd < 0) {
		printf("open_port error!!\n");
		exit(EXIT_FAILURE);
	}
	// Set the configureation of the port 
	set_interface_attribs(gpsr02_rs422_fd, USER_BAUD_RATE, 0);

	frame_full_size = sizeof(struct NSPO_GPSR_SCI_TLM_t) + sizeof(struct data_frame_header_t);
	frame.payload_len = frame_full_size - sizeof(struct data_frame_header_t);

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

	while (1) {
		pthread_mutex_lock(&gpsr02_mutex);
		while (!gpsr02_go)
			pthread_cond_wait(&gpsr02_cond, &gpsr02_mutex);
		gpsr02_go = 0;
		pthread_mutex_unlock(&gpsr02_mutex);
		
		header_offset = 0;
		tx_buffer = calloc(1,frame_full_size);

		memcpy(tx_buffer + header_offset, &frame.payload_len, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, &frame.crc, 4);
		header_offset += 4;
		
		memcpy(tx_buffer + header_offset, arg, frame.payload_len);
		
		status = write(gpsr02_rs422_fd, tx_buffer, frame_full_size);
		if (status < 0)
			printf("ERROR status %d\n", status);
		//hex_dump("gpsr02", tx_buffer, frame_full_size);
		free(tx_buffer);
	}
}

struct thread_info_t thread_info[] = {
	{(void *)&imu_data,  imu01_thread},
	{(void *)&(ratetable.rate.x), rate_table_x_thread},
	{(void *)&(ratetable.rate.y), rate_table_y_thread},
	{(void *)&(ratetable.rate.z), rate_table_z_thread},
	{(void *)&imu_data,  imu02_thread},
	{(void *)&gpsr_data, gpsr01_thread},
	{(void *)&gpsr_data, gpsr02_thread}
};

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
	pthread_t threads_id[THREADS_NUM];
	int32_t idx = 0, ret = 0;

#if 0
	if (argc != 2)
	{
		printf("usage : readCAN <CAN interface>\n");
	}
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
	fcntl(socket_can,F_GETFL,0);

	can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT |CAN_ERR_BUSOFF;
	status = setsockopt(socket_can, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
	if (status != 0)
		printf("setsockopt fail\n");

	printf("using %s to read\n", ifname);


	/*Pattern init*/
	imu_pattern_init(&imu_data);
	rate_table_pattern_init(&ratetable);
	gpsr_pattern_init((void *)&gpsr_data);
#if 1
	for (idx = 0; idx < THREADS_NUM; idx++) {
		printf("In main: creating thread %d\n", idx);
		ret = pthread_create(threads_id + idx, NULL, thread_info[idx].work_thread, thread_info[idx].payload);
		if (ret) {
			printf("ERROR; return code from pthread_create() is %d\n", ret);
			exit(-1);
		}
	}
#else /*debug purpose*/
	idx = 0;
	ret = pthread_create(threads_id + idx, NULL, thread_info[idx].work_thread, thread_info[idx].payload);
#endif


	while(1)
	{
		struct timespec ts;
		rx_nbytes = read(socket_can, &frame, sizeof(frame));
		if (rx_nbytes > 0) {
			rx_count--;
			//hex_dump("rx hex", (uint8_t *)&frame, sizeof(frame));
			if (frame.can_id & CAN_ERR_FLAG) {
				printf("error frame\n");
			} 
			if (rx_count == 0) {
				clock_gettime(CLOCK_MONOTONIC, &ts);
				printf("[%lld.%.9ld]CAN RX done!! \n",(long long)ts.tv_sec, ts.tv_nsec);
				pthread_mutex_lock(&imu01_mutex);
				imu01_go = 1;
				pthread_mutex_unlock(&imu01_mutex);

				pthread_mutex_lock(&imu02_mutex);
				imu02_go = 1;
				pthread_mutex_unlock(&imu02_mutex);

				pthread_mutex_lock(&gpsr01_mutex);
				gpsr01_go = 1;
				pthread_mutex_unlock(&gpsr01_mutex);

				pthread_mutex_lock(&gpsr02_mutex);
				gpsr02_go = 1;
				pthread_mutex_unlock(&gpsr02_mutex);

				pthread_mutex_lock(&rate_tbl_x_mutex);
				rate_tbl_x_go = 1;
				pthread_mutex_unlock(&rate_tbl_x_mutex);

				pthread_mutex_lock(&rate_tbl_y_mutex);
				rate_tbl_y_go = 1;
				pthread_mutex_unlock(&rate_tbl_y_mutex);

				pthread_mutex_lock(&rate_tbl_z_mutex);
				rate_tbl_z_go = 1;
				pthread_mutex_unlock(&rate_tbl_z_mutex);
				rx_count = RX_COUNTDOWN;
				
				pthread_cond_signal(&imu01_cond);
				pthread_cond_signal(&imu02_cond);
				pthread_cond_signal(&gpsr01_cond);
				pthread_cond_signal(&gpsr02_cond);
				pthread_cond_signal(&rate_tbl_x_cond);
				pthread_cond_signal(&rate_tbl_y_cond);
				pthread_cond_signal(&rate_tbl_z_cond);
			}
		}
	}
#if 0
	for (idx = 0; idx < THREADS_NUM; idx++) {
		pthread_cancel(threads_id[idx]);
	}
#endif
	/* wait for all threads to complete */
	for (idx = 0; idx < THREADS_NUM; idx++) {
		pthread_join(threads_id[idx], NULL);
	}

	return 0;
}

static __attribute__((constructor)) void init(void)
{

}

static __attribute__((destructor)) void finish(void)
{
	close(imu01_rs422_fd);
	close(imu02_rs422_fd);
	close(gpsr01_rs422_fd);
	close(gpsr02_rs422_fd);
	close(rate_tbl_x_rs422_fd);
	close(rate_tbl_y_rs422_fd);
	close(rate_tbl_z_rs422_fd);
}