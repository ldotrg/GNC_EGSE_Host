#ifndef __RS422_SERIALPORT_H__
#define __RS422_SERIALPORT_H__

#include "imu_interface.h"
#include "nspo_gps.h"
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>

#define SERIAL_PORT(idx) "/dev/ttyAP"#idx
#define RS422_SERIAL_BUFFER_SIZE 96
#define SWAP32(x) \
	((uint32_t) (\
	(((uint32_t) (x) & (uint32_t) 0x000000ffUL) << 24) | \
	(((uint32_t) (x) & (uint32_t) 0x0000ff00UL) << 8) | \
	(((uint32_t) (x) & (uint32_t) 0x00ff0000UL) >> 8) | \
	(((uint32_t) (x) & (uint32_t) 0xff000000UL) >> 24)))

#define cpu2le32(x) SWAP32((x))

typedef enum _ENUM_RS422_CMD_ID {
	RS422_CMD_GPSR_DATA = 0x1,
	RS422_CMD_IMU_DATA  = 0x2
} ENUM_RS422_CMD_ID, *P_ENUM_RS422_CMD_ID;


struct data_frame_header_t {
	uint32_t payload_len;
	uint32_t crc;
	uint32_t seq_no;
} __attribute__((packed));
/* 
*
* /dev/ttyAP0, /dev/ttyAP4: SDT_INTERFACER_t 200HZ for IMU
* /dev/ttyAP5, /dev/ttyAP6: SDT_INTERFACER_t 20HZ for GPSR
*
*/
struct SDT_INTERFACER_t {
    struct NSPO_GPSR_SCI_TLM_t     GPS_data_1;  // 624 bytes
    struct NSPO_GPSR_SCI_TLM_t     GPS_data_2;  // 624 bytes (redundancy)
    struct IMU_filtered_data_t     IMU_filtered_data_1;  // 96 bytes
    struct IMU_filtered_data_t     IMU_filtered_data_2;  // 96 bytes (redundancy)
} __attribute__((packed));

/* 
*
* /dev/ttyAP1(x), /dev/ttyAP2(y), /dev/ttyAP3(z): ProAxeSE_data_t 1000HZ
*
*/
struct int32_xyz_t {                                                                                                                   
	int32_t x;
	int32_t y;
	int32_t z;
} __attribute__((packed));

struct ProAxeSE_data_t {
	struct int32_xyz_t    rate;            /* *io  (r/s)     X, Y, Z Rate */
} __attribute__((packed));

typedef void *(*work_thread_f)(void *arg);

struct thread_info_t {
	char thread_name[12];
	char port_name[12];
	int32_t rs422_fd;
	uint32_t payload_size;
	uint32_t freq;
	uint8_t go_flag;
	void *payload;
	pthread_mutex_t *mutex;
	pthread_cond_t *cond;
};


int open_port(char *portname);
int set_interface_attribs(int fd, int speed, int parity);
int32_t data_send_scatter(int fd, uint8_t *payload, uint32_t data_len);
int32_t imu_pattern_init(struct IMU_filtered_data_t *imu);
int32_t rate_table_pattern_init(struct ProAxeSE_data_t *position);
int32_t gpsr_pattern_init(void *gpsr_data);
void hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen);
uint32_t crc32(uint32_t crc, const char *buf, uint32_t len);
uint32_t invert_crc32(uint32_t crc);
uint32_t crc_checker(uint32_t rx_crc, const char *buf, uint32_t size);

#endif /* __RS422_SERIALPORT__ */