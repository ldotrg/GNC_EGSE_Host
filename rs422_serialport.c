
/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */
#include "rs422_serialport.h"

int open_port(char *portname)
{
	int fd;
	fd = open(portname, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		fprintf(stderr, "Error: %d opening %s: %s\n", errno, portname, strerror (errno));
		return errno;
	}      

	return (fd);
}

int set_interface_attribs(int fd, int speed, int parity) 
{
	struct termios options;
	int ret;

	// Get the current options for the port
	if((ret = tcgetattr(fd, &options)) < 0){
		fprintf(stderr, "failed to get attr: %d, %s\n", fd, strerror(errno));
		exit(EXIT_FAILURE);
	}

	cfsetospeed(&options, (speed_t)speed);
	cfsetispeed(&options, (speed_t)speed);
	/* Enable the receiver and set local mode...*/
	options.c_cflag |= (CLOCAL | CREAD);

	/* Setting the Character Size */
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	options.c_cflag &= ~(PARENB | PARODD);
	options.c_cflag |= parity;
	options.c_cflag &= ~CSTOPB;   //1 stop bits

       /*Setting Hardware Flow Control*/
        options.c_cflag &= ~CRTSCTS;

	/* Local Options: Choosing Raw Input*/
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	
	/* Input Options: Setting Software Flow Control */
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_iflag &= ~(INLCR | ICRNL);

	/* Output Options: Choosing Raw Output */
	options.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
        options.c_cc[VMIN]  = 0;            // read doesn't block
        options.c_cc[VTIME] = 5;            // 0.5 seconds read timeout


    // Set the new attributes
    if((ret = tcsetattr(fd, TCSANOW, &options)) < 0){
        fprintf(stderr, "failed to set attr: %d, %s\n", fd, strerror(errno));
        exit(EXIT_FAILURE);
    }
    return 1;
}

int32_t data_send_scatter(int fd, uint8_t *payload, uint32_t data_len)
{
	uint32_t sent_len = 0, cur_len = 0;
	uint32_t sent_len_max = RS422_SERIAL_BUFFER_SIZE;
	int ret = 0;
	while (1) {
		sent_len = (data_len - cur_len) >=  sent_len_max ?
				   sent_len_max : (data_len - cur_len);
		if (sent_len > 0) {
			ret = write(fd, payload + cur_len, sent_len);
			if (ret < 0)
				goto error;

			cur_len += sent_len;
		} else
			break;
	}


error:
	//printf("%s:(ret = %d)\n", __func__, ret);
	return ret;
}

int32_t imu_pattern_init(struct IMU_filtered_data_t *imu)
{
	imu->r1 = 0x0101;
	imu->r2 = 0x0202;
	imu->r3 = 0x0303;
	imu->r4 = 0x0404;
	imu->r5 = 0x0505;
	imu->r6 = 0x0606;
	imu->r7 = 0x0707;
	imu->r8 = 0x0808;
	imu->r9 = 0x0909;
	imu->r10 = 0x0a0a;
	imu->r11 = 0x0b0b;
	imu->r12 = 0x0c0c;
	imu->q1 = 0x0d0d;
	imu->q2 = 0x0e0e;
	imu->q3 = 0x0f0f;
	imu->q4 = 0x1010;
	imu->q5 = 0x1111;
	imu->q6 = 0x1212;
	imu->q7 = 0x1313;
	imu->q8 = 0x1414;
	imu->q9 = 0x1515;
	imu->q10 = 0x1616;
	imu->q11 = 0x1717;
	imu->q12 = 0x1818;

	return 0;
}

int32_t rate_table_pattern_init(struct ProAxeSE_data_t *position)
{
	position->rate.x = 0xabababab;
	position->rate.y = 0xcdcdcdcd;
	position->rate.z = 0xefefefef;
	return 0;
}
int32_t gpsr_pattern_init(void *gpsr_data)
{
	uint8_t *tmpbuffer;
	tmpbuffer = (uint8_t *)gpsr_data;
	int idx;

	for (idx = 0; idx < sizeof(struct NSPO_GPSR_SCI_TLM_t); idx++)
	{
		tmpbuffer[idx] = idx & 0xFF;
	}
	return 0;
}

void hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen)
{
	uint8_t *pt;
	int x;
	pt = pSrcBufVA;
	printf("%s: %p, len = %d\n\r", str, pSrcBufVA, SrcBufLen);
	for (x=0; x<SrcBufLen; x++)
	{
	   if (x % 16 == 0)
	   printf("0x%04x : ", x);
	   printf("%02x ", ((uint8_t)pt[x]));
	   if (x%16 == 15) printf("\n\r");
	}
	printf("\n\r");
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

uint32_t invert_crc32(uint32_t crc)
{
	uint32_t	crc_30_00;
	uint32_t	crc_31;
	uint32_t	crc_tmp;
	uint32_t inv_crc;
	char  *ucPoint;

	crc_30_00 =	( (crc >> 1) & 0x7fffffff );
	crc_31	 = (crc & 0x1) ^
		((crc & (0x1 << 4)) >> 4) ^
		((crc & (0x1 << 8)) >> 8) ^
		((crc & (0x1 << 12)) >> 12) ^
		((crc & (0x1 << 17)) >> 17) ^
		((crc & (0x1 << 20)) >> 20) ^
		((crc & (0x1 << 23)) >> 23) ^
		((crc & (0x1 << 28)) >> 28);
	inv_crc = (crc_30_00 & 0x7fffffff ) | ( (crc_31 << 31) & 0x80000000);

	printf("  Invert_CRC crc32=0x%8x \n", inv_crc);
	ucPoint = (char *)&inv_crc;
	crc_tmp = crc32(crc, ucPoint, 4);
        printf("  After Invert CRC==> 0x%x\n", crc_tmp);
        return inv_crc;
}

uint32_t crc_checker(uint32_t rx_crc, const char *buf, uint32_t size)
{
	
	uint32_t crc = 0, total = 0;
        while (total < size)
        {
        	crc = crc32(crc, (char *)(buf + total), 4);
        	total += 4;
        }
        //printf("crc 0x%x, rx_crc: 0x%x\n", crc, rx_crc);
        return (rx_crc == crc);

}
