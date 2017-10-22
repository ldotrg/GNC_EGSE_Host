#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#define BUFSIZE 128
int main(int argc, char const *argv[])
{
	struct termios options;
	int fd, speed, parity = 0, i = 0, wdlen;
	char *portname = "/dev/ttyAP0";
	int ret;
	char tx_buffer[BUFSIZE];
	speed = B921600;
	fd = open(portname, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		fprintf(stderr, "Error: %d opening %s: %s\n", errno, portname, strerror(errno));
		return errno;
	} else {
		printf("Using %s to send\n", portname);
	}

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
    for (i = 0; i < BUFSIZE; ++i) {
    	tx_buffer[i] = i;
    }
    wdlen = write(fd, tx_buffer, BUFSIZE);
    printf("write to rs422 %d bytes\n", wdlen);
    while(1);
	return 0;
}