/*
 * embedded_linux_comms.c
 *
 *  Created on: Jun 16, 2012
 *      Author: bouchier
 */

#ifndef ROS_EMBEDDED_LINUX_COMMS_H
#define ROS_EMBEDDED_LINUX_COMMS_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <assert.h>

#define DEFAULT_PORTNUM 11411

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

void set_nonblock(int socket)
{
int flags;
flags = fcntl(socket,F_GETFL,0);
assert(flags != -1);
fcntl(socket, F_SETFL, flags | O_NONBLOCK);
}

int elCommInit(char *portName, int baud)
{
	struct termios options;
	int fd;
	char *ip;
	char *tcpPortNumString;
	long int tcpPortNum;
	int sockfd;
	struct sockaddr_in serv_addr;
	struct hostent *server;
	int rv;

	if (*portName == '/') {			// linux serial port names always begin with /dev
		printf("Opening serial port %s\n", portName);

		fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

		if (fd == -1){
			//Could not open the port.
			perror("init(): Unable to open serial port - ");
		}
		else{
			fcntl(fd, F_SETFL, FNDELAY); // Sets the read() function to return NOW and not wait for data to enter buffer if there isn't anything there.

			//Configure port for 8N1 transmission
			tcgetattr(fd, &options);					//Gets the current options for the port
			cfsetispeed(&options, B57600);				//Sets the Input Baud Rate
			cfsetospeed(&options, B57600);				//Sets the Output Baud Rate
			options.c_cflag |= (CLOCAL | CREAD);		//? all these set options for 8N1 serial operations
			options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;
			options.c_cflag &= ~CSIZE;
			options.c_cflag |= CS8;
			options.c_cflag &= ~CRTSCTS;
			options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	// set raw mode
			options.c_iflag &= ~(IXON | IXOFF | IXANY);		// disable SW flow control
			options.c_oflag &= ~OPOST;

			tcsetattr(fd, TCSANOW, &options);			//Set the new options for the port "NOW"
		}
		return fd;
	} else {		// Connect to the rosserial server
		// figure out the port number to use
		ip = strtok(portName, ":");			// IP address should be e.g. 192.168.1.10 or 192.168.1.10:11411
		tcpPortNumString = strtok(NULL, ":");	// get the port # if specified
		tcpPortNum = 0;
		if (tcpPortNumString != NULL) {
			tcpPortNum = strtol(tcpPortNumString, NULL, 10);	// convert port num string to long
		}
		if (tcpPortNum == 0){				// if port was not specified, or was not numeric
			tcpPortNum = DEFAULT_PORTNUM;
		}
		printf("Connecting to TCP server at %s:%ld....\n", ip, tcpPortNum);

		// create the socket
	    sockfd = socket(AF_INET, SOCK_STREAM, 0);
	    if (sockfd < 0) {
	        error("ERROR opening socket");
	        exit(-1);
	    }

	    /* Disable the Nagle (TCP No Delay) algorithm */
	    int flag = 1;
	    rv = setsockopt( sockfd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(flag) );
	    if (rv == -1) {
	      printf("Couldn't setsockopt(TCP_NODELAY)\n");
	      exit( -1 );
	    }

	    // connect to the server
	    server = gethostbyname(ip);
	    if (server == NULL) {
	        fprintf(stderr,"ERROR, no such host\n");
	        exit(0);
	    }
	    bzero((char *) &serv_addr, sizeof(serv_addr));
	    serv_addr.sin_family = AF_INET;
	    bcopy((char *)server->h_addr,
	         (char *)&serv_addr.sin_addr.s_addr,
	         server->h_length);
	    serv_addr.sin_port = htons(tcpPortNum);
	    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
	        error("ERROR connecting");
	    set_nonblock(sockfd);
	    printf("connected to server\n");
	    return sockfd;
	}
	return -1;
}

int elCommRead(int fd)
{
	unsigned char c;
	unsigned int i;
	int rv;
	rv = read(fd, &c, 1);	// read one byte
	i = c;					// convert byte to an int for return
	if (rv > 0)
		return i;			// return the character read
	if (rv < 0) {
		if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
			perror("elCommRead() error:");
	}
	return rv;				// return -1 or 0 either if we read nothing, or if read returned negative
}

int elCommWrite(int fd, uint8_t* data, int len)
{
	int rv;
	int length = len;
	int totalsent = 0;
	while (totalsent < length) {
		rv = write(fd, data+totalsent, length);
		if (rv < 0)
			perror("write(): error writing - trying again - ");
		else
			totalsent += rv;
	}
	return rv;
}

#endif
