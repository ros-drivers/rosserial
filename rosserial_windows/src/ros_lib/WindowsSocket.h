#ifndef WINDOWSSOCKET_H_
#define WINDOWSSOCKET_H_

extern "C++" int WinSock_Read();
extern "C++" int WinSock_Init(char* IP, char* port);
extern "C++" int WinSock_Write(unsigned char* data, int length);
// We have to do the below to ensure windows.h does not mess everything up
extern "C++" unsigned long WinSock_Time();

#include <stdio.h>

class WindowsSocket {
public:
	WindowsSocket()
	{
	}
	void init(char* IP, char* port)
	{
		if (WinSock_Init(IP,port) == 0) printf("Success!\n");
	}
	int read()
	{
		int c = WinSock_Read();
		printf("Data: %d\n", c);
		return c;
	}
	void write(unsigned char* data, int length)
	{
		WinSock_Write(data,length);
	}

	unsigned long time()
	{
		unsigned long t = WinSock_Time();
		printf("Time: %d\n", t);
		return t;
	}
};


#endif
