
#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>


// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

#define DEFAULT_PORT "11411" //for your information
int iResult;
WSADATA wsaData;
SOCKET ConnectSocket = INVALID_SOCKET;
struct addrinfo *result = NULL,*ptr = NULL, hints;

int WinSock_Init(char* IP, char* port)
{
	// Initialize Winsock
	printf("Connecting to TCP server at %s:%s....\n", IP, port);
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo(IP, port, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 1;
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 1;
		}

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);

		char value = 1; //disable nagle algorithm
		setsockopt( ConnectSocket, IPPROTO_TCP, TCP_NODELAY, &value, sizeof( value ) );
		u_long iMode = 1; //enable blocking
		iResult = ioctlsocket(ConnectSocket, FIONBIO,&iMode);
		//struct timeval tv;
		//tv.tv_sec = 0;
		//tv.tv_usec = 10000;
		//setsockopt(ConnectSocket, SOL_SOCKET, SO_RCVTIMEO,(char *)&tv,sizeof(struct timeval));

		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}


	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return 1;
	}
	return 0;
}



unsigned long WinSock_Time()
{
	SYSTEMTIME st_now;
	FILETIME ft_now;
	GetSystemTime(&st_now);
	unsigned long millis = st_now.wHour * 3600000 +
		                   st_now.wMinute * 60000 +
						   st_now.wSecond * 1000 +
						   st_now.wMilliseconds;
	return millis;
}

int WinSock_Write(unsigned char* data, int length)
{
	// Send an initial buffer
	iResult = send(ConnectSocket, reinterpret_cast<const char*>(data), length, 0);
	if (iResult == SOCKET_ERROR) {
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		return 1;
	}
	printf("Bytes Sent: %ld\n", iResult);
	return iResult;
}

int WinSock_Read()
{
	printf("13 ");
	char data;
	iResult = recv(ConnectSocket, &data, 1, 0);
	printf("14 ");
	if (iResult > 0)
	{
		//printf("%d",uData);
		printf("  Bytes received: %d\n", iResult);
	}
	else if (iResult == 0)
	{
		printf("Connection closed\n");
		return -1;
	}
	else
	{
		printf("recv failed with error: %d\n", WSAGetLastError());
		return -1;
	}

	return (unsigned char) data;
}