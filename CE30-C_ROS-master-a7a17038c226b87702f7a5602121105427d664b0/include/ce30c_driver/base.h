#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <math.h>
//#define DEBUG
#ifdef DEBUG
#include <fstream>
#endif // DEBUG

#define WIDTH       320
#define HEIGHT      24
#define TOTALDATA   2 * WIDTH * HEIGHT
//#define DEBUG
//#define PRINTDATA

// define command buff
const char kStart[51]       = "getDistanceAndAmplitudeSorted                     "; // start command
const char kStop[51]        = "join                                              "; // stop command
const char kDisconnect[51]  = "disconnect                                        "; // disconnect command
const char kGreyImage[51]   = "enableFeatures 131072                             "; // enable grey image
const char kSetIntegTime[51]= "setIntegrationTime3D 1600                         "; // integration time 1600
const char kSetROI[51]      = "roi 0 0 3                                         ";


// Ethernet connect
int TCP_connect(int &_sd){
    std::string addr = "192.168.1.80";
 	int port = 50660;

 	// CE_TCP
	_sd = socket(AF_INET, SOCK_STREAM, 0);
	if (_sd < 0){
		// printf("Socket error!\n");
		return -1;
	}
	struct sockaddr_in ser_addr = {0};
	ser_addr.sin_family = AF_INET;
	ser_addr.sin_port = htons(port);
	const char* c_addr = addr.c_str();
	ser_addr.sin_addr.s_addr = inet_addr(c_addr);

  // start connetion
	int ret = 0, total = 0;
	// printf("Device connecting ...\n");
	ret = connect(_sd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
	if(ret == -1){
		// printf("Connect failed!\n");
		return -1;
	}
	else{
		// printf("Connected!\n");
	}

	return 1;
}

// receive data
int recv_data(int _sd, unsigned char *_buff, int _length){
	int total = 0, ret = 0;
	while(total != _length){
		ret = recv(_sd, _buff + total, (_length - total), 0);
		if(ret < 0){
			// printf("Data receiving failed!\n");
			return -1;
		}
#ifdef PRINTDATA
		else{
			// printf("Receiving bytes: %d\n", ret);
		}
#endif // PRINTDATA
		total += ret;
	}

#ifdef PRINTDATA
	printf("Data: %02x, %02x, %02x, %02x, ...\n", _buff[0], _buff[1], _buff[2], _buff[3]);
#endif // PRINTDATA
	return total;
}

// send command
int send_command(int _sd, const char *_command, int _length){
    int total = 0, ret = 0;
    do
    {
        ret = send(_sd, _command + total, (_length - total), 0);
        if(ret < 0)
            return false;
        total += ret;
    }while(total != _length);
	return total;
}

