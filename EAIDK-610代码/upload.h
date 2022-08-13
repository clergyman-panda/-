#ifndef UPLOAD_H
#define UPLOAD_H

/* 
 * K610作为客户端，与服务端建立连接，并且上传文件
 */

#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

using namespace std;

#define serverIP 	"192.168.1.102" // 服务器端的 IP 和 端口
#define serverPort  9999

const int WriteBuffSize = 10240;
const int ReadBuffSize = 10240;

class Upload{

public:
	Upload(const char* IP = "192.168.1.102", int port = 9999);
	~Upload();
	void connectToServer();
	void disconnectToServer();
	bool uploadFile(const char* fileName);

private:
	int m_cfd;
	struct sockaddr_in m_serverAddr;
	char writeBuff[WriteBuffSize];
	char readBuff[ReadBuffSize];
};


#endif //UPLOAD_H