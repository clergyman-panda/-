#ifndef SERVER_UPLOAD_H
#define SERVER_UPLOAD_H

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

using namespace std;

const int WriteBuffSize = 10240;
const int ReadBuffSize = 10240;

class Upload{

public:
	Upload(const char* IP = "192.168.193.128", int port = 9999);
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


#endif //SERVER_UPLOAD_H
