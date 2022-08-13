#include "upload.h"

Upload::Upload(const char*  IP, int port) {
	m_cfd = 0;
	m_serverAddr.sin_family = AF_INET;
	m_serverAddr.sin_port = htons(port);
	inet_pton(AF_INET, IP, &m_serverAddr.sin_addr.s_addr);

	bzero(writeBuff, WriteBuffSize);
	bzero(readBuff, ReadBuffSize);
}

Upload::~Upload() {
	if (m_cfd != 0) {
		close(m_cfd);
		m_cfd = 0;
	}
}

void Upload::connectToServer() {

	// 创建通信套接字
	m_cfd = socket(AF_INET, SOCK_STREAM, 0);
	if (m_cfd == -1) {
		perror("socket");
		exit(-1);
	}

	// 连接至服务器
	int ret = connect(m_cfd, (struct sockaddr*)&m_serverAddr, sizeof(m_serverAddr));
	if (ret == -1) {
		perror("connect");
		exit(-1);
	}
}

bool Upload::uploadFile(const char *fileName) {
	// 打开文件
	struct stat fileInfo{};
	stat(fileName, &fileInfo);
	FILE *sendFile = fopen(fileName, "r");
	if (sendFile == NULL) {
		perror("fopen");
		exit(-1);
	}

	// 先发送文件头部（文件名和文件大小），等待服务器回应
	// 文件头部示例：FileName:test.mp4;FileSize:112300351
	char fileSize[32] = {0};            // 文件的大小，单位为字节
	char fileHead[256] = "FileName:";   // 文件头部信息
	sprintf(fileSize, ";FileSize:%ld", fileInfo.st_size);
	strcat(fileHead, fileName);
	strcat(fileHead, fileSize);

	strcpy(writeBuff, fileHead);
	write(m_cfd, fileHead, strlen(writeBuff));
        
	// 等待服务器回复的文件头部确认信息
	ssize_t readLen = read(m_cfd, readBuff, sizeof(readBuff));
	if (readLen == -1) {
		perror("read");
		exit(-1);
	}
	
	long sendSize = 0;
	// 收到服务器回应之后，发送文件本体
	if (strcmp(fileHead, readBuff) == 0) {
		int len = 0;
		while ((len = fread(writeBuff, sizeof(char), WriteBuffSize, sendFile)) > 0) {
			if (write(m_cfd, writeBuff, len) != len) {
				perror("error");
				exit(-1);
			}
			sendSize += len;
			printf("sendSize=%ld, fileSize=%s\n", sendSize, fileSize);
		}

		fclose(sendFile);   // 发送结束，关闭文件
	}
	return false;
}

void Upload::disconnectToServer() {
	if (m_cfd != 0) {
		close(m_cfd);
		m_cfd = 0;
	}
}

