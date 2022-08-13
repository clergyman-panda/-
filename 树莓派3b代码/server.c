#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>

#define MaxConnection 1
#define ReadBuffSize  10240

char* IP = "192.168.1.102";
int port = 9999;

struct sockInfo {
	int fd;
	pthread_t tid;
	struct sockaddr_in addr;
};
struct sockInfo sockInfos[MaxConnection];   // 连接的客户端
char readBuff[ReadBuffSize];                // 接收缓冲区

void* communicating(void *argInput) {
	// 转换获得主线程传递的参数
	struct sockInfo* clientInfo = (struct sockInfo*)argInput;

	char clientIP[16];
	inet_ntop(AF_INET, &clientInfo->addr.sin_addr.s_addr, clientIP, sizeof(clientIP));
	uint16_t clientPort = ntohs(clientInfo->addr.sin_port);
	printf("client ip : %s, port is %d\n", clientIP, clientPort);

	// 接收客户端发送的信息
    int recvFileFlag = 0;   // 文件接收阶段标志
	char fileNameBuff[256];
	char fileSizeBuff[32];
	long fileSize = 0;
	long recvSize = 0;
	FILE* recvFile;
	while (1) {
		int len = read(clientInfo->fd, &readBuff, sizeof(readBuff));
		if (len == -1) {
			perror("read");
			exit(-1);
		}
		else if (len > 0) {
            if (recvFileFlag == 0) {    	// 解析文件名和文件大小
				// printf("%s\n", readBuff);
				// 文件头部示例：FileName:test.mp4;FileSize:112300351
                char key1[10] = {0};
				char key2[10] = {0};
				memset(fileNameBuff, '\0', sizeof(fileNameBuff));
				memset(fileSizeBuff, '\0', sizeof(fileSizeBuff));
				
				strncpy(key1, readBuff, 9);
				if (strcmp(key1, "FileName:") == 0) {
					// 获取文件名
					char* pos = strchr(readBuff, ';');
					strncpy(fileNameBuff, readBuff + 9, (pos - readBuff - 9));
					// printf("%s\n", fileNameBuff);

					// 获取文件大小
					strncpy(key2, pos + 1, 9);
					if (strcmp(key2, "FileSize:") == 0) {
						char* pos = strchr(readBuff + 9, ':');
						strncpy(fileSizeBuff, pos + 1, (readBuff + strlen(readBuff) - pos));
						// printf("%s\n", fileSizeBuff);
						fileSize = atol(fileSizeBuff);
					}
					else {
						printf("The format of file-header is wrong!\n");
						exit(-1);	
					}
				}
				else {
					printf("The format of file-header is wrong!\n");
					exit(-1);
				}

				// 创建文件
				recvFile = fopen(fileNameBuff, "w+");
				if (recvFile == NULL) {
					perror("fopen");
    				exit(-1);
				}
				write(clientInfo->fd, readBuff, strlen(readBuff));
				recvFileFlag = 1;
            }
			else if (recvFileFlag == 1) {	// 保存文件内容
				len = fwrite(readBuff, sizeof(char), len, recvFile);
				recvSize += len;
				if (recvSize == fileSize) {
					printf("Receive the file %s successfully!\n", fileNameBuff);
					int ret = fclose(recvFile);
					if (ret != 0) {
						perror("fclose");
						exit(-1);
					}
					recvSize = 0;
					recvFileFlag = 0;
				}
			}
			memset(readBuff, '\0', ReadBuffSize);
		}
		else if (len == 0) {
			printf("client closed!\n");
			break;
		}
	}
	close(clientInfo->fd);
	bzero(clientInfo, sizeof(clientInfo));
	clientInfo->fd = -1;
	clientInfo->tid = -1;
	return NULL;
}

int main() {

	// 1.创建监听套接字
	int lfd = socket(AF_INET, SOCK_STREAM, 0);
	if (lfd == -1) {
		perror("socket");
		exit(-1);
	}

	int optval = 1;
   setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

	// 2.将监听套接字与端口绑定
	struct sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(port);
	inet_pton(AF_INET, IP, &serverAddr.sin_addr.s_addr);
	int ret = bind(lfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
	if (ret == -1) {
		perror("bind");
		exit(-1);
	}

	// 3.监听
	ret = listen(lfd, 128);
	if (ret == -1) {
		perror("listen");
		exit(-1);
	}

	// 4.初始化数据
	for (int i = 0; i < MaxConnection; i++) {
		bzero(&sockInfos[i], sizeof(sockInfos[i]));
		sockInfos[i].fd = -1;
		sockInfos[i].tid = -1;
	}

	// 5.等待客户端连接
	while (1) {
		// 接收客户端连接
		struct sockaddr_in clientAddr;
		int clientAddrLen = sizeof(clientAddr);
		int cfd = accept(lfd, (struct sockaddr*)&clientAddr, &clientAddrLen);
		if (cfd == -1) {
			perror("accept");
			exit(-1);
		}

		// 创建一个子线程进行通信
		struct sockInfo* clientInfo;
		for (int i = 0; i < MaxConnection; ++i) {
			if (sockInfos[i].fd == -1) {
				clientInfo = &sockInfos[i];
				break;
			}
			// 资源不足则阻塞等待
			if (i == MaxConnection - 1) {
				sleep(1);
				i = 0;
			}
		}
		clientInfo->fd = cfd;
		clientInfo->addr = clientAddr;

		ret = pthread_create(&clientInfo->tid, NULL, communicating, clientInfo);
		if (ret != 0) {
			printf("create child pthread error!\n");
		}

		pthread_detach(clientInfo->tid);
	}

	return 0;
}
