#include "server.h"
#include "upload.h"
#include"opencv2/opencv.hpp"
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
using namespace cv;
using namespace std;
extern char savebuf[128];
extern char localbuf[128];
extern pthread_mutex_t save_mutex;
int connectWithClient(const char* IP, int port) {
    
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
	
    // 4.等待客户端连接
    socklen_t clientAddrLen = sizeof(clientAddr);
    int cfd = accept(lfd, (struct sockaddr*)&clientAddr, &clientAddrLen);
    if (cfd == -1) {
        perror("accept");
        exit(-1);
    }
	
    close(lfd);
    return cfd;
}

void receiveFiles(int cfd) {
    
    char clientIP[16];
	inet_ntop(AF_INET, &clientAddr.sin_addr.s_addr, clientIP, sizeof(clientIP));
	uint16_t clientPort = ntohs(clientAddr.sin_port);
	printf("client ip : %s, port is %d\n", clientIP, clientPort);

    // 接收客户端发送的信息
    int recvFileFlag = 0;   // 文件接收阶段标志
	char fileNameBuff[256];
	char fileSizeBuff[32];
	long fileSize = 0;
	long recvSize = 0;
	FILE* recvFile;

	// 将文件上传服务器
	Upload upload(serverIP, serverPort);
	upload.connectToServer();
	std::cout << "成功连接至服务器\n";
	
	while (1) {
	    bzero(readBuff, ReadBuffSize);
		int len = read(cfd, &readBuff, sizeof(readBuff));
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
					printf("%s\n", fileNameBuff);
					
									
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
				write(cfd, readBuff, strlen(readBuff));
				recvFileFlag = 1;
            }
			else if (recvFileFlag == 1) {	// 保存文件内容
				len = fwrite(readBuff, sizeof(char), len, recvFile);
				recvSize += len;
				if (recvSize == fileSize) {
					
					// 成功接收文件
					printf("Receive the file %s successfully!\n", fileNameBuff);
					int ret = fclose(recvFile);
					if (ret != 0) {
						perror("fclose");
						exit(-1);
					}
					recvSize = 0;
					recvFileFlag = 0;
					Mat src;
  				    src = imread(fileNameBuff, 1); // 读取图像文件
					printf("image read over\r\n");
				

					pthread_mutex_lock(&save_mutex);
					cv::putText(src, localbuf, cv::Point2d(0, 40),
							cv::FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 2);
					pthread_mutex_unlock(&save_mutex);
					printf("%s\r\n",localbuf);
					
					imwrite(fileNameBuff,src);
					printf("image write over\r\n");

					upload.uploadFile(fileNameBuff);
				}
			}
		}
		else if (len == 0) {
			printf("client closed!\n");
			break;
		}
	}
	close(cfd);
}

/*
 * 与小板子通信的线程，用来接收小板子的连接，并且接收小板子传输的文件；
 * 不进行文件传输的时候线程处于阻塞状态。
 */
void* communicateWithClient(void* argv) {

	while (1) {
		int cfd = connectWithClient(IP, port);
		receiveFiles(cfd);
	}

	return NULL;
}
