#ifndef SERVER_H
#define SERVER_H

/* 
 * K610作为服务端，处理R329的连接，并且接收R329传过来的文件
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>

#define READBUFFSIZE 10240

const char* IP = "192.168.1.104";	// 本机的 IP 和 端口
int port = 9999;
struct sockaddr_in clientAddr;				// 连接客户端
char readBuff[READBUFFSIZE];                // 接收缓冲区

int connectWithClient(const char* IP, int port);
void receiveFiles(int cfd);
void* communicateWithClient(void* argv);

#endif