#ifndef USART_H
#define USART_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <termios.h>
#include <sys/time.h>
#include <stdbool.h> 

typedef struct data{
    int VSS;
    int RPM;
    float LOAD_PCT;
    int CNT;
    float Longitude;
    float Latitude;
    int TP;
    float ACC; 
} vehicle_data;
int open_uart(const char* device_name);
int set_uart_attr(int fd, int nSpeed, int nBits, char nEvent, int nStop);
bool is_recv(char* buf);
void data_analyse(char* buf,vehicle_data* recvdata);

int read_data(int fd,char* buf,int size);
int write_data(int fd,char* buf,int size);
int setnonblocking( int fd );
int setblocking(int fd);

#endif
