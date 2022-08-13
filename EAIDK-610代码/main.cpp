#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> 
#include "usart.h"
#include <pthread.h>
#include <thread>
#include <string>
#include "usbcamera.h"



using namespace std;

#define USART_DEVICE_ "/dev/ttyUSB0"
#define USB_CAMERA "/dev/video0"
//#define SAVE_PATH "/run/media/openailab/3630-6538/video/"   //该路径在EAIDK-610的SD卡中，尽量用当前时间为该视频命名。
#define SAVE_PATH ""
void* thread_save_video(void *);
void* communicateWithClient(void* argv);
char savebuf[128]={0};
char localbuf[128]={0};
pthread_mutex_t save_mutex;
int main()
{
    int ret=0;
    int	fd;
    pthread_mutex_init(&save_mutex, NULL);
    char buf[2048] = {0};
    fd = open_uart(USART_DEVICE_);
    if (fd == -1)
    {
        printf("open failed\n");
        return -1;
    }

    set_uart_attr(fd, 115200, 8, 'N', 1);

    int frames_num = 100;
    int fps = 20;
    std::string saving_path = SAVE_PATH;
    UsbCamera fatigue_detec(frames_num, fps, saving_path, USB_CAMERA);
    std::thread fatigue_detection_thread(&UsbCamera::FatigueDrivingDetectionThread, &fatigue_detec);
    fatigue_detection_thread.detach();
    

 
    pthread_t save_video_pid;  



    //与R329通信的线程
    pthread_t communicateWithCleintThread;
    pthread_create(&communicateWithCleintThread, NULL, communicateWithClient, NULL);
    pthread_detach(communicateWithCleintThread);



	while (1)
	{
        vehicle_data recvdata;
        
		memset(buf,0,sizeof(buf));
    
		if (read_data(fd,buf,sizeof(buf)) < 0)
		{
			printf("read fail\n");
			continue;
		}
        data_analyse(buf,&recvdata);    //分析接受到的数据

        

        pthread_mutex_lock(&save_mutex);        
        memset(savebuf,0,sizeof(savebuf));
	
        sprintf(savebuf,"ACC:%.1f,TP:%d",recvdata.ACC,recvdata.TP);
        sprintf(localbuf,"Lat:%.4f,Lon:%.4f",recvdata.Latitude,recvdata.Longitude); 

	    pthread_mutex_unlock(&save_mutex);
        // printf("VSS:%d\r\n",recvdata.VSS);
        // printf("TP:%d\r\n",recvdata.TP);
        // printf("ACC:%.3f\r\n",recvdata.ACC);
        //printf("Latitude:%f\r\n",recvdata.Latitude);
        // printf("Longitude:%f\r\n",recvdata.Longitude);
            
        //检测到异常驾驶后通知STM32开发板，通过蜂鸣器提醒驾驶员
        if(fatigue_detec.is_fatigue_driving())
        {
            write_data(fd,"$FatigueDriving\r\n",17);
        }
        //判断是否危险驾驶或者异常驾驶
        if(((abs(recvdata.ACC)>2.5) && (recvdata.TP>75)) || (fatigue_detec.is_fatigue_driving()))
        {
            //printf("is fatigue driving\r\n");
            time_t now;
            struct tm *p_tm;
            char videoName[256]={0};
            time(&now);
            p_tm = localtime(&now);  //将time_t时间类型转换为 struct tm 时间结构体类型
            sprintf(videoName, "%4d-%02d-%02d %02d %02d %02d.avi", p_tm->tm_year+1900, p_tm->tm_mon+1, p_tm->tm_mday,p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);
            //fatigue_detec.setVedioName(videoName);
            //printf("******fatigue_detec.is_start_save:%d*****\r\n",fatigue_detec.is_start_save);
            if(fatigue_detec.is_start_save == false)
            {
                fatigue_detec.is_start_save = true;
                fatigue_detec.setVedioName(videoName);
                printf("*****************************create save_video thread***************************\r\n");
                
                pthread_create(&save_video_pid, NULL, thread_save_video, (void*)&fatigue_detec);
                pthread_detach(save_video_pid);
            }
        }
        


		//usleep(200);
    }
  
    close(fd);
    return ret;
}
