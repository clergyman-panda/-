#ifndef USBCAMERA_H
#define USBCAMERA_H

#include<iostream>
#include<string>
#include "usart.h"
#include "upload.h"

#include<opencv2/opencv.hpp>

//#define SAVE_PATH "/run/media/openailab/3630-6538/video/"   //该路径在EAIDK-610的SD卡中，尽量用当前时间为该视频命名。
#define SAVE_PATH ""
class UsbCamera
{
private:
    int frames_num_;
    int fps_;
    int frame_index_;
    std::string video_save_path_;
    std::string video_save_name_;
    std::string camera_dev_;
    bool is_fatigue_driving_;           //单帧疲劳状态 default = false
    bool is_longtime_fatigue_driving_;  //长时间疲劳状态 default = false
    
    std::vector<cv::Mat> *cache_frames_ptr_;

    // std::vector<cv::Mat> *last_frames_ptr_; //存储检测到疲劳的前一段时间图像
    // std::vector<cv::Mat> *next_frames_ptr_; //检测到疲劳的后一段时间图像
    int FatigueDrivingDetec(cv::Mat& frame, cv::CascadeClassifier& face_cascade);
public:
    bool is_start_save;//是否开始视频采集
    UsbCamera(int frames_num, int fps, std::string video_save_path, std::string camera_dev);
    void FatigueDrivingDetectionThread();
    bool is_fatigue_driving() {return is_longtime_fatigue_driving_;}
    bool save_video(std::string name, std::string saving_path); 
    // bool StartDetectionThread();
    // bool StartSavingThread();
    void  setVedioName(std::string name);
    friend void *thread_save_video(void* argv);
    ~UsbCamera();
};


// void *thread_fun_write(char* sendbuf);
// void *thread_fun_read(char* recvbuf);

#endif


