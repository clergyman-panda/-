#include"usbcamera.h"
#include<thread>
#include<mutex>
#include"opencv2/opencv.hpp"
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/objdetect.hpp> //对象检测头文件

extern char savebuf[128];
extern char localbuf[128];
extern pthread_mutex_t save_mutex;
UsbCamera::UsbCamera(int frames_num, int fps, std::string video_save_path, 
                    std::string camera_dev){
    frames_num_ = frames_num;
    fps_ = fps;
    video_save_path_ = video_save_path;
    camera_dev_ = camera_dev;
    cache_frames_ptr_ = new std::vector<cv::Mat>[frames_num]();
    is_start_save = false;
    // last_frames_ptr_ = new std::vector<cv::Mat>[frames_num]();
    // next_frames_ptr_ = new std::vector<cv::Mat>[frames_num]();
    is_fatigue_driving_ = false;
    is_longtime_fatigue_driving_ = false;
    frame_index_ = 0;
    cv::Mat init_mat(2, 2, CV_8UC3, cv::Scalar(0,255,0));

}

UsbCamera::~UsbCamera() {
    delete[] cache_frames_ptr_;
}

void UsbCamera::FatigueDrivingDetectionThread() {
    cv::VideoCapture cam_campture(camera_dev_);
    if(cam_campture.isOpened()) {
        std::cout << "Open camera: " << camera_dev_ << "successfully!\n"; 
    } else {
        std::cerr << "Open the camera: " << camera_dev_ << "failed!\n";
        return;
    }
    cv::Mat frame;
    int consecutive_fatigue_num = 0;
    int not_fatigue_num = 0;
    cam_campture.read(frame);
    cv::putText(frame, "init frame", cv::Point2d(400, 430), 
            cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255,0,0), 2);
    //初始化缓存
    for(int i=0; i<frames_num_; i++) {

        cache_frames_ptr_->push_back(frame.clone());
    }
    // cv::imshow("init",cache_frames_ptr_->at(20));
    // cv::waitKey(0);
    cv::CascadeClassifier face_cascade;
    //从文件加载分类器(已经训练好的模型)
	face_cascade.load("/home/openailab/Test/test_usbcam/config/haarcascade_frontalface_default.xml");
	//检测文件是否加载成功
	if (face_cascade.empty()) { 
        std::cerr << "XML file not loaded" << std::endl;
        return; 
    }
    // cv::Size img_size = Size((int)cam_capture.get(CV_CAP_PROP_FRAME_WIDTH),
	//                             (int)cam_capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    std::mutex cache_lock;
    while(true) {
        // int frame_count = 0;
        // while(frame_count++ < 5){
            cam_campture.read(frame);
        // }
        
        int detection_flag = FatigueDrivingDetec(frame, face_cascade);
        is_fatigue_driving_ = (detection_flag == 1);
        if(detection_flag != -1) {
            if(is_fatigue_driving_) {
                consecutive_fatigue_num++;
                not_fatigue_num = 0;
            } else {
                not_fatigue_num++;
                consecutive_fatigue_num = 0;
            }
            //判断是否是长时间的疲劳状态
            if(consecutive_fatigue_num >= 30) {
                not_fatigue_num = 0;
                is_longtime_fatigue_driving_ = true;
            } else if(not_fatigue_num >= 30 && is_longtime_fatigue_driving_) {
                consecutive_fatigue_num = 0;
                is_longtime_fatigue_driving_ = false;
            }
            cache_lock.lock();
            cache_frames_ptr_->at(frame_index_) = frame.clone();
            cache_lock.unlock();
            // cv::imwrite("/home/khz/tmp/" + std::to_string(frame_index_)+".jpg", frame);
            frame_index_ = frame_index_ < frames_num_-1 ? frame_index_+1 : 0;
        }

        cv::imshow("Image", frame);
        cv::waitKey(1);    
                
    }
    
}
int UsbCamera::FatigueDrivingDetec(cv::Mat& frame, cv::CascadeClassifier& face_cascade) {
    bool is_out_of_boundry = false;
    cv::Point2d boundry_tl(170, 110);
    cv::Point2d boundry_br(470, 350);
    // cv::Point2d boundry_tl(0, 0);
    // cv::Point2d boundry_br(700, 700);
    cv::Point2d face_tl;
    cv::Point2d face_br;
    int max_area = 0;
    cv::Scalar face_boundry_color(0, 255, 0);
    cv::rectangle(frame, boundry_tl, boundry_br, cv::Scalar(0, 255, 0), 2);
	std::vector<cv::Rect> faces;//定义用于接收检测结果的
    //在输入图像中检测不同大小的对象。检测到的对象将以矩形列表的形式返回。
    //通过增加最近邻的值可以消除误报，但是过大将会导致漏检测
	face_cascade.detectMultiScale(frame, faces, 1.1, 2.5/*最近邻*/);
	for (int i = 0; i < faces.size(); i++) {
        // if(faces[i].area() < 20000 || faces[i].area() > 30000) continue;
        if(faces[i].area() > max_area) {
            max_area = faces[i].area();
            face_br = faces[i].br();
            face_tl = faces[i].tl();
        }
	}
    //检测异常处理 

    if(faces.empty() || max_area < 5000) {
        //printf("erro:-1\r\n");
        return -1;
    }
    if(face_tl.x < boundry_tl.x || face_br.x > boundry_br.x
        || face_br.y > boundry_br.y) {
        face_boundry_color = cv::Scalar(0, 0, 255);
        is_out_of_boundry = true;     
    }
    
    pthread_mutex_lock(&save_mutex);
    cv::putText(frame, savebuf, cv::Point2d(10, 10), 
             cv::FONT_HERSHEY_PLAIN, 1, face_boundry_color, 2);
    cv::putText(frame, localbuf, cv::Point2d(10, 30), 
             cv::FONT_HERSHEY_PLAIN, 1, face_boundry_color, 2);
    pthread_mutex_unlock(&save_mutex);
    cv::rectangle(frame, face_tl,face_br, face_boundry_color, 3);
    cv::putText(frame, std::to_string(frame_index_), cv::Point2d(400, 400), 
            cv::FONT_HERSHEY_PLAIN, 2, face_boundry_color, 2);
    if(is_out_of_boundry) {
        cv::putText(frame, "Abnormal Driving", cv::Point2d(400, 430), 
            cv::FONT_HERSHEY_PLAIN, 2, face_boundry_color, 2);
    } else {
        cv::putText(frame, "Normal Driving", cv::Point2d(400, 430), 
            cv::FONT_HERSHEY_PLAIN, 2, face_boundry_color, 2);
    }
    
    // cv::imwrite("/home/khz/tmp/" + std::to_string(frame_index_)+".jpg", frame);
    return (is_out_of_boundry == true ? 1 : 0);
}

bool UsbCamera::save_video(std::string name, std::string saving_path) {
    if(saving_path.empty()) {
        saving_path = video_save_path_;
    }
    int cur_frame_index = frame_index_;
    std::vector<cv::Mat> *last_frames = new std::vector<cv::Mat>[frames_num_];
    std::vector<cv::Mat> *next_frames = new std::vector<cv::Mat>[frames_num_];
    std::mutex cache_lock;
    cache_lock.lock();  
    *last_frames = *cache_frames_ptr_; 
    cache_lock.unlock();
    std::cout << "wating next frames...\n";
    while(frame_index_ != cur_frame_index - 1) {}
    cache_lock.lock();
    *next_frames = *cache_frames_ptr_;
    cache_lock.unlock();
    // std::cout << "tmp frame size: " << tmp_frame.size() << std::endl;  
    cv::Mat tmp_frame = last_frames->at(cur_frame_index);
    // std::cout << "tmp frame size: " << tmp_frame.size() << std::endl;
    std::cout << "start saving video......\n";
    cv::Size S = tmp_frame.size();
    cv::VideoWriter writer(saving_path + name, CV_FOURCC('M', 'J', 'P', 'G'), fps_, S, true);
    
    for(int i=cur_frame_index+1; i != cur_frame_index; i++) {
        i = i < frames_num_ ? i : 0;
        writer.write(last_frames->at(i));
    }
    // writer.write(last_frames[cur_frame_index]);
    for(int i=cur_frame_index+1; i != cur_frame_index; i++) {
        i = i < frames_num_ ? i : 0;
        writer.write(next_frames->at(i));
    }
    // writer.write(next_frames[cur_frame_index]);
    //is_start_save = false;
    writer.release();
    delete[] last_frames;
    delete[] next_frames;
    std::cout << "Video has been saved  successfully!\n";
    return true;
}


void UsbCamera::setVedioName(std::string name) {
    video_save_name_ = name;
}

void *thread_save_video(void* argv) {
    
    UsbCamera* camera = (UsbCamera*)argv;
    printf("vedioName:%s\r\n", camera->video_save_name_);
    camera->save_video(camera->video_save_name_, camera->video_save_path_);
    //camera->is_start_save = false;

    Upload upload(serverIP, serverPort);
    upload.connectToServer();
    std::cout << "成功连接至服务器\n";

    upload.uploadFile((camera->video_save_path_+camera->video_save_name_).c_str());
    camera->is_start_save = false;

    return NULL;
}
