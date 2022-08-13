/**
 * @file main.cpp
 * @brief 
 * 
 * Copyright (c) 2021 Sipeed team
 * 
 * 
 */
extern "C" {
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include "fbviewer.h"
#include "label.h"
#include <sys/wait.h>
#include <sys/types.h>
}

#include "standard_api.h"
#include <iostream>
#include <sys/time.h>
#include "upload.h"
#include "pthread.h"

#include "opencv2/opencv.hpp"
using namespace cv;

#define DBG_LINE() printf("###L%d\r\n", __LINE__)

int label_oft = 1;

typedef struct {
    int index;
    int8_t val;
} int8_data_t;

typedef struct {
    int index;
    uint8_t val;
} uint8_data_t;

int uint8_comp_down(const void*p1, const void*p2) {
	//<0, 元素1排在元素2之前；即降序
	int tmp = (((uint8_data_t*)p2)->val) - (((uint8_data_t*)p1)->val);
	return tmp;  
}

int int8_comp_down(const void*p1, const void*p2) {
	//<0, 元素1排在元素2之前；即降序
	int tmp = (((int8_data_t*)p2)->val) - (((int8_data_t*)p1)->val);
	return tmp;  
}


static void decode_result_int8(int8_t *result, uint32_t size, int* label_idx, int* prob)
{
    int8_data_t* buf = (int8_data_t*)malloc(sizeof(int8_data_t)*size);
    if(buf == NULL) return;
    for(int i=0; i < size; i++) {
        buf[i].index = i;
        buf[i].val = result[i];
    }
    qsort(buf, size, sizeof(int8_data_t), int8_comp_down);
    printf("Decode Result:\r\n");
    for(int i=0; i < size; i++) {
        printf("buf[%d]val:%d , buf[%d].idx:%d\n",i , buf[i].val, i, buf[i].index);   
    }
    // printf("size:%d", size);
    //for(int i=0; i < 5; i++) {
    // printf("    %d: class %4d, prob %3d; label: %s\r\n", 0, buf[0].index, buf[0].val, labels[buf[0].index-label_oft]);
    //}
    *label_idx = buf[0].index;
    *prob = buf[0].val;
    free(buf);
    return;
}

static void decode_result_uint8(uint8_t *result, uint32_t size, int* label_idx, int* prob)
{
    uint8_data_t* buf = (uint8_data_t*)malloc(sizeof(uint8_data_t)*size);
    if(buf == NULL) return;
    for(int i=0; i < size; i++) {
        buf[i].index = i;
        buf[i].val = result[i];
    }
    qsort(buf, size, sizeof(uint8_data_t), uint8_comp_down);
    printf("size:%d\n",size);
    printf("Decode Result:\r\n");
    for(int i=0; i < size; i++) {
        printf("buf[%d]val:%d , buf[%d].idx:%d\n",i , buf[i].val, i, buf[i].index);   
    }
    // for(int i=0; i < 5; i++) {
        // if (!buf[i].index && !buf[i].val) continue;
        // printf("    %d: class %4d, prob %3u; label: %s\r\n", 0, buf[0].index, buf[0].val, labels[buf[0].index-label_oft]);
    // }
    *label_idx = buf[0].index;
    *prob = buf[0].val;
    free(buf);
    return;
}


cv::VideoCapture capture(0);

int init_cam(void)
{
    int x,y;
    getCurrentRes(&x, &y);
    printf("LCD width is %d, height is %d\n", x, y);
    cv::Mat img;
    VideoCapture cap;
    cap.isOpened();
    
    if(!capture.isOpened())
    {
        std::cout<<"video not open."<<std::endl;
        return 1;
    }
    //get default video fps, set fps to 30fps
    double rate = capture.get(CAP_PROP_FPS);
    printf("rate is %lf\n", rate);
    capture.set(CAP_PROP_FPS, 30);
    rate = capture.get(CAP_PROP_FPS);
    printf("rate is %lf\n", rate);
    //get default video frame info
    double frame_width = capture.get(CAP_PROP_FRAME_WIDTH);
    double frame_height = capture.get(CAP_PROP_FRAME_HEIGHT);
    printf("frame_width is %lf, frame_height is %lf\n", frame_width, frame_height);
    //set video frame size to QVGA (then we crop to 224x224)
    frame_width = 320;
    frame_height = 240;
    if(!capture.set(CAP_PROP_FRAME_WIDTH,frame_width))
    {
        printf("set width failed\n");
        return 2;
    }
    if(!capture.set(CAP_PROP_FRAME_HEIGHT, frame_height))
    {
        printf("set width failed\n");
        return 3;
    } 
    return 0;
}

int init_graph(char* file_model, aipu_ctx_handle_t ** ctx, aipu_graph_desc_t* gdesc, aipu_buffer_alloc_info_t* info)
{
    const char* status_msg =NULL;
    aipu_status_t status = AIPU_STATUS_SUCCESS;
    int ret = 0;
    //Step1: init ctx handle
    status = AIPU_init_ctx(ctx);       
    if (status != AIPU_STATUS_SUCCESS) {
        AIPU_get_status_msg(status, &status_msg);
        printf("[DEMO ERROR] AIPU_init_ctx: %s\n", status_msg);
        ret = -1;
        //goto out;
    }

    //Step2: load graph
    status = AIPU_load_graph_helper(*ctx, file_model, gdesc);
    if (status != AIPU_STATUS_SUCCESS) {
        AIPU_get_status_msg(status, &status_msg);
        printf("[DEMO ERROR] AIPU_load_graph_helper: %s\n", status_msg);
        ret = -2;
        //goto deinit_ctx;
    }
    printf("[DEMO INFO] AIPU load graph successfully.\n");

    //Step3: alloc tensor buffers
    status = AIPU_alloc_tensor_buffers(*ctx, gdesc, info);
    if (status != AIPU_STATUS_SUCCESS) {
        AIPU_get_status_msg(status, &status_msg);
        printf("[DEMO ERROR] AIPU_alloc_tensor_buffers: %s\n", status_msg);
        ret = -3;
        //goto unload_graph;
    }
    printf("init graph OK!!\n");
    
    return ret;
}

int cap_img(Mat* lcd_frame, Mat* ai_frame)
{    
    Rect roi(40, 0, 240/4*3, 240);  //16/9 -> 4/3  
    Rect input_roi(8, 8, 224, 224);
    Size dsize = Size(240, 240);
    if(!capture.read(*lcd_frame))
    {
        printf("no video frame\r\n");
        return -1;
    }
    *lcd_frame = (*lcd_frame)(roi).clone();
    rotate(*lcd_frame, *lcd_frame, ROTATE_180);
    resize(*lcd_frame, *lcd_frame, dsize);
    cvtColor(*lcd_frame, *lcd_frame, COLOR_BGR2RGB);
    *ai_frame = ((*lcd_frame)(input_roi).clone() + Scalar(-0.485, -0.456, -0.406))/0.226;
    std::cout << "ai_frame : " << ai_frame->size() << std::endl;
    imwrite("/root/1.bmp", (*lcd_frame)(input_roi).clone());
    
    return 0;
}

int infer_img(Mat* ai_frame, aipu_ctx_handle_t ** ctx, aipu_graph_desc_t* gdesc, aipu_buffer_alloc_info_t* info, int signed_flag, int* label_idx, int* label_prob)
{
    uint32_t job_id=0;
    const char* status_msg =NULL;
    int32_t time_out=-1;
    bool finish_job_successfully = true;
    aipu_status_t status = AIPU_STATUS_SUCCESS;
    int ret = 0;
    
    memcpy(info->inputs.tensors[0].va, ai_frame->data, info->inputs.tensors[0].size);
    status = AIPU_create_job(*ctx, gdesc, info->handle, &job_id);
    if (status != AIPU_STATUS_SUCCESS) {
        AIPU_get_status_msg(status, &status_msg);
        printf("[DEMO ERROR] AIPU_create_job: %s\n", status_msg);
        ret = -1;
        //goto free_tensor_buffers;
    }
    status = AIPU_finish_job(*ctx, job_id, time_out);
    if (status != AIPU_STATUS_SUCCESS) {
        AIPU_get_status_msg(status, &status_msg);
        printf("[DEMO ERROR] AIPU_finish_job: %s\n", status_msg);
        finish_job_successfully = false;
    } else {
        finish_job_successfully = true;
    }

    if (finish_job_successfully) {
        int8_t *result = (int8_t *)info->outputs.tensors[0].va;
        uint32_t size = info->outputs.tensors[0].size;
        float val0 = result[0];
        float val1 = result[1];
        float val = val0*0.1  + val1*0.9;
        *label_prob = (int)val;
        if(val1 > 80) {
            *label_idx = 1;
        } else {
            *label_idx = 0;
        }
        // printf("tensor size:%d\n", size);
        printf("val0:[%f]  val1:[%f] val:[%f]\n",val0, val1, val);
        // if(signed_flag == 0) {
        //     decode_result_uint8((uint8_t*)result, size, label_idx, label_prob);
        // } else {
        //     decode_result_int8(result, size, label_idx, label_prob);
        // }
    }

    status = AIPU_clean_job(*ctx, job_id);
    if (status != AIPU_STATUS_SUCCESS) {
        AIPU_get_status_msg(status, &status_msg);
        printf("[TEST ERROR] AIPU_clean_job: %s\n", status_msg);
        ret = -2;
        //goto free_tensor_buffers;
    }
    return ret;
}


float cal_fps(struct timeval start, struct timeval end)
{
    struct timeval interval;
    if (end.tv_usec >= start.tv_usec) {
        interval.tv_usec = end.tv_usec - start.tv_usec;
        interval.tv_sec = end.tv_sec - start.tv_sec;
    } else  {
        interval.tv_usec = 1000000 + end.tv_usec - start.tv_usec;
        interval.tv_sec = end.tv_sec - 1 - start.tv_sec;
    }
    float fps = 1000000.0 / interval.tv_usec;
    return fps;
}

volatile int exit_flag = 0;
void my_handler(int s){
    printf("Caught signal %d\n",s);
    exit_flag = 1;
    return;
}
bool is_start_send=false;
cv::Mat lcd_frame;
void *send_image(void* argv)
{
    time_t now;
    struct tm *p_tm;
    char buf[256]={0};
    time(&now);
    p_tm = localtime(&now);  //将time_t时间类型转换为 struct tm 时间结构体类型
    sprintf(buf, "%4d-%02d-%02d %02d %02d %02d.png", p_tm->tm_year+1900, p_tm->tm_mon+1, p_tm->tm_mday,p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);
    imwrite(buf, lcd_frame);
    Upload* image=(Upload*)argv;
    image->uploadFile(buf);
    printf("image send successful\r\n");
    is_start_send=false;
    return NULL;
}

int main(int argc, char *argv[])
{
    int ret = 0;
    uint32_t job_id=0;
    int32_t time_out=-1;
    bool finish_job_successfully = true;
    int model_inw, model_inh, model_inch, model_outw, model_outh, model_outch, img_size;    
    int8_t* bmpbuf;
    pthread_t image_tid;
    cv::Mat ai_frame;
    int label_idx, label_prob;
    struct timeval start, end;
    

    signal(SIGINT, my_handler); 
    
   Upload upload("192.168.1.104", 9999); 
   upload.connectToServer();


    printf("Zhouyi Cam test program: \r\n");
    printf("Usage: \r\n");
    printf("    ./zhouyi aipu.bin signed [label_oft]\r\n");
    printf("    signed=0, uint8 output; =1, int8 output\r\n");
    printf("    real_label_idx = predict_idx-label_oft, \r\n");
    printf("    NOTE: default cal with 224x224\r\n");

    aipu_ctx_handle_t * ctx = NULL;
    aipu_status_t status = AIPU_STATUS_SUCCESS;
    const char* status_msg =NULL;
    aipu_graph_desc_t gdesc;
    aipu_buffer_alloc_info_t info;

    //Step 0: parse input argv
	if(argc < 3) {
		printf("argc=%d error\r\n", argc);
        return -1;
	}
    if(argc >3) label_oft = atoi(argv[3]);
    char* file_model= argv[1];
    int signed_flag = atoi(argv[2]);
    
    //Step 1: set USB camera
    ret = init_cam();DBG_LINE();
    if(ret != 0) {
        printf("[DEMO ERROR] init_cam err: %s\n", ret);
        goto out;
    }
    
    //Step 2: init model graph
    ret = init_graph(file_model, &ctx, &gdesc, &info);DBG_LINE();
    if(ret == -1) goto out;
    else if(ret == -2) goto deinit_ctx;
    else if(ret == -3) goto unload_graph;
     
    //MAIN LOOP
    while(!exit_flag)
    {
        //1. cap cam img
        if(cap_img(&lcd_frame, &ai_frame) != 0) {
            break;
        }
        //2. infer cam img, get label
        gettimeofday(&start, NULL);
        ret = infer_img(&ai_frame, &ctx, &gdesc, &info, signed_flag, &label_idx, &label_prob);
        if(ret != 0) goto free_tensor_buffers;

        gettimeofday(&end, NULL);
        //3. draw lcd
        // putText(lcd_frame, labels[label_idx-label_oft], Point(0, 224), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255,0,0), 2);
        if(label_idx == 0) {
            putText(lcd_frame, "No-Accident", Point(0, 224), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255,0,0), 2);
        } else if(label_idx == 1) {
       
       	    putText(lcd_frame, "Accident", Point(0, 224), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255,0,0), 2);
            if(is_start_send==false)
	    
	    {
	       is_start_send=true;
               pthread_create(&image_tid, NULL, send_image,(void*)&upload);
               pthread_detach(image_tid);
	    }
        }
        float fps = cal_fps(start, end);
        char fps_str[16];
        sprintf(fps_str, "%.1ffps", fps);
        putText(lcd_frame, fps_str, Point(0, 16), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255,0,0), 2);
        fb_display(lcd_frame.data, 0, 240, 240, 0, 0, 0, 0);
        
    }

free_tensor_buffers:
    status = AIPU_free_tensor_buffers(ctx, info.handle);
    if (status != AIPU_STATUS_SUCCESS) {
        AIPU_get_status_msg(status, &status_msg);
        printf("[DEMO ERROR] AIPU_free_tensor_buffers: %s\n", status_msg);
        ret = -1;
    }
unload_graph:
    status = AIPU_unload_graph(ctx, &gdesc);
    if (status != AIPU_STATUS_SUCCESS) {
        AIPU_get_status_msg(status, &status_msg);
        printf("[DEMO ERROR] AIPU_unload_graph: %s\n", status_msg);
        ret = -1;
    }
deinit_ctx:
    status = AIPU_deinit_ctx(ctx);
    if (status != AIPU_STATUS_SUCCESS) {
        AIPU_get_status_msg(status, &status_msg);
        printf("[DEMO ERROR] AIPU_deinit_ctx: %s\n", status_msg);
        ret = -1;
    }

out:

    return ret;
}
