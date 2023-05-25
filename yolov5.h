#ifndef YOLOV5_H
#define YOLOV5_H
#include <iostream>
#include <chrono>
#include <cmath>
#include "cuda_utils.h"
#include "logging.h"

#include "utils.h"
#include "calibrator.h"
#include "preprocess.h"

#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1
#define MAX_IMAGE_INPUT_SIZE_THRESH 3000 * 3000 // ensure it exceed the maximum size in the input images !


struct Net_config
{
    float gd;//engine threshold
    float gw;//engine threshold
    const char* netname;
};

class Yolov5
{
public:
	Yolov5();
	~Yolov5();
	
	void Initialize(const char* model_path,int num);
	int Detecting(cv::Mat& frame,std::vector<cv::Rect>& Boxes,std::vector<const char*>& ClassLables);

private:
    char netname[20] = { 0 };
    float gd = 0.0f, gw = 0.0f;
    const char* classes[80] = { "Cuboid Block", "Round Block", "Carton" };
    Net_config yolo_nets[4] = {
        {0.33,0.50,"yolov5s"},
        {0.67,0.75,"yolov5m"},
        {1.00,1.00,"yolov5l"},
        {1.33,1.25,"yolov5x"}
    };

    //int CLASS_NUM = 80;
    size_t size = 0;
    //char* trtModelStream = nullptr;
    float prob[BATCH_SIZE * 6001];
    float* buffers[2] = { 0 };
    int inputIndex = 0;
    int outputIndex = 0;

    nvinfer1::IRuntime* runtime;
    nvinfer1::ICudaEngine* engine;
    nvinfer1::IExecutionContext* context;
    cudaStream_t stream;

    uint8_t* img_host = nullptr;
    uint8_t* img_device = nullptr;
};
#endif  // YOLOV5_H