//! for object detection logic header file

// detection.h
#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>

class YoloDetector
{
private:
    cv::dnn::Net net;
    std::vector<std::string> classNames;
    float confThreshold;
    float nmsThreshold;

public:
    YoloDetector(const std::string &modelPath, const std::string &classesPath,
                 float confThresh = 0.25, float nmsThresh = 0.45);

    bool detect(const cv::Mat &frame, std::vector<cv::Rect> &boxes,
                std::vector<int> &classIds, std::vector<float> &confidences);

    const std::vector<std::string> &getClassNames() const;
};