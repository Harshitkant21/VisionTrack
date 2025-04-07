//! for main file

#include "detection.h"
#include "config.h"
#include <iostream>

int main(int argc, char **argv)
{
    // === 1. Parse arguments ===
    std::string configFile = "config.txt";
    std::string videoPath = "";
    int camIndex = -1;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg.find(".txt") != std::string::npos || arg.find(".cfg") != std::string::npos)
        {
            configFile = arg;
        }
        else if (arg.find(".mp4") != std::string::npos || arg.find(".avi") != std::string::npos)
        {
            videoPath = arg;
        }
        else
        {
            try
            {
                camIndex = std::stoi(arg);
            }
            catch (...)
            {
                videoPath = arg;
            }
        }
    }

    // === 2. Load configuration ===
    Config config;
    std::vector<std::string> searchPaths = {
        "./", "../", "../../",
        "./config/", "../config/", "../../config/"};

    bool configLoaded = false;
    for (const auto &path : searchPaths)
    {
        if (config.loadFromFile(path + configFile))
        {
            std::cout << "Loaded configuration from " << path + configFile << std::endl;
            configLoaded = true;
            break;
        }
    }

    if (!configLoaded)
    {
        std::cout << "Using default configuration" << std::endl;
    }

    // === 3. Get values from config ===
    std::string modelPath = config.get("model_path");
    std::string classesPath = config.get("classes_path");
    float confThreshold = config.getFloat("confidence_threshold", 0.25f);
    float nmsThreshold = config.getFloat("nms_threshold", 0.45f);

    // === 4. Initialize detector ===
    YoloDetector detector(modelPath, classesPath, confThreshold, nmsThreshold);

    // === 5. Initialize video capture ===
    cv::VideoCapture cap;

    if (!videoPath.empty())
    {
        cap.open(videoPath);
        std::cout << "Opened video file: " << videoPath << std::endl;
    }
    else
    {
        if (camIndex == -1)
            camIndex = config.getInt("default_source", 0);

        cap.open(camIndex);
        std::cout << "Opened webcam index: " << camIndex << std::endl;
    }

    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open video source" << std::endl;
        return -1;
    }

    // === 6. Main loop ===
    cv::Mat frame;
    std::vector<cv::Rect> boxes;
    std::vector<int> classIds;
    std::vector<float> confidences;

    while (true)
    {
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "End of video stream" << std::endl;
            break;
        }

        // Detect objects
        detector.detect(frame, boxes, classIds, confidences);

        // Draw results
        for (size_t i = 0; i < boxes.size(); ++i)
        {
            cv::rectangle(frame, boxes[i], cv::Scalar(0, 255, 0), 2);

            std::string label;
            if (classIds[i] < detector.getClassNames().size())
            {
                label = detector.getClassNames()[classIds[i]] + ": " +
                        std::to_string(static_cast<int>(confidences[i] * 100)) + "%";
            }
            else
            {
                label = "Unknown: " + std::to_string(static_cast<int>(confidences[i] * 100)) + "%";
            }

            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::rectangle(frame,
                          cv::Point(boxes[i].x, boxes[i].y - labelSize.height - 10),
                          cv::Point(boxes[i].x + labelSize.width, boxes[i].y),
                          cv::Scalar(0, 255, 0), cv::FILLED);
            cv::putText(frame, label, cv::Point(boxes[i].x, boxes[i].y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }

        // Debug output
        for (size_t i = 0; i < boxes.size(); ++i)
        {
            std::cout << "Box " << i << ": " << boxes[i].x << ", " << boxes[i].y
                      << ", " << boxes[i].width << ", " << boxes[i].height
                      << " Class: " << classIds[i]
                      << " Conf: " << confidences[i] << std::endl;
        }

        cv::imshow("VisionTrack", frame);

        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
