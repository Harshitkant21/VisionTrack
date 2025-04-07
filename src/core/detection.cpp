//! for object detection logic

// detection.cpp
#include "detection.h"

YoloDetector::YoloDetector(const std::string &modelPath, const std::string &classesPath,
                           float confThresh, float nmsThresh)
{
    // Load the model
    try
    {
        net = cv::dnn::readNetFromONNX(modelPath);
        std::cout << "Model loaded successfully: " << modelPath << std::endl;
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "Error loading model: " << e.what() << std::endl;
        throw;
    }

    // Set backend and target
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // Load class names
    std::ifstream ifs(classesPath);
    if (ifs.is_open())
    {
        std::string line;
        while (std::getline(ifs, line))
        {
            classNames.push_back(line);
        }
        std::cout << "Loaded " << classNames.size() << " class names" << std::endl;
    }
    else
    {
        std::cerr << "Error opening classes file: " << classesPath << std::endl;
    }

    confThreshold = confThresh;
    nmsThreshold = nmsThresh;
}

bool YoloDetector::detect(const cv::Mat &frame, std::vector<cv::Rect> &boxes,
                          std::vector<int> &classIds, std::vector<float> &confidences)
{
    // Create blob from image
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(640, 640),
                                          cv::Scalar(0, 0, 0), true, false);

    // Set input
    net.setInput(blob);

    // Forward pass
    std::vector<cv::Mat> outputs;
    try
    {
        net.forward(outputs, net.getUnconnectedOutLayersNames());
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "Error during inference: " << e.what() << std::endl;
        return false;
    }

    // Debug output shape
    std::cout << "Output shape: " << outputs[0].size[0] << "x"
              << outputs[0].size[1] << "x" << outputs[0].size[2] << std::endl;

    // Process output
    if (outputs.empty() || outputs[0].empty())
    {
        std::cerr << "Empty output from network" << std::endl;
        return false;
    }

    // Clear vectors
    boxes.clear();
    classIds.clear();
    confidences.clear();

    // Format for YOLOv8 output: [1, 84, 8400] - transpose to [8400, 84]
    cv::Mat output = outputs[0];
    cv::Mat transposed = output.reshape(1, output.size[1]);
    transposed = transposed.t();

    // Process detections
    for (int i = 0; i < transposed.rows; ++i)
    {
        float *row_ptr = transposed.ptr<float>(i);

        // Class scores start at index 4
        cv::Mat scores(1, classNames.size(), CV_32F, row_ptr + 4);
        cv::Point classIdPoint;
        double maxScore;
        cv::minMaxLoc(scores, nullptr, &maxScore, nullptr, &classIdPoint);

        if (maxScore > confThreshold)
        {
            // Extract bounding box - assuming direct pixel coordinates
            float x_center = row_ptr[0]; // center x in pixels
            float y_center = row_ptr[1]; // center y in pixels
            float width = row_ptr[2];    // width in pixels
            float height = row_ptr[3];   // height in pixels

            // Print values for debugging
            std::cout << "Original box values: " << x_center << ", " << y_center << ", "
                      << width << ", " << height << std::endl;

            // Convert to top-left corner format
            int left = static_cast<int>(x_center - width / 2);
            int top = static_cast<int>(y_center - height / 2);
            int box_width = static_cast<int>(width);
            int box_height = static_cast<int>(height);

            // Ensure box is within image boundaries
            left = std::max(0, std::min(frame.cols - 1, left));
            top = std::max(0, std::min(frame.rows - 1, top));
            box_width = std::min(frame.cols - left, box_width);
            box_height = std::min(frame.rows - top, box_height);

            // Only add valid boxes
            if (box_width > 0 && box_height > 0)
            {
                boxes.push_back(cv::Rect(left, top, box_width, box_height));
                classIds.push_back(classIdPoint.x);
                confidences.push_back(static_cast<float>(maxScore));
            }
        }
    }

    // Apply NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    // Filter results
    std::vector<cv::Rect> nmsBoxes;
    std::vector<int> nmsClassIds;
    std::vector<float> nmsConfidences;

    for (int idx : indices)
    {
        nmsBoxes.push_back(boxes[idx]);
        nmsClassIds.push_back(classIds[idx]);
        nmsConfidences.push_back(confidences[idx]);
    }

    boxes = nmsBoxes;
    classIds = nmsClassIds;
    confidences = nmsConfidences;

    std::cout << "Detected " << boxes.size() << " objects" << std::endl;
    return !boxes.empty();
}

const std::vector<std::string> &YoloDetector::getClassNames() const
{
    return classNames;
}