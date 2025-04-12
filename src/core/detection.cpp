//! for object detection logic

// detection.cpp
#include "detection.h"

Detection::Detection(const std::string &modelPath, const std::string &classesPath, float confThresh, float nmsThresh)
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

bool Detection::detect(const cv::Mat &frame, std::vector<cv::Rect> &boxes, std::vector<int> &classIds, std::vector<float> &confidences)
{
    // Original dimensions of the image
    int original_width = frame.cols;
    int original_height = frame.rows;

    // Network input dimensions
    int input_width = 640;
    int input_height = 640;

    // Create blob from image
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(input_width, input_height), cv::Scalar(0, 0, 0), true, false);

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
    std::cout << "Output shape: " << outputs[0].size[0] << "x" << outputs[0].size[1] << "x" << outputs[0].size[2] << std::endl;

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

    // Calculate scale factors for mapping back to original image
    float x_factor = static_cast<float>(original_width) / input_width;
    float y_factor = static_cast<float>(original_height) / input_height;

    // Format for YOLOv11 output - adjust according to actual output format
    cv::Mat output = outputs[0];
    cv::Mat transposed = output.reshape(1, output.size[1]);
    transposed = transposed.t();

    // Process detections
    for (int i = 0; i < transposed.rows; ++i)
    {
        float *row_ptr = transposed.ptr<float>(i);

        // Class scores start at index 4
        int num_classes = classNames.size();
        cv::Mat scores(1, num_classes, CV_32F, row_ptr + 4);
        cv::Point classIdPoint;
        double maxScore;
        cv::minMaxLoc(scores, nullptr, &maxScore, nullptr, &classIdPoint);

        if (maxScore > confThreshold)
        {
            // Get raw box coordinates from model output
            float x = row_ptr[0];
            float y = row_ptr[1];
            float w = row_ptr[2];
            float h = row_ptr[3];

            // Debug print the raw values
            std::cout << "Raw box values from model: " << x << ", " << y << ", " << w << ", " << h << std::endl;

            // YOLOv11 may output coordinates in different formats:
            // 1. Normalized (0-1)
            // 2. Relative to input size (0-640)
            // 3. Direct pixel values for original image

            // Assuming normalized coordinates (0-1) - most common for YOLO models
            if (x <= 1.0 && y <= 1.0 && w <= 1.0 && h <= 1.0)
            {
                std::cout << "Detected normalized coordinates (0-1)" << std::endl;
                // Scale to original image size
                x *= original_width;
                y *= original_height;
                w *= original_width;
                h *= original_height;
            }
            // If coordinates are relative to input size (0-640)
            else if (x <= input_width && y <= input_height)
            {
                std::cout << "Detected input-relative coordinates (0-640)" << std::endl;
                // Scale to original image size
                x = x * x_factor;
                y = y * y_factor;
                w = w * x_factor;
                h = h * y_factor;
            }
            // If coordinates are already in original image pixels, no scaling needed
            else
            {
                std::cout << "Detected direct pixel coordinates" << std::endl;
            }

            // Convert center-width-height to top-left format
            int left = static_cast<int>(x - w / 2);
            int top = static_cast<int>(y - h / 2);
            int width = static_cast<int>(w);
            int height = static_cast<int>(h);

            // Ensure box is within image boundaries
            left = std::max(0, std::min(original_width - 1, left));
            top = std::max(0, std::min(original_height - 1, top));
            width = std::min(original_width - left, width);
            height = std::min(original_height - top, height);

            // Only add valid boxes
            if (width > 0 && height > 0)
            {
                boxes.push_back(cv::Rect(left, top, width, height));
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

const std::vector<std::string> &Detection::getClassNames() const
{
    return classNames;
}