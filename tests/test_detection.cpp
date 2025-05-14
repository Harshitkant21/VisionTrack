//! for testing detection

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "../src/core/detection.h"

class DetectionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Use test model path
        modelPath = "D:/Projects/VisionTrack/models/yolov8n.onnx";
        classesPath = "D:/Projects/VisionTrack/models/coco.names";
        detector = nullptr;

        // Verify files exist before testing
        std::ifstream modelFile(modelPath);
        std::ifstream classesFile(classesPath);
        if (!modelFile.good() || !classesFile.good())
        {
            std::cerr << "Test files not found at:" << std::endl;
            std::cerr << "Model: " << modelPath << std::endl;
            std::cerr << "Classes: " << classesPath << std::endl;
            GTEST_SKIP();
        }
    }

    void TearDown() override
    {
        delete detector;
    }

    std::string modelPath;
    std::string classesPath;
    Detection *detector;
};

TEST_F(DetectionTest, InitializationTest)
{
    try
    {
        detector = new Detection(modelPath, classesPath, 0.25f, 0.45f);
        EXPECT_FALSE(detector->getClassNames().empty());
    }
    catch (const std::exception &e)
    {
        FAIL() << "Exception thrown: " << e.what();
    }
}

TEST_F(DetectionTest, InvalidModelPath)
{
    EXPECT_THROW({ Detection invalidDetector("invalid_path.onnx", classesPath); }, std::runtime_error); // Changed from cv::Exception to std::runtime_error
}

TEST_F(DetectionTest, DetectionOnEmptyFrame)
{
    detector = new Detection(modelPath, classesPath);
    cv::Mat emptyFrame = cv::Mat::zeros(640, 640, CV_8UC3); // Create valid but empty frame
    std::vector<cv::Rect> boxes;
    std::vector<int> classIds;
    std::vector<float> confidences;

    EXPECT_FALSE(detector->detect(emptyFrame, boxes, classIds, confidences));
    EXPECT_TRUE(boxes.empty());
}