//! for testing tracking

#include "test_config.h"
#include <gtest/gtest.h>
#include "../src/core/tracking.h"

float calculateIoU(const cv::Rect &box1, const cv::Rect &box2)
{
    int x1 = std::max(box1.x, box2.x);
    int y1 = std::max(box1.y, box2.y);
    int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    int y2 = std::min(box1.y + box1.height, box2.y + box2.height);

    if (x2 <= x1 || y2 <= y1)
        return 0.0f;

    float intersectionArea = (x2 - x1) * (y2 - y1);
    float box1Area = box1.width * box1.height;
    float box2Area = box2.width * box2.height;

    return intersectionArea / (box1Area + box2Area - intersectionArea);
}

class TrackerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ASSERT_TRUE(TestPaths::validatePaths()) << "Test data paths not found";

        tracker = new Tracker(50, 5, 0.1f, 30, 0.7f, 1.0f / 30.0f);

        // Load test video paths
        testVideoPath = TestPaths::TEST_VIDEOS + "/tracking/test2.mp4";
        ASSERT_TRUE(std::filesystem::exists(testVideoPath)) << "Test video not found: " << testVideoPath;
    }

    void TearDown() override
    {
        delete tracker;
    }

    Tracker *tracker;
    std::string testVideoPath;
};

TEST_F(TrackerTest, BasicTracking)
{
        cv::VideoCapture cap(testVideoPath);
    ASSERT_TRUE(cap.isOpened());

    cv::Mat frame;
    std::vector<cv::Rect> boxes;
    std::vector<int> classIds;

    // Process first 10 frames
    int frameCount = 0;
    while (cap.read(frame) && frameCount < 10)
    {
        // Simulate detection by creating a moving box
        cv::Rect movingBox(100 + frameCount * 10, 100 + frameCount * 5, 50, 50);
        boxes = {movingBox};
        classIds = {0};

        // Update tracker with new detection
        auto tracks = tracker->update(boxes, classIds);

        // Verify tracking
        ASSERT_FALSE(tracks.empty()) << "No tracks found in frame " << frameCount;
        EXPECT_EQ(tracks.size(), 1) << "Expected single track in frame " << frameCount;

        if (!tracks.empty())
        {
            // Check if track ID remains consistent
            EXPECT_EQ(tracks[0].first, 0) << "Track ID changed in frame " << frameCount;

            // Verify position tracking
            float iou = calculateIoU(tracks[0].second, movingBox);
            EXPECT_GT(iou, 0.5f) << "Track lost object in frame " << frameCount;

            // Check velocity calculation after first frame
            if (frameCount > 0)
            {
                float vx, vy;
                EXPECT_TRUE(tracker->getVelocity(0, vx, vy))
                    << "Failed to get velocity in frame " << frameCount;

                // Expect positive x velocity (moving right)
                EXPECT_GT(vx, 0) << "Incorrect x velocity in frame " << frameCount;

                // Expect positive y velocity (moving down)
                EXPECT_GT(vy, 0) << "Incorrect y velocity in frame " << frameCount;
            }
        }
        frameCount++;
    }

    cap.release();
}

TEST_F(TrackerTest, InitializationTest)
{ 

    std::vector<cv::Rect> boxes = {cv::Rect(100, 100, 50, 50)};
    std::vector<int> classIds = {0};

    auto tracks = tracker->update(boxes, classIds);
    ASSERT_EQ(tracks.size(), 1) << "Should create exactly one track";

    if (!tracks.empty())
    {
        // Track IDs start from 0
        EXPECT_EQ(tracks[0].first, 0) << "First track should have ID 0";

        // Verify track position
        EXPECT_EQ(tracks[0].second, boxes[0]) << "Track position should match input box";
    }
}

TEST_F(TrackerTest, TrackingConsistency)
{
    std::vector<cv::Rect> boxes1 = {cv::Rect(100, 100, 50, 50)};
    std::vector<cv::Rect> boxes2 = {cv::Rect(110, 110, 50, 50)};
    std::vector<int> classIds = {0};

    auto tracks1 = tracker->update(boxes1, classIds);
    auto tracks2 = tracker->update(boxes2, classIds);

    EXPECT_EQ(tracks1[0].first, tracks2[0].first); // Same track ID
}

TEST_F(TrackerTest, VelocityCalculation)
{
    std::vector<cv::Rect> boxes1 = {cv::Rect(100, 100, 50, 50)};
    std::vector<cv::Rect> boxes2 = {cv::Rect(110, 110, 50, 50)};
    std::vector<int> classIds = {0};

    tracker->update(boxes1, classIds);
    tracker->update(boxes2, classIds);

    float vx, vy;
    EXPECT_TRUE(tracker->getVelocity(0, vx, vy)) << "Should calculate velocity for track 0";

    if (tracker->getVelocity(0, vx, vy))
    {
        EXPECT_GT(vx, 0) << "X velocity should be positive";
        EXPECT_GT(vy, 0) << "Y velocity should be positive";
    }
}