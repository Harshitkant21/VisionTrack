//! For video recording header
#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <queue>
#include <filesystem>

class VideoRecorder
{
public:
    VideoRecorder(const std::string &outputDir, int maxFiles, const std::string &codec);

    bool startRecording(const cv::Size &frameSize, double fps);
    void stopRecording();
    bool writeFrame(const cv::Mat &frame);
    bool isRecording() const { return writer.isOpened(); }

private:
    std::string outputDir;
    int maxFiles;
    std::string codec;
    cv::VideoWriter writer;
    std::queue<std::string> recordingFiles;

    void cleanOldRecordings();
    std::string generateFileName() const;
    void createOutputDir();
};