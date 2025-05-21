//! For recording video 

#include "video.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>

VideoRecorder::VideoRecorder(const std::string& outDir, int maxFiles, const std::string& codecStr)
    : outputDir(outDir), maxFiles(maxFiles), codec(codecStr) {
    createOutputDir();
}

void VideoRecorder::createOutputDir() {
    try {
        std::filesystem::create_directories(outputDir);
    } catch (const std::exception& e) {
        std::cerr << "Failed to create output directory: " << e.what() << std::endl;
    }
}

std::string VideoRecorder::generateFileName() const {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << outputDir << "/recording_" 
       << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") 
       << ".mp4";
    return ss.str();
}

bool VideoRecorder::startRecording(const cv::Size& frameSize, double fps) {
    if (writer.isOpened()) {
        return false;
    }

    std::string filename = generateFileName();
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    
    writer.open(filename, fourcc, fps, frameSize);
    
    if (writer.isOpened()) {
        recordingFiles.push(filename);
        cleanOldRecordings();
        std::cout << "Started recording to: " << filename << std::endl;
        return true;
    }
    
    std::cerr << "Failed to start recording to: " << filename << std::endl;
    return false;
}

void VideoRecorder::stopRecording() {
    if (writer.isOpened()) {
        writer.release();
        std::cout << "Recording stopped" << std::endl;
    }
}

bool VideoRecorder::writeFrame(const cv::Mat& frame) {
    if (!writer.isOpened()) {
        return false;
    }
    
    try {
        writer.write(frame);
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "Error writing frame: " << e.what() << std::endl;
        return false;
    }
}

void VideoRecorder::cleanOldRecordings() {
    while (recordingFiles.size() > maxFiles) {
        std::string oldestFile = recordingFiles.front();
        recordingFiles.pop();
        try {
            if (std::filesystem::remove(oldestFile)) {
                std::cout << "Removed old recording: " << oldestFile << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to remove old recording: " << e.what() << std::endl;
        }
    }
}