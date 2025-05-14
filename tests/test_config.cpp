//! For testing configuration

#pragma once
#include "test_config.h"
#include <opencv2/opencv.hpp>

class TestDataHelper
{
public:
    static void copyToTestFolder(const std::string &sourcePath, const std::string &testType)
    {
        std::string destPath;
        if (sourcePath.find(".mp4") != std::string::npos)
        {
            destPath = TestPaths::TEST_VIDEOS + "/" + testType;
        }
        else
        {
            destPath = TestPaths::TEST_IMAGES + "/" + testType;
        }

        std::filesystem::create_directories(destPath);
        std::filesystem::copy_file(sourcePath, destPath + "/" + std::filesystem::path(sourcePath).filename().string(), std::filesystem::copy_options::overwrite_existing);
    }

    static std::vector<std::string> getTestFiles(const std::string &testType, const std::string &extension)
    {
        std::vector<std::string> files;
        std::string path = extension == ".mp4" ? TestPaths::TEST_VIDEOS + "/" + testType : TestPaths::TEST_IMAGES + "/" + testType;

        for (const auto &entry : std::filesystem::directory_iterator(path))
        {
            if (entry.path().extension() == extension)
            {
                files.push_back(entry.path().string());
            }
        }
        return files;
    }
};