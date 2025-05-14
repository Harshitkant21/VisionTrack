//! For testing configuration

#pragma once
#include <string>
#include <filesystem>

struct TestPaths
{
    static const std::string BASE_PATH;
    static const std::string TEST_VIDEOS;
    static const std::string TEST_IMAGES;

    static bool validatePaths()
    {
        return std::filesystem::exists(TEST_VIDEOS) &&
               std::filesystem::exists(TEST_IMAGES);
    }
};

const std::string TestPaths::BASE_PATH = "D:/Projects/VisionTrack/data";
const std::string TestPaths::TEST_VIDEOS = BASE_PATH + "/test_videos";
const std::string TestPaths::TEST_IMAGES = BASE_PATH + "/test_images";