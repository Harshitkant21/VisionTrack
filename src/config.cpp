// config.cpp
#include "config.h"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <system_error>
#include <algorithm>


Config::Config()
{
    //! Initialize with default values
    
    //file names
    settings["model_path"] = "D:/VisionTrack/models/yolov8n.onnx";
    settings["classes_path"] = "D:/VisionTrack/models/coco.names";

    // Detection parameters
    settings["confidence_threshold"] = "0.25";
    settings["nms_threshold"] = "0.45";
    settings["default_source"] = "0";
    settings["tracking_max_distance"]="50";
    settings["tracking_max_frames_lost"]="5";
    settings["tracking_overlap_threshold"]="0.7";
    settings["reidentification_threshold"]="0.7";
    settings["max_frames_keep_history"]="30";
    settings["pixels_to_meters"]="0.1";
    settings["video_fps"]="30.0"; 

    // Speed parameters
    settings["min_speed_kmh"]="1.0";
    settings["max_speed_kmh"]="150.0";
    settings["speed_normalization_factor"]="50.0";

    // Display parameters
    settings["show_trajectories"] = "true";
    settings["trajectory_length"] = "10";
    settings["show_velocity_vectors"] = "true";
    settings["velocity_vector_scale"]=2.0;
    settings["font_scale"]=0.5;
    settings["font_thickness"]=1;
    settings["footer_height"]=50;
    settings["alert_panel_width"]=std::to_string(400);

    // Alert parameters
    settings["speed_limit"]="50.0";
    settings["stopped_time_threshold"]="30.0";
    settings["restricted_area_threshold"]="0.5";
    settings["alert_display_time"]="60";
}

bool Config::loadFromFile(const std::string &filename)
{
    std::error_code ec;
    if (!std::filesystem::exists(filename, ec) || !std::filesystem::is_regular_file(filename, ec))
    {
        std::cerr << "Invalid config file: " << filename << std::endl;
        return false;
    }

    std::ifstream file(filename);
    if (!file.is_open())
    {

        std::cerr << "Could not open config file: " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        // Parse key=value
        size_t delimPos = line.find('=');
        if (delimPos != std::string::npos)
        {
            std::string key = line.substr(0, delimPos);
            std::string value = line.substr(delimPos + 1);

            // Trim whitespace
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);

            settings[key] = value;
        }
    }

    file.close();
    return true;
}

std::string Config::get(const std::string &key, const std::string &defaultValue) const
{
    auto it = settings.find(key);
    if (it != settings.end())
    {
        return it->second;
    }
    return defaultValue;
}

float Config::getFloat(const std::string &key, float defaultValue) const
{
    auto it = settings.find(key);
    if (it != settings.end())
    {
        try
        {
            return std::stof(it->second);
        }
        catch (...)
        {
            std::cerr << "Warning: Could not convert '" << key << "' value to float" << std::endl;
        }
    }
    return defaultValue;
}

int Config::getInt(const std::string &key, int defaultValue) const
{
    auto it = settings.find(key);
    if (it != settings.end())
    {
        try
        {
            return std::stoi(it->second);
        }
        catch (...)
        {
            std::cerr << "Warning: Could not convert '" << key << "' value to int" << std::endl;
        }
    }
    return defaultValue;
}

bool Config::getBool(const std::string &key, bool defaultValue) const
{
    auto it = settings.find(key);
    if (it != settings.end())
    {
        std::string value = it->second;
        std::transform(value.begin(), value.end(), value.begin(), ::tolower); // make lowercase
        if (value == "true" || value == "1")
            return true;
        if (value == "false" || value == "0")
            return false;

        std::cerr << "Warning: Invalid boolean value for '" << key << "'; using default" << std::endl;
    }
    return defaultValue;
}