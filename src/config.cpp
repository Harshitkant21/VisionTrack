// config.cpp
#include "config.h"
#include <fstream>
#include <iostream>

Config::Config()
{
    // Initialize with default values
    settings["model_path"] = "models/yolo11n.onnx";
    settings["classes_path"] = "models/coco.names";
    settings["confidence_threshold"] = "0.25";
    settings["nms_threshold"] = "0.45";
    settings["default_source"] = "0";
    
    settings["show_trajectories"] = "true";
    settings["trajectory_length"] = "10";
    settings["show_velocity_vectors"] = "true";
    settings["use_gpu"] = "true";
}

bool Config::loadFromFile(const std::string &filename)
{
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