// config.h
#pragma once

#include <string>
#include <map>

class Config
{
private:
    std::map<std::string, std::string> settings;

public:
    Config();
    bool loadFromFile(const std::string &filename);
    std::string get(const std::string &key, const std::string &defaultValue = "") const;
    float getFloat(const std::string &key, float defaultValue = 0.0f) const;
    int getInt(const std::string &key, int defaultValue = 0) const;
};