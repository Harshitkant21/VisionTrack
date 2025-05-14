//! for alert generation

#include "alerts.h"
#include <filesystem>
#include <sstream>

AlertManager::AlertManager(float speedLimit, float stoppedTimeThreshold, float restrictedAreaThreshold)
    : speedLimit(speedLimit),
      stoppedTimeThreshold(stoppedTimeThreshold),
      restrictedAreaThreshold(restrictedAreaThreshold) {}

void AlertManager::checkSpeedViolation(int trackId, const std::string &className, float speed, const cv::Rect &box)
{
    if (speed > speedLimit)
    {
        Alert alert{
            AlertType::SPEED_VIOLATION,
            trackId,
            className,
            speed,
            box,
            std::chrono::system_clock::now(),
            "Speed violation: " + std::to_string(static_cast<int>(speed)) + " km/h",
            generateUniqueId(trackId, AlertType::SPEED_VIOLATION),
            true};
        addAlert(std::move(alert));
    }
}

void AlertManager::checkRestrictedArea(int trackId, const std::string &className, const cv::Rect &box, const cv::Rect &restrictedZone)
{
    // Calculate intersection with restricted area
    cv::Rect intersection = box & restrictedZone;
    float overlapRatio = intersection.area() / static_cast<float>(box.area());

    if (overlapRatio > restrictedAreaThreshold)
    {
        Alert alert{
            AlertType::RESTRICTED_AREA,
            trackId,
            className,
            overlapRatio,
            box,
            std::chrono::system_clock::now(),
            "Restricted area violation",
            generateUniqueId(trackId, AlertType::RESTRICTED_AREA),
            true};
        addAlert(std::move(alert));
    }
}

void AlertManager::checkStoppedVehicle(int trackId, const std::string &className, const cv::Point2f &position, const cv::Rect &box)
{
    auto now = std::chrono::system_clock::now();

    if (stoppedVehicles.find(trackId) == stoppedVehicles.end())
    {
        stoppedVehicles[trackId] = now;
    }
    else
    {
        auto stoppedDuration = std::chrono::duration_cast<std::chrono::seconds>(now - stoppedVehicles[trackId]).count();

        if (stoppedDuration > stoppedTimeThreshold)
        {
            Alert alert{
                AlertType::STOPPED_VEHICLE,
                trackId,
                className,
                static_cast<float>(stoppedDuration),
                box,
                now};
            addAlert(std::move(alert));
            stoppedVehicles.erase(trackId);
        }
    }
}

void AlertManager::drawAlerts(cv::Mat &frame)
{
    int yOffset = 30;

    for (const auto &alert : activeAlerts)
    {
        // Prepare alert text
        std::string displayText = "ID " + std::to_string(alert.trackId) + ": " + alert.message;

        // Draw text background
        int baseLine;
        cv::Size textSize = cv::getTextSize(displayText, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseLine);

        cv::rectangle(frame,
                      cv::Point(10, yOffset - textSize.height),
                      cv::Point(10 + textSize.width, yOffset + baseLine),
                      cv::Scalar(0, 0, 0),
                      cv::FILLED);

        // Draw alert text
        cv::putText(frame, displayText,
                    cv::Point(10, yOffset),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.6, cv::Scalar(255, 255, 255), 2);

        yOffset += textSize.height + 10;
    }
}

// Add method to export alerts as JSON for UI
std::string AlertManager::getAlertsAsJson() const
{
    std::stringstream ss;
    ss << "{\"alerts\":[";
    bool first = true;

    for (const auto &alert : activeAlerts)
    {
        if (!first)
            ss << ",";
        ss << "{"
           << "\"id\":\"" << alert.uniqueId << "\","
           << "\"trackId\":" << alert.trackId << ","
           << "\"type\":\"" << static_cast<int>(alert.type) << "\","
           << "\"message\":\"" << alert.message << "\","
           << "\"value\":" << alert.value << ","
           << "\"timestamp\":\"" << std::chrono::system_clock::to_time_t(alert.timestamp) << "\""
           << "}";
        first = false;
    }
    ss << "]}";
    return ss.str();
}

void AlertManager::clearOldAlerts(int maxAgeSeconds)
{
    auto now = std::chrono::system_clock::now();
    activeAlerts.erase(
        std::remove_if(activeAlerts.begin(), activeAlerts.end(),
                       [now, maxAgeSeconds](const Alert &alert)
                       {
                           auto age = std::chrono::duration_cast<std::chrono::seconds>(
                                          now - alert.timestamp)
                                          .count();
                           return age > maxAgeSeconds;
                       }),
        activeAlerts.end());
}

void AlertManager::addAlert(Alert &&alert)
{
    // Check if similar alert already exists
    for (const auto &existing : activeAlerts)
    {
        if (existing.trackId == alert.trackId &&
            existing.type == alert.type)
        {
            return; // Skip duplicate alert
        }
    }
    activeAlerts.push_back(std::move(alert));
}

// Generate a unique ID for the alert
std::string AlertManager::generateUniqueId(int trackId, AlertType type) const
{
    std::stringstream ss;
    ss << "alert_" << trackId << "_" << static_cast<int>(type) << "_" << std::chrono::system_clock::now().time_since_epoch().count();
    return ss.str();
}

// Generate a human-readable message for the alert
std::string AlertManager::generateAlertMessage(const Alert &alert) const
{
    std::stringstream ss;
    ss << "Alert: " << alert.message;
    return ss.str();
}

