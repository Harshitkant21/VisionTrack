//! for alert generation header file

#ifndef ALERTS_H
#define ALERTS_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <chrono>

enum class AlertType
{
    SPEED_VIOLATION,
    RESTRICTED_AREA,
    STOPPED_VEHICLE,
    DIRECTION_VIOLATION
};

struct Alert
{
    AlertType type;
    int trackId;
    std::string className;
    float value;
    cv::Rect location;
    std::chrono::system_clock::time_point timestamp;
    std::string message;  // Human readable message
    std::string uniqueId; // Unique identifier for the alert
    bool isActive;        // Current state of alert
};

class AlertManager
{
public:
    AlertManager(float speedLimit, float stoppedTimeThreshold, float restrictedAreaThreshold);

    // Updated alert checks
    
    void checkSpeedViolation(int trackId, const std::string &className, float speed, const cv::Rect &box);

    void checkRestrictedArea(int trackId, const std::string &className, const cv::Rect &box, const cv::Rect &restrictedZone);

    void checkStoppedVehicle(int trackId, const std::string &className, const cv::Point2f &position, const cv::Rect &box);

    // Methods for UI integration
    std::vector<Alert> getActiveAlerts() const { return activeAlerts; }
    void clearOldAlerts(int maxAgeSeconds = 60);
    void drawAlerts(cv::Mat &frame, int xStart, int yStart);

    // Export alerts for UI
    std::string getAlertsAsJson() const;

private:
    float speedLimit;
    float restrictedAreaThreshold;
    float stoppedTimeThreshold;
    std::vector<Alert> activeAlerts;
    std::unordered_map<int, std::chrono::system_clock::time_point> stoppedVehicles;
    cv::Rect restrictedZone; // Define your restricted area

    void addAlert(Alert &&alert);
    std::string generateAlertMessage(const Alert &alert) const;
    std::string generateUniqueId(int trackId, AlertType type) const;
};

#endif