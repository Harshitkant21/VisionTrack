// //! for object tracking logic header file

// tracking.h

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>

// Structure to hold track information
struct Track {
    int id;                 // Unique ID for this track
    cv::Rect rect;          // Bounding box
    cv::Point2f centroid;   // Center point
    int classId;            // Class ID from detector
    int framesLost;         // Counter for consecutive frames object was not detected
    
    // Kalman filter for this track
    cv::Ptr<cv::KalmanFilter> kalman;
    
    // Last prediction from Kalman filter
    cv::Mat prediction;
    
    // Velocity components
    float vx;
    float vy;
};

class Tracker {
public:
    // Constructor with parameters for tracking
    Tracker(int maxDistThreshold = 50, int maxDisappeared = 5);
    
    // Update tracking with new detections, returns vector of track IDs and bounding boxes
    std::vector<std::pair<int, cv::Rect>> update(const std::vector<cv::Rect>& boxes, const std::vector<int>& classIds);
    
    // Get class ID for a specific track
    int getClassId(int trackId) const;
    
    // Get velocity information for a specific track
    bool getVelocity(int trackId, float& vx, float& vy) const;

private:
    // Map of active tracks
    std::unordered_map<int, Track> tracks;
    
    // Next available track ID
    int nextId;
    
    // Maximum allowed distance for matching
    int maxDistance;
    
    // Maximum number of frames an object can be lost before track is deleted
    int maxDisappeared;
    
    // Calculate centroid of a bounding box
    cv::Point2f calculateCentroid(const cv::Rect& box) const;
    
    // Calculate distance between two points
    float calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2) const;
    
    // Initialize Kalman filter for a new track
    cv::Ptr<cv::KalmanFilter> initializeKalmanFilter(const cv::Rect& box) const;
    
    // Update Kalman filter with new measurement
    void updateKalmanFilter(Track& track, const cv::Rect& box);
    
    // Predict new position using Kalman filter
    cv::Rect predictKalmanFilter(Track& track);
};

#endif // TRACKING_H