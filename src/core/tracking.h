// //! for object tracking logic header file

// tracking.h

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>

// Structure to hold track information
struct Track
{
    int id;               // Unique ID for this track
    cv::Rect rect;        // Bounding box
    cv::Point2f centroid; // Center point
    int classId;          // Class ID from detector
    int framesLost;       // Counter for consecutive frames object was not detected

    // Kalman filter for this track
    cv::Ptr<cv::KalmanFilter> kalman;

    // Last prediction from Kalman filter
    cv::Mat prediction;

    // Velocity components
    float vx;
    float vy;
};

struct TrackHistory
{
    cv::Point2f lastPosition;
    int classId;
    int framesSinceDelete;
    cv::Rect lastRect;
};

class Tracker
{
public:
    // Constructor with parameters for tracking
    Tracker(int maxDistThreshold = 50, int maxDisappeared = 5, float pixelsToMeters = 0.1f, int maxHistoryFrames = 30,
            float reidThreshold = 0.7f, float frameTime = 1.0f / 30.0f);

        // Update tracking with new detections, returns vector of track IDs and bounding boxes
    std::vector<std::pair<int, cv::Rect>> update(const std::vector<cv::Rect> &boxes, const std::vector<int> &classIds);

    // Get class ID for a specific track
    int getClassId(int trackId) const;

    // Get velocity information for a specific track
    bool getVelocity(int trackId, float &vx, float &vy) const;

    void handleLostTracks(std::vector<std::pair<int, cv::Rect>> &trackedObjects);
    void createNewTrack(const cv::Rect &box, int classId, const cv::Point2f &centroid,
                        std::vector<std::pair<int, cv::Rect>> &trackedObjects);
    void createNewTracks(const std::vector<cv::Rect> &boxes,
                         const std::vector<int> &classIds,
                         const std::vector<cv::Point2f> &centroids,
                         std::vector<std::pair<int, cv::Rect>> &trackedObjects);
    float calculateIoU(const cv::Rect &box1, const cv::Rect &box2) const;



private:
    // Map of active tracks
    std::unordered_map<int, Track> tracks;

    // Next available track ID
    int nextId;

    // reidentification ID
    int reidId;
    
    // Maximum allowed distance for matching
    int maxDistance;

    // Maximum number of frames an object can be lost before track is deleted
    int maxDisappeared;

    // Conversion factor from pixels to meters
    const float PIXELS_TO_METERS;

    // Frame time in seconds
    const float FRAME_TIME;

    // Maximum number of frames to keep history for reidentification
    const int maxFramesToKeepHistory;

    // Threshold for reidentification
    const float reidentificationThreshold;

    // Calculate centroid of a bounding box
    cv::Point2f calculateCentroid(const cv::Rect &box) const;

    // Calculate distance between two points
    float calculateDistance(const cv::Point2f &p1, const cv::Point2f &p2) const;

    // Initialize Kalman filter for a new track
    cv::Ptr<cv::KalmanFilter> initializeKalmanFilter(const cv::Rect &box) const;

    // Update Kalman filter with new measurement
    void updateKalmanFilter(Track &track, const cv::Rect &box);

    // Predict new position using Kalman filter
    cv::Rect predictKalmanFilter(Track &track);

    std::map<int, TrackHistory> recentlyLostTracks;
};

#endif // TRACKING_H