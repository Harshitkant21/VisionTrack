// //! for object tracking logic

#include "tracking.h"
#include <iostream>
#include <limits>
#include <algorithm>
#include <cmath>
#include <set>

Tracker::Tracker(int maxDistThreshold, int maxDisappeared) 
    : maxDistance(maxDistThreshold), 
      maxDisappeared(maxDisappeared), 
      nextId(0) {
    std::cout << "Tracker initialized with max distance: " << maxDistance 
              << ", max disappeared: " << maxDisappeared << std::endl;
}

std::vector<std::pair<int, cv::Rect>> Tracker::update(const std::vector<cv::Rect>& boxes, const std::vector<int>& classIds) {
    // Result vector with <ID, Rect> pairs
    std::vector<std::pair<int, cv::Rect>> trackedObjects;
    
    // First, predict new positions for all existing tracks using Kalman filter
    for (auto& [id, track] : tracks) {
        // Predict new position
        cv::Rect predictedBox = predictKalmanFilter(track);
        
        // Update track with prediction (will be corrected later if detection found)
        track.rect = predictedBox;
        track.centroid = calculateCentroid(predictedBox);
    }
    
    // If no detections, increment disappeared counter for all existing tracks
    if (boxes.empty()) {
        std::vector<int> idsToRemove;
        
        for (auto& [id, track] : tracks) {
            track.framesLost++;
            
            // Check if we should remove this track
            if (track.framesLost > maxDisappeared) {
                idsToRemove.push_back(id);
            } else {
                // Keep the track with its predicted position
                trackedObjects.push_back(std::make_pair(id, track.rect));
            }
        }
        
        // Remove tracks that have been missing for too long
        for (int id : idsToRemove) {
            std::cout << "Removed track ID " << id << " due to disappearance" << std::endl;
            tracks.erase(id);
        }
        
        return trackedObjects;
    }
    
    // Calculate centroids for current detections
    std::vector<cv::Point2f> inputCentroids;
    for (const auto& box : boxes) {
        inputCentroids.push_back(calculateCentroid(box));
    }
    
    // If we currently aren't tracking any objects, register all of them
    if (tracks.empty()) {
        for (size_t i = 0; i < boxes.size(); i++) {
            Track newTrack;
            newTrack.id = nextId;
            newTrack.rect = boxes[i];
            newTrack.centroid = inputCentroids[i];
            newTrack.classId = classIds[i];
            newTrack.framesLost = 0;
            newTrack.vx = 0.0f;
            newTrack.vy = 0.0f;
            
            // Initialize Kalman filter for this track
            newTrack.kalman = initializeKalmanFilter(boxes[i]);
            
            tracks[nextId] = newTrack;
            trackedObjects.push_back(std::make_pair(nextId, boxes[i]));
            
            std::cout << "Registered new track ID " << nextId << " of class " << classIds[i] << std::endl;
            nextId++;
        }
    }
    // Otherwise, match new detections to existing objects
    else {
        // Get IDs of existing tracks
        std::vector<int> trackIds;
        std::vector<cv::Point2f> existingCentroids;
        
        for (const auto& [id, track] : tracks) {
            trackIds.push_back(id);
            existingCentroids.push_back(track.centroid);
        }
        
        // Compute distance matrix between existing predicted positions and new detections
        std::vector<std::vector<float>> D(trackIds.size(), std::vector<float>(inputCentroids.size()));
        
        for (size_t i = 0; i < trackIds.size(); i++) {
            for (size_t j = 0; j < inputCentroids.size(); j++) {
                D[i][j] = calculateDistance(existingCentroids[i], inputCentroids[j]);
            }
        }
        
        // Find minimum distance for each row and column
        std::vector<int> rowIndices;
        std::vector<int> colIndices;
        
        // Find the smallest distances using a greedy algorithm
        // (Could be improved with Hungarian algorithm for optimal assignment)
        while (true) {
            float minDist = std::numeric_limits<float>::max();
            int minRow = -1;
            int minCol = -1;
            
            // Find the minimum distance in the remaining rows and columns
            for (size_t i = 0; i < D.size(); i++) {
                if (std::find(rowIndices.begin(), rowIndices.end(), i) != rowIndices.end()) {
                    continue;
                }
                
                for (size_t j = 0; j < D[i].size(); j++) {
                    if (std::find(colIndices.begin(), colIndices.end(), j) != colIndices.end()) {
                        continue;
                    }
                    
                    if (D[i][j] < minDist) {
                        minDist = D[i][j];
                        minRow = i;
                        minCol = j;
                    }
                }
            }
            
            // If we couldn't find any more minimum values, break
            if (minRow == -1 || minCol == -1) {
                break;
            }
            
            // Add to our matches if the distance is within threshold
            if (minDist <= maxDistance) {
                rowIndices.push_back(minRow);
                colIndices.push_back(minCol);
            } else {
                break;
            }
        }
        
        // Used to keep track of rows and columns we've already examined
        std::set<int> usedRows(rowIndices.begin(), rowIndices.end());
        std::set<int> usedCols(colIndices.begin(), colIndices.end());
        
        // Update matched existing tracks
        for (size_t i = 0; i < rowIndices.size(); i++) {
            int trackId = trackIds[rowIndices[i]];
            int detectionIdx = colIndices[i];
            
            // Update the Kalman filter with the new detection
            updateKalmanFilter(tracks[trackId], boxes[detectionIdx]);
            
            // Update track with new position and reset lost counter
            tracks[trackId].rect = boxes[detectionIdx];
            tracks[trackId].centroid = inputCentroids[detectionIdx];
            
            // Class ID voting (maintain consistency by not changing class on every frame)
            // Only update class if detection confidence is high or track is new
            if (tracks[trackId].framesLost > 2) {
                tracks[trackId].classId = classIds[detectionIdx];
            }
            
            tracks[trackId].framesLost = 0;
            
            std::cout << "Updated track ID " << trackId << " to new position" << std::endl;
        }
        
        // Register unmatched detections as new tracks
        for (size_t j = 0; j < inputCentroids.size(); j++) {
            if (usedCols.find(j) != usedCols.end()) {
                continue;
            }
            
            // Create new track
            Track newTrack;
            newTrack.id = nextId;
            newTrack.rect = boxes[j];
            newTrack.centroid = inputCentroids[j];
            newTrack.classId = classIds[j];
            newTrack.framesLost = 0;
            newTrack.vx = 0.0f;
            newTrack.vy = 0.0f;
            
            // Initialize Kalman filter for this track
            newTrack.kalman = initializeKalmanFilter(boxes[j]);
            
            tracks[nextId] = newTrack;
            
            std::cout << "Registered new track ID " << nextId << " of class " << classIds[j] << std::endl;
            nextId++;
        }
        
        // Update unmatched tracks (mark as lost)
        std::vector<int> idsToRemove;
        
        for (size_t i = 0; i < trackIds.size(); i++) {
            if (usedRows.find(i) != usedRows.end()) {
                continue;
            }
            
            int trackId = trackIds[i];
            tracks[trackId].framesLost++;
            
            // Check if we should remove this track
            if (tracks[trackId].framesLost > maxDisappeared) {
                idsToRemove.push_back(trackId);
            }
        }
        
        // Remove tracks that have been missing for too long
        for (int id : idsToRemove) {
            std::cout << "Removed track ID " << id << " due to disappearance" << std::endl;
            tracks.erase(id);
        }
        
        // Create the final result with all current tracks
        for (const auto& [id, track] : tracks) {
            trackedObjects.push_back(std::make_pair(id, track.rect));
        }
    }
    
    return trackedObjects;
}

int Tracker::getClassId(int trackId) const {
    auto it = tracks.find(trackId);
    if (it != tracks.end()) {
        return it->second.classId;
    }
    return -1;  // Invalid class ID
}

bool Tracker::getVelocity(int trackId, float& vx, float& vy) const {
    auto it = tracks.find(trackId);
    if (it != tracks.end()) {
        vx = it->second.vx;
        vy = it->second.vy;
        return true;
    }
    return false;
}

cv::Point2f Tracker::calculateCentroid(const cv::Rect& box) const {
    float centerX = box.x + box.width / 2.0f;
    float centerY = box.y + box.height / 2.0f;
    return cv::Point2f(centerX, centerY);
}

float Tracker::calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2) const {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

cv::Ptr<cv::KalmanFilter> Tracker::initializeKalmanFilter(const cv::Rect& box) const {
    // Create a Kalman filter with:
    // - 6 dynamic parameters (x, y, width, height, vx, vy)
    // - 4 measurement parameters (x, y, width, height)
    // - 0 control parameters
    cv::Ptr<cv::KalmanFilter> kalman = cv::makePtr<cv::KalmanFilter>(6, 4, 0);
    
    // Transition matrix (describes how state evolves)
    // [1, 0, 0, 0, 1, 0] - x = x + vx
    // [0, 1, 0, 0, 0, 1] - y = y + vy
    // [0, 0, 1, 0, 0, 0] - width = width
    // [0, 0, 0, 1, 0, 0] - height = height
    // [0, 0, 0, 0, 1, 0] - vx = vx
    // [0, 0, 0, 0, 0, 1] - vy = vy
    cv::setIdentity(kalman->transitionMatrix);
    kalman->transitionMatrix.at<float>(0, 4) = 1.0f; // x += vx
    kalman->transitionMatrix.at<float>(1, 5) = 1.0f; // y += vy
    
    // Measurement matrix (describes how to map state to measurement)
    cv::Mat measurement = cv::Mat::zeros(4, 1, CV_32F);
    kalman->measurementMatrix = cv::Mat::zeros(4, 6, CV_32F);
    kalman->measurementMatrix.at<float>(0, 0) = 1.0f; // x
    kalman->measurementMatrix.at<float>(1, 1) = 1.0f; // y
    kalman->measurementMatrix.at<float>(2, 2) = 1.0f; // width
    kalman->measurementMatrix.at<float>(3, 3) = 1.0f; // height
    
    // Process noise covariance matrix (describes process noise)
    cv::setIdentity(kalman->processNoiseCov, cv::Scalar::all(0.05));
    kalman->processNoiseCov.at<float>(4, 4) = 0.1f; // More uncertainty in velocity
    kalman->processNoiseCov.at<float>(5, 5) = 0.1f;
    
    // Measurement noise covariance matrix (describes measurement noise)
    cv::setIdentity(kalman->measurementNoiseCov, cv::Scalar::all(0.1));
    
    // Error covariance matrix (describes initial uncertainty)
    cv::setIdentity(kalman->errorCovPost, cv::Scalar::all(1.0));
    
    // Initial state
    kalman->statePost.at<float>(0) = box.x + box.width / 2.0f;  // x center
    kalman->statePost.at<float>(1) = box.y + box.height / 2.0f; // y center
    kalman->statePost.at<float>(2) = box.width;                 // width
    kalman->statePost.at<float>(3) = box.height;                // height
    kalman->statePost.at<float>(4) = 0;                         // vx (initial velocity is 0)
    kalman->statePost.at<float>(5) = 0;                         // vy
    
    return kalman;
}

void Tracker::updateKalmanFilter(Track& track, const cv::Rect& box) {
    // Create measurement
    cv::Mat measurement = cv::Mat::zeros(4, 1, CV_32F);
    measurement.at<float>(0) = box.x + box.width / 2.0f;  // x center
    measurement.at<float>(1) = box.y + box.height / 2.0f; // y center
    measurement.at<float>(2) = box.width;                 // width
    measurement.at<float>(3) = box.height;                // height
    
    // First predict, to align time with measurement
    track.prediction = track.kalman->predict();
    
    // Calculate current velocity from prediction and previous position
    track.vx = track.prediction.at<float>(4);
    track.vy = track.prediction.at<float>(5);
    
    // Then correct
    cv::Mat estimated = track.kalman->correct(measurement);
    
    // Could use estimated state directly, but we'll use the measurement
    // for the position and size, as it's more accurate for visible objects
    // track.rect = cv::Rect(
    //     estimated.at<float>(0) - estimated.at<float>(2) / 2.0f,
    //     estimated.at<float>(1) - estimated.at<float>(3) / 2.0f,
    //     estimated.at<float>(2),
    //     estimated.at<float>(3)
    // );
}

cv::Rect Tracker::predictKalmanFilter(Track& track) {
    // Generate prediction
    track.prediction = track.kalman->predict();
    
    // Extract the prediction into a rectangle
    float x = track.prediction.at<float>(0) - track.prediction.at<float>(2) / 2.0f;
    float y = track.prediction.at<float>(1) - track.prediction.at<float>(3) / 2.0f;
    float width = track.prediction.at<float>(2);
    float height = track.prediction.at<float>(3);
    
    // Make sure the predicted box is valid
    x = std::max(0.0f, x);
    y = std::max(0.0f, y);
    width = std::max(1.0f, width);
    height = std::max(1.0f, height);
    
    return cv::Rect(static_cast<int>(x), static_cast<int>(y), 
                   static_cast<int>(width), static_cast<int>(height));
}