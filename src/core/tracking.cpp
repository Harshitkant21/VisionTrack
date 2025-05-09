// //! for object tracking logic

#include "tracking.h"
#include "hungarian.h"
#include <iostream>
#include <limits>
#include <algorithm>
#include <cmath>
#include <set>

Tracker::Tracker(int maxDistThreshold, int maxDisappeared,
                 float pixelsToMeters, int maxHistoryFrames,
                 float reidThreshold, float frameTime)
    : maxDistance(maxDistThreshold),
      maxDisappeared(maxDisappeared),
      PIXELS_TO_METERS(pixelsToMeters),
      maxFramesToKeepHistory(maxHistoryFrames),
      reidentificationThreshold(reidThreshold),
      FRAME_TIME(frameTime),
      nextId(0)
{
    std::cout << "Tracker initialized with max distance: " << maxDistance
              << ", max disappeared: " << maxDisappeared << std::endl;
}

// std::vector<std::pair<int, cv::Rect>> Tracker::update(const std::vector<cv::Rect> &boxes, const std::vector<int> &classIds)
// {
//     // Result vector with <ID, Rect> pairs
//     std::vector<std::pair<int, cv::Rect>> trackedObjects;

//     // First, predict new positions for all existing tracks using Kalman filter
//     for (auto &[id, track] : tracks)
//     {
//         // Predict new position
//         cv::Rect predictedBox = predictKalmanFilter(track);

//         // Update track with prediction (will be corrected later if detection found)
//         track.rect = predictedBox;
//         track.centroid = calculateCentroid(predictedBox);
//     }

//     // If no detections, increment disappeared counter for all existing tracks
//     if (boxes.empty())
//     {
//         std::vector<int> idsToRemove;

//         for (auto &[id, track] : tracks)
//         {
//             track.framesLost++;

//             // Check if we should remove this track
//             if (track.framesLost > maxDisappeared)
//             {
//                 idsToRemove.push_back(id);
//             }
//             else
//             {
//                 // Keep the track with its predicted position
//                 trackedObjects.push_back(std::make_pair(id, track.rect));
//             }
//         }

//         // Remove tracks that have been missing for too long
//         for (int id : idsToRemove)
//         {
//             std::cout << "Removed track ID " << id << " due to disappearance" << std::endl;
//             tracks.erase(id);
//         }

//         return trackedObjects;
//     }

//     // Calculate centroids for current detections
//     std::vector<cv::Point2f> inputCentroids;
//     for (const auto &box : boxes)
//     {
//         inputCentroids.push_back(calculateCentroid(box));
//     }

//     // If we currently aren't tracking any objects, register all of them
//     if (tracks.empty())
//     {
//         for (size_t i = 0; i < boxes.size(); i++)
//         {
//             Track newTrack;
//             newTrack.id = nextId;
//             newTrack.rect = boxes[i];
//             newTrack.centroid = inputCentroids[i];
//             newTrack.classId = classIds[i];
//             newTrack.framesLost = 0;
//             newTrack.vx = 0.0f;
//             newTrack.vy = 0.0f;

//             // Initialize Kalman filter for this track
//             newTrack.kalman = initializeKalmanFilter(boxes[i]);

//             tracks[nextId] = newTrack;
//             trackedObjects.push_back(std::make_pair(nextId, boxes[i]));

//             std::cout << "Registered new track ID " << nextId << " of class " << classIds[i] << std::endl;
//             nextId++;
//         }
//     }
//     // Otherwise, match new detections to existing objects
//     else
//     {
//         // Get IDs of existing tracks
//         std::vector<int> trackIds;
//         std::vector<cv::Point2f> existingCentroids;

//         for (const auto &[id, track] : tracks)
//         {
//             trackIds.push_back(id);
//             existingCentroids.push_back(track.centroid);
//         }

//         // Compute distance matrix between existing predicted positions and new detections
//         std::vector<std::vector<float>> D(trackIds.size(), std::vector<float>(inputCentroids.size()));

//         for (size_t i = 0; i < trackIds.size(); i++)
//         {
//             for (size_t j = 0; j < inputCentroids.size(); j++)
//             {
//                 D[i][j] = calculateDistance(existingCentroids[i], inputCentroids[j]);
//             }
//         }

//         // Find minimum distance for each row and column
//         std::vector<int> rowIndices;
//         std::vector<int> colIndices;

//         // Find the smallest distances using a greedy algorithm
//         // (Could be improved with Hungarian algorithm for optimal assignment)
//         while (true)
//         {
//             float minDist = std::numeric_limits<float>::max();
//             int minRow = -1;
//             int minCol = -1;

//             // Find the minimum distance in the remaining rows and columns
//             for (size_t i = 0; i < D.size(); i++)
//             {
//                 if (std::find(rowIndices.begin(), rowIndices.end(), i) != rowIndices.end())
//                 {
//                     continue;
//                 }

//                 for (size_t j = 0; j < D[i].size(); j++)
//                 {
//                     if (std::find(colIndices.begin(), colIndices.end(), j) != colIndices.end())
//                     {
//                         continue;
//                     }

//                     if (D[i][j] < minDist)
//                     {
//                         minDist = D[i][j];
//                         minRow = i;
//                         minCol = j;
//                     }
//                 }
//             }

//             // If we couldn't find any more minimum values, break
//             if (minRow == -1 || minCol == -1)
//             {
//                 break;
//             }

//             // Add to our matches if the distance is within threshold
//             if (minDist <= maxDistance)
//             {
//                 rowIndices.push_back(minRow);
//                 colIndices.push_back(minCol);
//             }
//             else
//             {
//                 break;
//             }
//         }

//         // Used to keep track of rows and columns we've already examined
//         std::set<int> usedRows(rowIndices.begin(), rowIndices.end());
//         std::set<int> usedCols(colIndices.begin(), colIndices.end());

//         // Update matched existing tracks
//         for (size_t i = 0; i < rowIndices.size(); i++)
//         {
//             int trackId = trackIds[rowIndices[i]];
//             int detectionIdx = colIndices[i];

//             // Update the Kalman filter with the new detection
//             updateKalmanFilter(tracks[trackId], boxes[detectionIdx]);

//             // Update track with new position and reset lost counter
//             tracks[trackId].rect = boxes[detectionIdx];
//             tracks[trackId].centroid = inputCentroids[detectionIdx];

//             // Class ID voting (maintain consistency by not changing class on every frame)
//             // Only update class if detection confidence is high or track is new
//             if (tracks[trackId].framesLost > 2)
//             {
//                 tracks[trackId].classId = classIds[detectionIdx];
//             }

//             tracks[trackId].framesLost = 0;

//             std::cout << "Updated track ID " << trackId << " to new position" << std::endl;
//         }

//         // Register unmatched detections as new tracks
//         for (size_t j = 0; j < inputCentroids.size(); j++)
//         {
//             if (usedCols.find(j) != usedCols.end())
//             {
//                 continue;
//             }

//             // Create new track
//             Track newTrack;
//             newTrack.id = nextId;
//             newTrack.rect = boxes[j];
//             newTrack.centroid = inputCentroids[j];
//             newTrack.classId = classIds[j];
//             newTrack.framesLost = 0;
//             newTrack.vx = 0.0f;
//             newTrack.vy = 0.0f;

//             // Initialize Kalman filter for this track
//             newTrack.kalman = initializeKalmanFilter(boxes[j]);

//             tracks[nextId] = newTrack;

//             std::cout << "Registered new track ID " << nextId << " of class " << classIds[j] << std::endl;
//             nextId++;
//         }

//         // Update unmatched tracks (mark as lost)
//         std::vector<int> idsToRemove;

//         for (size_t i = 0; i < trackIds.size(); i++)
//         {
//             if (usedRows.find(i) != usedRows.end())
//             {
//                 continue;
//             }

//             int trackId = trackIds[i];
//             tracks[trackId].framesLost++;

//             // Check if we should remove this track
//             if (tracks[trackId].framesLost > maxDisappeared)
//             {
//                 idsToRemove.push_back(trackId);
//             }
//         }

//         // Remove tracks that have been missing for too long
//         for (int id : idsToRemove)
//         {
//             std::cout << "Removed track ID " << id << " due to disappearance" << std::endl;
//             tracks.erase(id);
//         }

//         // Create the final result with all current tracks
//         for (const auto &[id, track] : tracks)
//         {
//             trackedObjects.push_back(std::make_pair(id, track.rect));
//         }
//     }

//     return trackedObjects;
// }

std::vector<std::pair<int, cv::Rect>> Tracker::update(const std::vector<cv::Rect> &boxes, const std::vector<int> &classIds)
{
    std::vector<std::pair<int, cv::Rect>> trackedObjects;

    // First update all existing tracks with Kalman prediction
    for (auto &[id, track] : tracks)
    {
        cv::Rect predictedBox = predictKalmanFilter(track);
        track.rect = predictedBox;
    }

    if (boxes.empty())
    {
        handleLostTracks(trackedObjects);
        return trackedObjects;
    }

    // Build cost matrix using IoU
    vector<vector<double>> costMatrix;
    vector<int> trackIds;

    for (const auto &[id, track] : tracks)
    {
        trackIds.push_back(id);
        vector<double> costs;
        for (const auto &box : boxes)
        {
            float iou = calculateIoU(track.rect, box);
            costs.push_back(1.0 - iou); // Convert IoU to cost
        }
        costMatrix.push_back(costs);
    }

    // Only run Hungarian algorithm if we have both tracks and detections
    if (!costMatrix.empty() && !costMatrix[0].empty())
    {
        HungarianAlgorithm hungarian;
        vector<int> assignments;
        hungarian.Solve(costMatrix, assignments);

        std::set<int> usedDetections;

        // Process assignments
        for (size_t i = 0; i < assignments.size(); i++)
        {
            int detectionIdx = assignments[i];
            if (detectionIdx != -1 && costMatrix[i][detectionIdx] < 0.7)
            {
                int trackId = trackIds[i];
                Track &track = tracks[trackId];

                updateKalmanFilter(track, boxes[detectionIdx]);
                track.rect = boxes[detectionIdx];
                track.classId = classIds[detectionIdx];
                track.framesLost = 0;

                usedDetections.insert(detectionIdx);
                trackedObjects.emplace_back(trackId, boxes[detectionIdx]);
            }
        }

        // Create new tracks for unmatched detections
        for (size_t i = 0; i < boxes.size(); i++)
        {
            if (usedDetections.find(i) == usedDetections.end())
            {
                cv::Point2f centroid = calculateCentroid(boxes[i]);
                createNewTrack(boxes[i], classIds[i], centroid, trackedObjects);
            }
        }
    }
    else if (tracks.empty())
    {
        // If no existing tracks, create new ones for all detections
        for (size_t i = 0; i < boxes.size(); i++)
        {
            cv::Point2f centroid = calculateCentroid(boxes[i]);
            createNewTrack(boxes[i], classIds[i], centroid, trackedObjects);
        }
    }

    // Update lost tracks
    handleLostTracks(trackedObjects);

    return trackedObjects;
}

// Modify handleLostTracks method
void Tracker::handleLostTracks(std::vector<std::pair<int, cv::Rect>> &trackedObjects)
{
    std::vector<int> idsToRemove;

    // Update lost tracks
    for (auto &[id, track] : tracks)
    {
        track.framesLost++;
        if (track.framesLost > maxDisappeared)
        {
            idsToRemove.push_back(id);

            // Add to recently lost tracks history
            TrackHistory history;
            history.lastPosition = calculateCentroid(track.rect);
            history.classId = track.classId;
            history.framesSinceDelete = 0;
            history.lastRect = track.rect;
            recentlyLostTracks[id] = history;
        }
        else
        {
            trackedObjects.emplace_back(id, track.rect);
        }
    }

    // Remove lost tracks
    for (int id : idsToRemove)
    {
        tracks.erase(id);
    }

    // Update and clean up track history
    std::vector<int> historyToRemove;
    for (auto &[id, history] : recentlyLostTracks)
    {
        history.framesSinceDelete++;
        if (history.framesSinceDelete > maxFramesToKeepHistory)
        {
            historyToRemove.push_back(id);
        }
    }

    for (int id : historyToRemove)
    {
        recentlyLostTracks.erase(id);
    }
}

// Modify createNewTrack method to check for reidentification
void Tracker::createNewTrack(const cv::Rect &box, int classId, const cv::Point2f &centroid,
                             std::vector<std::pair<int, cv::Rect>> &trackedObjects)
{
    // Try to reidentify from recently lost tracks
    int reidId = -1;
    float bestIoU = 0.5f;

    for (const auto &[id, history] : recentlyLostTracks)
    {
        if (history.classId == classId &&
            history.framesSinceDelete < 10)
        { // Only consider recently lost tracks
            float iou = calculateIoU(box, history.lastRect);
            if (iou > reidentificationThreshold && iou > bestIoU)
            {
                bestIoU = iou;
                reidId = id;
            }
        }
    }

    Track newTrack;
    if (reidId != -1)
    {
        newTrack.id = reidId;
        recentlyLostTracks.erase(reidId);
    }
    else
    {
        // Ensure sequential IDs
        while (tracks.find(nextId) != tracks.end())
        {
            nextId++;
        }
        newTrack.id = nextId++;
    }

    newTrack.rect = box;
    newTrack.centroid = centroid;
    newTrack.classId = classId;
    newTrack.framesLost = 0;
    newTrack.vx = 0.0f;
    newTrack.vy = 0.0f;
    newTrack.kalman = initializeKalmanFilter(box);

    tracks[newTrack.id] = newTrack;
    trackedObjects.emplace_back(newTrack.id, box);
}

void Tracker::createNewTracks(const std::vector<cv::Rect> &boxes,
                              const std::vector<int> &classIds,
                              const std::vector<cv::Point2f> &centroids,
                              std::vector<std::pair<int, cv::Rect>> &trackedObjects)
{
    for (size_t i = 0; i < boxes.size(); i++)
    {
        createNewTrack(boxes[i], classIds[i], centroids[i], trackedObjects);
    }
}

float Tracker::calculateIoU(const cv::Rect &box1, const cv::Rect &box2) const
{
    int x1 = std::max(box1.x, box2.x);
    int y1 = std::max(box1.y, box2.y);
    int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    int y2 = std::min(box1.y + box1.height, box2.y + box2.height);

    if (x2 <= x1 || y2 <= y1)
        return 0.0f;

    float intersection = static_cast<float>((x2 - x1) * (y2 - y1));
    float area1 = static_cast<float>(box1.width * box1.height);
    float area2 = static_cast<float>(box2.width * box2.height);

    return intersection / (area1 + area2 - intersection);
}

int Tracker::getClassId(int trackId) const
{
    auto it = tracks.find(trackId);
    if (it != tracks.end())
    {
        return it->second.classId;
    }
    return -1; // Invalid class ID
}

bool Tracker::getVelocity(int trackId, float &vx, float &vy) const
{
    auto it = tracks.find(trackId);
    if (it != tracks.end())
    {
        vx = it->second.vx;
        vy = it->second.vy;
        return true;
    }
    return false;
}

cv::Point2f Tracker::calculateCentroid(const cv::Rect &box) const
{
    float centerX = box.x + box.width / 2.0f;
    float centerY = box.y + box.height / 2.0f;
    return cv::Point2f(centerX, centerY);
}

float Tracker::calculateDistance(const cv::Point2f &p1, const cv::Point2f &p2) const
{
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

cv::Ptr<cv::KalmanFilter> Tracker::initializeKalmanFilter(const cv::Rect &box) const
{
    // Create 6-state Kalman filter
    cv::Ptr<cv::KalmanFilter> kalman = cv::makePtr<cv::KalmanFilter>(6, 4, 0);

    // State: [x, y, w, h, vx, vy]
    kalman->transitionMatrix = cv::Mat::eye(6, 6, CV_32F);
    kalman->transitionMatrix.at<float>(0, 4) = FRAME_TIME; // x += vx * dt
    kalman->transitionMatrix.at<float>(1, 5) = FRAME_TIME; // y += vy * dt

    // Initialize state
    float x = box.x + box.width / 2.0f;
    float y = box.y + box.height / 2.0f;
    kalman->statePost.at<float>(0) = x;
    kalman->statePost.at<float>(1) = y;
    kalman->statePost.at<float>(2) = box.width;
    kalman->statePost.at<float>(3) = box.height;
    kalman->statePost.at<float>(4) = 0; // initial velocity = 0
    kalman->statePost.at<float>(5) = 0;

    // Measurement matrix (maps state to measurement)
    kalman->measurementMatrix = cv::Mat::zeros(4, 6, CV_32F);
    kalman->measurementMatrix.at<float>(0, 0) = 1; // x
    kalman->measurementMatrix.at<float>(1, 1) = 1; // y
    kalman->measurementMatrix.at<float>(2, 2) = 1; // width
    kalman->measurementMatrix.at<float>(3, 3) = 1; // height

    // Process noise
    kalman->processNoiseCov = cv::Mat::eye(6, 6, CV_32F) * 0.03;
    kalman->processNoiseCov.at<float>(4, 4) = 0.1; // velocity noise
    kalman->processNoiseCov.at<float>(5, 5) = 0.1;

    // Measurement noise
    kalman->measurementNoiseCov = cv::Mat::eye(4, 4, CV_32F) * 0.1;

    return kalman;
}

void Tracker::updateKalmanFilter(Track &track, const cv::Rect &box)
{
    // Create measurement (center coordinates)
    float center_x = box.x + box.width / 2.0f;
    float center_y = box.y + box.height / 2.0f;

    cv::Mat measurement = (cv::Mat_<float>(4, 1) << center_x,
                           center_y,
                           (float)box.width,
                           (float)box.height);

    // First predict
    cv::Mat prediction = track.kalman->predict();

    // Store previous position
    float prev_x = prediction.at<float>(0);
    float prev_y = prediction.at<float>(1);

    // Then correct
    cv::Mat corrected = track.kalman->correct(measurement);

    // Calculate velocity in pixels/frame
    float dx = (corrected.at<float>(0) - prev_x);
    float dy = (corrected.at<float>(1) - prev_y);

    // Convert to meters/second
    track.vx = dx * PIXELS_TO_METERS / FRAME_TIME;
    track.vy = dy * PIXELS_TO_METERS / FRAME_TIME;

    // Store prediction for next frame
    track.prediction = corrected;
}

cv::Rect Tracker::predictKalmanFilter(Track &track)
{
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