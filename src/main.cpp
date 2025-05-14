// //! for main file

#include "detection.h"
#include "config.h"
#include "tracking.h"
#include "util/alerts.h"
#include "util/video.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <unordered_map>

bool running = true;

int main(int argc, char **argv)
{
    //! === 1. Parse command line arguments ===
    std::string configFile = "config.txt";
    std::string videoPath = "";
    int camIndex = -1;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg.find(".txt") != std::string::npos || arg.find(".cfg") != std::string::npos)
        {
            configFile = arg;
        }
        else if (arg.find(".mp4") != std::string::npos || arg.find(".avi") != std::string::npos)
        {
            videoPath = arg;
        }
        else
        {
            try
            {
                camIndex = std::stoi(arg);
            }
            catch (...)
            {
                videoPath = arg;
            }
        }
    }

    //! === 2. Load configuration ===
    Config config;
    std::vector<std::string> searchPaths = {
        "./", "../", "../../",
        "./config/", "../config/", "../../config/"};

    bool configLoaded = false;
    for (const auto &path : searchPaths)
    {
        if (config.loadFromFile(path + configFile))
        {
            std::cout << "Loaded configuration from " << path + configFile << std::endl;
            configLoaded = true;
            break;
        }
    }

    if (!configLoaded)
    {
        std::cout << "Using default configuration" << std::endl;
    }

    //! === 3. Get values from config ===

    std::string modelPath = config.get("model_path");
    std::string classesPath = config.get("classes_path");
    // Detection parameters
    float confThreshold = config.getFloat("confidence_threshold", 0.25f);
    float nmsThreshold = config.getFloat("nms_threshold", 0.45f);
    int maxDistThreshold = config.getInt("tracking_max_distance", 50);
    int maxDisappeared = config.getInt("tracking_max_frames_lost", 5);
    float overlapThreshold = config.getFloat("tracking_overlap_threshold", 0.7f);
    float reidThreshold = config.getFloat("reidentification_threshold", 0.7f);
    int maxHistoryFrames = config.getInt("max_frames_keep_history", 30);
    float pixelsToMeters = config.getFloat("pixels_to_meters", 0.1f);
    float targetFPS = config.getFloat("video_fps", 30.0f);
    float frameTime = 1.0f / targetFPS;

    // Speed parameters
    float minSpeedKmh = config.getFloat("min_speed_kmh", 1.0f);
    float maxSpeedKmh = config.getFloat("max_speed_kmh", 150.0f);
    float speedNormalization = config.getFloat("speed_normalization_factor", 50.0f);

    // Display parameters
    bool showTrajectories = config.getBool("show_trajectories", true);
    int trajectoryLength = config.getInt("trajectory_length", 10);
    bool showVelocityVectors = config.getBool("show_velocity_vectors", true);
    float velocityScale = config.getFloat("velocity_vector_scale", 2.0f);
    float fontScale = config.getFloat("font_scale", 0.5f);
    int fontThickness = config.getInt("font_thickness", 1);

    // Alert parameters
    float speedLimitKmh = config.getFloat("speed_limit", 50.0f);
    float stoppedTimeThreshold = config.getFloat("stopped_time_threshold", 30.0f);
    float restrictedAreaThreshold = config.getFloat("restricted_area_threshold", 0.5f);
    int alertDisplayTime = config.getInt("alert_display_time", 60);

    // Recording parameters
    std::string outputDir = config.get("video_output_path", "outputs/");
    std::string codec = config.get("video_codec", "XVID");
    bool recordOnStartup = config.getBool("record_on_startup", false);
    bool recordWithAnnotations = config.getBool("record_with_annotations", true);

    //! === 4. Initialize detector ===
    Detection detector(modelPath, classesPath, confThreshold, nmsThreshold);

    //! === 5. Initialize tracker ===
    Tracker tracker(maxDistThreshold, maxDisappeared, pixelsToMeters, maxHistoryFrames, reidThreshold, frameTime);

    //! === 6. Initialize alert manager ===
    AlertManager alertManager(speedLimitKmh, stoppedTimeThreshold, restrictedAreaThreshold);

    //! === 7. Initialize video capture ===
    cv::VideoCapture cap;

    if (!videoPath.empty())
    {
        cap.open(videoPath);
        std::cout << "Opened video file: " << videoPath << std::endl;
    }
    else
    {
        if (camIndex == -1)
            camIndex = config.getInt("default_source", 0);

        cap.open(camIndex);
        std::cout << "Opened webcam index: " << camIndex << std::endl;
    }

    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open video source" << std::endl;
        return -1;
    }

    // Get video properties for FPS calculation
    double actualFPS = cap.get(cv::CAP_PROP_FPS);
    if (actualFPS < 1.0)
        actualFPS = targetFPS; // Default FPS if not available
    std::cout << "Video FPS: " << actualFPS << std::endl;

    // Track previous positions for trajectory visualization
    std::unordered_map<int, std::vector<cv::Point>> trajectories;

    //! === 8. Main loop ===
    cv::Mat frame;
    std::vector<cv::Rect> boxes;
    std::vector<int> classIds;
    std::vector<float> confidences;
    cv::Rect restrictedZone(100, 100, 200, 200);

    while (running)
    {
        // Measure start time for this frame
        auto frameStartTime = std::chrono::steady_clock::now();

        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "End of video stream" << std::endl;
            break;
        }

        // Detect objects
        detector.detect(frame, boxes, classIds, confidences);

        // Update tracking with new detections
        std::vector<std::pair<int, cv::Rect>> tracks = tracker.update(boxes, classIds);

        // Draw tracked objects
        for (const auto &[trackId, box] : tracks)
        {
            // Get class ID for this track
            int classId = tracker.getClassId(trackId);
            std::string className = classId < detector.getClassNames().size() ? detector.getClassNames()[classId] : "Unknown";

            // Find confidence for this detection (best guess from current frame)
            float confidence = 0.0f;
            for (size_t i = 0; i < boxes.size(); i++)
            {
                // Simple check: if the boxes overlap significantly, use that confidence
                cv::Rect intersection = box & boxes[i];
                float overlap = (intersection.width * intersection.height) /
                                static_cast<float>(box.width * box.height);

                if (overlap > overlapThreshold)
                { // If 70% overlap, assume it's the same object
                    confidence = confidences[i];
                    break;
                }
            }

            // Get velocity information from Kalman filter
            float vx = 0.0f, vy = 0.0f;
            bool hasVelocity = tracker.getVelocity(trackId, vx, vy);

            // Calculate speed in m/s
            float speed_ms = 0.0f;
            if (hasVelocity)
            {
                speed_ms = std::sqrt(vx * vx + vy * vy);
            }

            // Convert to km/h
            float speed_kmh = speed_ms * 3.6f;

            // Choose color based on speed
            cv::Scalar color;
            if (hasVelocity && speed_ms > 0.5f)
            {
                // Red for fast, yellow for medium, green for slow
                float normalizedSpeed = std::min(speed_kmh / speedNormalization, 1.0f); // Normalize to 50 km/h
                color = cv::Scalar(0, 255 * (1 - normalizedSpeed), 255);
            }
            else
            {
                color = cv::Scalar(0, 255, 0); // Green for stationary/slow objects
            }

            // Draw bounding box with color based on velocity
            cv::rectangle(frame, box, color, 2);

            // Create label with ID, class name, confidence and speed
            std::stringstream ss;
            ss << "ID " << trackId << ": " << className;

            if (hasVelocity)
            {
                // float speed_ms = std::sqrt(vx * vx + vy * vy);
                float speed_kmh = speed_ms * 3.6f; // Convert m/s to km/h

                // Only show speed if it's reasonable
                if (speed_kmh > minSpeedKmh && speed_kmh < maxSpeedKmh)
                { // Reasonable speed range
                    ss << " " << std::fixed << std::setprecision(1) << speed_kmh << " km/h";

                    // Color based on speed
                    float normalizedSpeed = std::min(speed_kmh / 50.0f, 1.0f);
                    color = cv::Scalar(0, 255 * (1 - normalizedSpeed), 255);
                }
                else
                {
                    color = cv::Scalar(0, 255, 0); // Green for stationary/invalid speed
                }
            }

            // Check for alerts
            if (hasVelocity)
            {
                // Check speed violation
                alertManager.checkSpeedViolation(trackId, className, speed_kmh, box);

                // Check restricted area
                alertManager.checkRestrictedArea(trackId, className, box, restrictedZone);

                // Check stopped vehicle
                if (speed_ms < 0.5f)
                {
                    cv::Point2f pos(box.x + box.width / 2.0f, box.y + box.height / 2.0f);
                    alertManager.checkStoppedVehicle(trackId, className, pos, box);
                }
            }

            // Clean old alerts
            alertManager.clearOldAlerts(60); // Remove alerts older than 60 seconds

            // Draw alerts
            alertManager.drawAlerts(frame);

            // Get active alerts as JSON for UI
            std::string alertsJson = alertManager.getAlertsAsJson();

            std::string label = ss.str();

            // Draw label
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::rectangle(frame,
                          cv::Point(box.x, box.y - labelSize.height - baseLine - 10),
                          cv::Point(box.x + labelSize.width, box.y),
                          cv::Scalar(255, 255, 255), cv::FILLED);
            cv::putText(frame, label, cv::Point(box.x, box.y - baseLine - 5),
                        cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 0, 0), fontThickness);

            // Store trajectory point
            cv::Point center(box.x + box.width / 2, box.y + box.height / 2);
            if (trajectories.find(trackId) == trajectories.end())
            {
                trajectories[trackId] = std::vector<cv::Point>();
            }
            trajectories[trackId].push_back(center);

            // Limit trajectory length
            if (trajectories[trackId].size() > trajectoryLength)
            {
                trajectories[trackId].erase(trajectories[trackId].begin());
            }

            // Draw trajectory
            if (showTrajectories && trajectories[trackId].size() > 1)
            {
                for (size_t i = 1; i < trajectories[trackId].size(); i++)
                {
                    // Calculate color based on recency (more recent = brighter)
                    float alpha = static_cast<float>(i) / trajectories[trackId].size();
                    cv::Scalar pointColor = color * alpha + cv::Scalar(0, 0, 255) * (1.0f - alpha);

                    cv::line(frame, trajectories[trackId][i - 1], trajectories[trackId][i], pointColor, 2);
                }
            }

            // Draw predicted trajectory (if object has velocity and prediction enabled)
            if (showVelocityVectors && hasVelocity && speed_ms > 0.5f)
            {
                cv::Point center(box.x + box.width / 2, box.y + box.height / 2);
                cv::Point future = center + cv::Point(static_cast<int>(vx * velocityScale), static_cast<int>(vy * velocityScale));
                cv::arrowedLine(frame, center, future, cv::Scalar(0, 255, 255), 2);
            }
        }

        // Calculate and display FPS
        auto frameEndTime = std::chrono::steady_clock::now();
        float currentFPS = 1000.0f / std::chrono::duration_cast<std::chrono::milliseconds>(
                                         frameEndTime - frameStartTime)
                                         .count();
        cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(currentFPS)),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 255, 0), 2);

        // Display frame
        cv::imshow("VisionTrack", frame);

        // Calculate how much time we should wait to maintain consistent framerate
        int processingTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 frameEndTime - frameStartTime)
                                 .count();

        int waitTime = std::max(1, static_cast<int>(1000 / actualFPS) - processingTime);

        // Check for user input (ESC to exit)
        char key = cv::waitKey(waitTime);
        if (key == 27)
        { // ESC key
            break;
        }
        else if (key == 't' || key == 'T')
        {
            // Toggle trajectory display
            showTrajectories = !showTrajectories;
        }
        else if (key == 'v' || key == 'V')
        {
            // Toggle velocity vector display
            showVelocityVectors = !showVelocityVectors;
        }
    }

    // Clean up
    cap.release();
    cv::destroyAllWindows();
    return 0;
}