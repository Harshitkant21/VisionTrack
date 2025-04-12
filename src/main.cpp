// //! for main file

#include "detection.h"
#include "config.h"
#include "tracking.h" // Now includes Kalman filter tracking
#include <iostream>
#include <iomanip> // For std::fixed and std::setprecision
#include <chrono>  // For FPS calculation

int main(int argc, char **argv)
{
    // === 1. Parse arguments ===
    std::string configFile = "config.txt";
    std::string videoPath = "";
    // std::string source = "webcam"; // default
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
    // for (int i = 1; i < argc; ++i) {
    //     std::string arg = argv[i];
    //     if (arg.find("--source=") != std::string::npos) {
    //         source = arg.substr(arg.find("=") + 1);
    //     } else if (arg.find("--video=") != std::string::npos) {
    //         videoPath = arg.substr(arg.find("=") + 1);
    //     }
    // }

    // // Choose camIndex based on selected source
    // if (source == "webcam") {
    //     camIndex = 0; // usually laptop webcam
    // } else if (source == "phone") {
    //     camIndex = 2; // usually Camo, but verify via test
    // } else {
    //     std::cerr << "Invalid source specified. Use --source=webcam or --source=phone" << std::endl;
    //     return -1;
    // }

    // === 2. Load configuration ===
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

    // === 3. Get values from config ===
    std::string modelPath = config.get("model_path");
    std::string classesPath = config.get("classes_path");
    float confThreshold = config.getFloat("confidence_threshold", 0.25f);
    float nmsThreshold = config.getFloat("nms_threshold", 0.45f);

    // Get tracking parameters from config (with defaults if not present)
    int maxDistThreshold = config.getInt("tracking_max_distance", 50);
    int maxDisappeared = config.getInt("tracking_max_frames_lost", 5);

    // Additional tracking parameters for Kalman filter
    bool showTrajectories = config.getBool("show_trajectories", true);
    int trajectoryLength = config.getInt("trajectory_length", 10);
    bool showVelocityVectors = config.getBool("show_velocity_vectors", true);

    // === 4. Try to use GPU acceleration if available ===
    bool useGPU = config.getBool("use_gpu", true);

    // === 5. Initialize detector ===
    Detection detector(modelPath, classesPath, confThreshold, nmsThreshold);

    // === 6. Initialize tracker ===
    Tracker tracker(maxDistThreshold, maxDisappeared);
    std::cout << "Initialized Kalman filter tracker with max distance: " << maxDistThreshold
              << ", max disappeared: " << maxDisappeared << std::endl;

    // === 7. Initialize video capture ===
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
    double videoFPS = cap.get(cv::CAP_PROP_FPS);
    if (videoFPS < 1.0)
        videoFPS = 30.0; // Default FPS if not available
    std::cout << "Video FPS: " << videoFPS << std::endl;

    // === 8. Performance monitoring variables ===
    int frameCount = 0;
    auto startTime = std::chrono::steady_clock::now();
    float fps = 0.0f;

    // Track previous positions for trajectory visualization
    std::unordered_map<int, std::vector<cv::Point>> trajectories;

    // === 9. Main loop ===
    cv::Mat frame;
    std::vector<cv::Rect> boxes;
    std::vector<int> classIds;
    std::vector<float> confidences;

    while (true)
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

        // Debug output for detections (uncomment if needed)
        /*
        std::cout << "Detections in this frame: " << boxes.size() << std::endl;
        for (size_t i = 0; i < boxes.size(); ++i)
        {
            std::cout << "Box " << i << ": " << boxes[i].x << ", " << boxes[i].y
                      << ", " << boxes[i].width << ", " << boxes[i].height
                      << " Class: " << classIds[i]
                      << " Conf: " << confidences[i] << std::endl;
        }

        // Debug output for tracks
        std::cout << "Active tracks in this frame: " << tracks.size() << std::endl;
        */

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

                if (overlap > 0.7f)
                { // If 70% overlap, assume it's the same object
                    confidence = confidences[i];
                    break;
                }
            }

            // Get velocity information from Kalman filter
            float vx = 0.0f, vy = 0.0f;
            bool hasVelocity = tracker.getVelocity(trackId, vx, vy);

            // Calculate color based on velocity (red=fast, green=slow)
            cv::Scalar color;
            if (hasVelocity)
            {
                float speed = std::sqrt(vx * vx + vy * vy);
                float maxSpeed = 20.0f; // Adjust based on your expected speed range
                float speedRatio = std::min(speed / maxSpeed, 1.0f);

                // Color gradient from green (slow) to yellow to red (fast)
                color = cv::Scalar(
                    0,                         // B
                    255 * (1.0f - speedRatio), // G
                    255                        // R
                );
            }
            else
            {
                color = cv::Scalar(0, 255, 0); // Default green
            }

            // Draw bounding box with color based on velocity
            cv::rectangle(frame, box, color, 2);

            // Create label with ID, class name, confidence and speed
            std::stringstream ss;
            ss << "ID " << trackId << ": " << className;

            if (confidence > 0)
            {
                ss << " " << std::fixed << std::setprecision(0) << confidence * 100 << "%";
            }

            if (hasVelocity)
            {
                float speed = std::sqrt(vx * vx + vy * vy);
                ss << " " << std::fixed << std::setprecision(1) << speed << "px/f";
            }

            std::string label = ss.str();

            // Draw label
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::rectangle(frame,
                          cv::Point(box.x, box.y - labelSize.height - baseLine - 10),
                          cv::Point(box.x + labelSize.width, box.y),
                          cv::Scalar(255, 255, 255), cv::FILLED);
            cv::putText(frame, label, cv::Point(box.x, box.y - baseLine - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

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
            if (showVelocityVectors && hasVelocity && (std::abs(vx) > 0.5 || std::abs(vy) > 0.5))
            {
                cv::Point future = center + cv::Point(vx * 10, vy * 10); // Predict 10 frames ahead
                cv::arrowedLine(frame, center, future, cv::Scalar(0, 255, 255), 2);
            }
        }

        // Calculate and display FPS
        frameCount++;
        auto currentTime = std::chrono::steady_clock::now();
        float timeDiff = std::chrono::duration<float>(currentTime - startTime).count();

        if (timeDiff >= 1.0f)
        {
            fps = frameCount / timeDiff;
            frameCount = 0;
            startTime = currentTime;
        }

        // Draw FPS counter on frame
        std::string fpsText = "FPS: " + std::to_string(static_cast<int>(fps));
        cv::putText(frame, fpsText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                    1.0, cv::Scalar(0, 255, 0), 2);

        // Display frame
        cv::imshow("VisionTrack", frame);

        // Calculate how much time we should wait to maintain consistent framerate
        auto frameEndTime = std::chrono::steady_clock::now();
        int processingTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 frameEndTime - frameStartTime)
                                 .count();

        int waitTime = std::max(1, static_cast<int>(1000 / videoFPS) - processingTime);

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