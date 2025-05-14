# VisionTrack Technical Specifications

## Overview

VisionTrack is a C++ application using OpenCV for real-time object tracking and surveillance. This document details the project structure, dependencies, and current implementation.

## Folder Structure

- **`src/`**: Source code.
  - `main.cpp`: Entry point (currently handles webcam capture).
  - `core/`: Detection, tracking, and classification logic.
  - `utils/`: Video handling and alert generation.
  - `ui/`: User interface components.
- **`include/`**: Local OpenCV headers (`opencv2/`).
- **`lib/`**: Local OpenCV `.lib` files for linking.
- **`bin/`**: Local OpenCV `.dll` files for runtime (optional, not tracked by Git).
- **`build/`**: Generated build files (ignored by `.gitignore`).
- **`tests/`**: Unit tests for core functionality.
- **`docs/`**: Documentation (this file, user guide, UML).
- **`data/`**: Sample videos and upload storage.
- **`model/`**: Stores the exported YOLOv8n ONNX model and label definitions.
  - `yolov8n.onnx`: YOLOv8n object detection model in ONNX format.
  - `coco.names`: COCO dataset class labels corresponding to the ONNX model output.
- **`config/`**: Contains configuration files for runtime detection settings.
  - `config.txt`: Specifies model path, thresholds, and default video source.
  
## CMake Configuration

- **File**: `CMakeLists.txt` in the root.
- **Setup**:
  - Uses local OpenCV in `VisionTrack/include/` and `VisionTrack/lib/`.
  - Includes `src/main.cpp` as the executable target.
  - Links to OpenCV libraries (e.g., `opencv_core`, `opencv_videoio`).
- **Build Process**:
  - Configure with CMake GUI or `cmake ..` from `build/`.
  - Generate and build to produce `VisionTrack.exe`.

## Current Features

- **Webcam Capture**: Displays live feed from your webcam in `main.cpp`.
- **Dependencies**: Fully self-contained with OpenCV files in the repo.

## Planned Features

- **Object Detection**: Using Haar cascades or YOLO in `core/detection.cpp`.
- **Tracking**: Kalman filter or similar in `core/tracking.cpp`.
- **Classification**: Basic type detection (e.g., person, vehicle) in `core/classifier.cpp`.
- **Alerts**: Condition-based alerts in `utils/alerts.cpp`.
- **UI**: Display and controls in `ui/interface.cpp`.

## Development Notes

- **OpenCV Version**: Based on the local copy (e.g., 4.6.0, verify with your `.lib` filenames).
- **Compiler**: Tested with Visual Studio 2019 (VC16).
- **Portability**: `.dll` files in `bin/` can be excluded from Git; copy them manually to `build/Debug/` after building.
