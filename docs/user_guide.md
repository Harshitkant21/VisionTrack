# VisionTrack User Guide

## Introduction
This guide provides step-by-step instructions to set up, build, and run VisionTrack, a real-time object tracking and surveillance system built with C++ and OpenCV. It’s designed for team members to replicate the development environment and contribute collaboratively via GitHub.

## Installation

### Step 1: Clone the Repository
- Open a terminal (e.g., PowerShell, Command Prompt, or Git Bash).
- Run: `git clone https://github.com/Harshitkant21/VisionTrack.git` .
- Navigate to the project folder: `cd VisionTrack`.

### Step 2: Install Prerequisites
- **MinGW**: Install MinGW-w64 (e.g., via MSYS2 or standalone from mingw-w64.org) to provide `gcc`/`g++` for Windows.
  - Add MinGW’s `bin/` folder (e.g., `C:/MinGW/bin/`) to your system PATH.
- **CMake**: Download and install from cmake.org (version 3.10+).
  - Add CMake’s `bin/` folder (e.g., `C:/Program Files/CMake/bin/`) to your system PATH.
- **Git**: Install from git-scm.com if not already present.
- **Webcam**: Ensure your webcam is connected and drivers are installed.

### Step 3: Configure with CMake
- Open a terminal in `D:/Projects/VisionTrack/` (adjust to your project path).
- Run: `cmake -G "MinGW Makefiles" -S . -B build/`
  - This generates MinGW Makefiles in `VisionTrack/build/` using the source in `VisionTrack/`.
- Check the output for errors:
  - If CMake can’t find OpenCV, ensure `VisionTrack/include/` and `VisionTrack/lib/` are in your project (they should be included from the repo).

### Step 4: Build the Project
- Run: `cmake --build build/ --config Release`
  - This builds VisionTrack in Release mode using MinGW (e.g., `mingw32-make`).
- After building, `VisionTrack.exe` will be in `VisionTrack/build/`.

### Step 5: (Optional) Install the Project
- If you want to install VisionTrack to a separate folder:
  - From `VisionTrack/build/`, run: `cmake --build . --target INSTALL --config Release`
  - This copies `VisionTrack.exe` and dependencies to an install directory (default is `VisionTrack/install/` unless changed in `CMakeLists.txt`).
- Check `VisionTrack/install/bin/` (or your install path) for the executable.

### Step 6: Run VisionTrack
- **From Build Folder**:
  - Navigate: `cd build/`
  - Run: `./VisionTrack.exe`
- **From Install Folder** (if installed):
  - Navigate: `cd install/bin/`
  - Run: `./VisionTrack.exe`
- Expected output: A window showing your webcam feed (once implemented).
- Press a key (e.g., 'q') to exit (once added to the code).
- **Note**: If you get a “DLL not found” error, copy `.dll` files from `VisionTrack/bin/` to `VisionTrack/build/` (or `install/bin/` if installed).

## Dependencies
- **OpenCV**: Included locally in the repository:
  - Headers: `VisionTrack/include/opencv2/` (e.g., `core.hpp`, `videoio.hpp`).
  - Libraries: `VisionTrack/lib/` (e.g., `libopencv_core.a`, `libopencv_videoio.a` for MinGW).
  - Binaries: `VisionTrack/bin/` (e.g., `opencv_world460.dll`—not tracked by Git, copy manually if needed).
- No external OpenCV installation is required—just use what’s in the repo.

## Usage
- **Current Features** (as of initial setup):
  - Displays live webcam feed in a window (implemented in `src/main.cpp`).
- **Planned Features**:
  - Object detection (e.g., people, vehicles) using Haar cascades or YOLO.
  - Object tracking with velocity and trajectory (e.g., Kalman filter).
  - Classification of detected objects.
  - Alerts for specific conditions (e.g., restricted areas).
  - Video recording and UI controls.
- Check `README.md` or GitHub Issues for the latest updates.

## Contributing
- **Workflow**:
  1. Pull the latest code: `git pull origin main`.
  2. Create a branch: `git checkout -b feature-name` (e.g., `detection-module`).
  3. Implement your changes in `src/` (e.g., detection in `core/detection.cpp`).
  4. Commit: `git add . && git commit -m "Added feature X"`.
  5. Push: `git push origin feature-name`.
  6. Submit a Pull Request (PR) on GitHub to merge into `main`.
- **Tasks**: Pick tasks from GitHub Issues (e.g., “Implement tracking”).
- **Documentation**: Update this guide or `tech_specs.md` if you add new setup steps or features.

## Troubleshooting
- **CMake Errors**:
  - “Generator not found”: Ensure MinGW’s `bin/` is in your PATH (e.g., `C:/MinGW/bin/`).
  - “OpenCV not found”: Verify `VisionTrack/include/` and `VisionTrack/lib/` exist and match your OpenCV version.
- **Build Errors**:
  - “Missing library”: Check that `lib/` has the correct `.a` files for MinGW (e.g., `libopencv_core.a`).
- **Runtime Errors**:
  - “DLL not found”: Copy `.dll` files from `VisionTrack/bin/` to `build/` (or `install/bin/`).
  - “Webcam not opening”: Test with a standalone OpenCV program to isolate driver issues.
- **Contact**: Ask the team lead (you!) via GitHub Issues if stuck.

## Notes
- This setup uses MinGW Makefiles for Windows. If using Linux/macOS, adjust the generator (e.g., `Unix Makefiles`) and library extensions (e.g., `.so` instead of `.dll`).
- Update this guide as new features are added (e.g., how to trigger alerts).