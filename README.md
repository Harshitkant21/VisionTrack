# VisionTrack

## Overview
VisionTrack is a real-time object tracking and surveillance system built with C++ and OpenCV. It uses webcam input to detect, track, and classify objects (e.g., people, vehicles), providing tracking info, alerts, and video recording capabilities.

## Prerequisites
- **C++ Compiler**: Visual Studio 2019 (VC16) or compatible.
- **CMake**: Version 3.10 or higher.
- **Git**: For cloning and contributing to the repository.
- **Webcam**: Required for live testing.

## Setup Instructions
1. **Clone the Repository**:
   - Run `git clone https://github.com/Harshitkant21/VisionTrack.git` .
   - Navigate to the project folder: `cd VisionTrack`.

2. **Configure with CMake**:
   - Open CMake GUI.
   - Set "Where is the source code" to `VisionTrack/`.
   - Set "Where to build the binaries" to `VisionTrack/build/`.
   - Click "Configure" (select your compiler, e.g., Visual Studio 16 2019).
   - Click "Generate" to create build files.

3. **Build the Project**:
   - Open `VisionTrack/build/VisionTrack.sln` in Visual Studio and build (or run `cmake --build build/` from terminal).
   - Ensure `.dll` files from `VisionTrack/bin/` are copied to `VisionTrack/build/Debug/` (or where your `.exe` is).

4. **Run the Application**:
   - Run `build/Debug/VisionTrack.exe` to start the webcam feed.

## Contributing
- See `docs/user_guide.md` for detailed setup and usage.
- Check `docs/tech_specs.md` for technical details.
- Use GitHub Issues to pick tasks, create a branch (`git checkout -b feature-name`), commit changes, and submit a Pull Request to `main`.

