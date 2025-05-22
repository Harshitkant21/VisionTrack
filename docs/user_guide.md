# 📔VisionTrack User Guide

## 📘Introduction
Welcome to **VisionTrack** – a real-time object tracking and surveillance system designed for high-performance monitoring using C++ and OpenCV.
> ⚙️ This guide will help you set up the environment, build, and run the application easily.

---

## ⚡ Quick Links

- 🚀 [Installation](#-installation)
- 🧪 [Testing Setup](#-testing-setup)
- 🛠️ [Core Features & Controls](#-core-features-and-controls)
- 📈 [Performance Optimization](#-performance-optimization)
- 🙋‍♂️ [Contributing](#-contributing)
- 🧯 [Troubleshooting](#-troubleshooting)

---

## 🛠️ Installation


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
- **YOLOv8 (Model Preparation Only)**: We use the [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) Python package only to **export the model to ONNX format** for use in our C++ project.

  1. Install the package (in a separate Python environment):
     ```bash
     pip install ultralytics
     ```

  2. Export a pretrained model to ONNX:
     ```bash
      #Load the model
     from ultralytics import YOLO
     model = YOLO("yolo8n.pt")
     ```
     ```bash
     # Export the model to ONNX format
      model.export(format="onnx", dynamic=True)
     ```
     - You can also use `yolov8s.pt` or other variants depending on accuracy and size needs.
     - This command will generate a file like `yolov8n.onnx`.

  3. Move the `.onnx` file into the project under:
     ```
     VisionTrack/model/yolov11n.onnx
     ```

  4. Also include the `coco.names` file (class labels for COCO dataset) in the same `model/` folder:
     ```
     VisionTrack/model/coco.names
     ```

     > You can get this file from [here](https://github.com/pjreddie/darknet/blob/master/data/coco.names) or generate it manually. It must match the labels used in the YOLO model.

  ✅ Once the model is in ONNX format and the `coco.names` file is added, no Python is required during runtime.


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

## 🧰 Dependencies
- **OpenCV**: Included locally in the repository:
  - Headers: `VisionTrack/include/opencv2/` (e.g., `core.hpp`, `videoio.hpp`).
  - Libraries: `VisionTrack/lib/` (e.g., `libopencv_core.a`, `libopencv_videoio.a` for MinGW).
  - Binaries: `VisionTrack/bin/` (e.g., `opencv_world460.dll`—not tracked by Git, copy manually if needed).
- No external OpenCV installation is required—just use what’s in the repo.

## 🎮 Core Features and Controls

### Keyboard Controls
| Key | Function | Visual Indicator |
|-----|----------|------------------|
| R | Start/Stop Recording | - |
| T | Toggle Trajectories | Lines showing path |
| V | Toggle Velocity Vectors | Arrows showing direction |
| A | Toggle Alert Display | Alert boxes |
| ESC/Q | Exit Application | - |


### Alert System
| Alert Type | Visual | Description |
|------------|---------|-------------|
| Speed Violation | ⚠️ Yellow Box | Vehicle exceeding speed limit |
| Restricted Area | 🚫 Red Box | Object in prohibited zone |
| Stopped Vehicle | ⏰ Blue Box | Vehicle stationary too long |

## 🧪 Testing Setup

### Setting up GoogleTest
1. Clone GoogleTest into include directory:
```bash
cd D:\Projects\VisionTrack
git submodule add https://github.com/google/googletest.git include/googletest
```

2. Update CMakeLists.txt:
```cmake
# Add GoogleTest as a subdirectory 
add_subdirectory(include/googletest)

# Add the tests directory
add_subdirectory(tests)
```

3. Create test directory structure:
```
VisionTrack/
├── tests/
│   ├── CMakeLists.txt
│   ├── test_detection.cpp
│   ├── test_tracking.cpp
│   ├── test_alerts.cpp
│   └── test_config.cpp
```

### Running Tests
1. Build with testing enabled:
```bash
cd build
cmake -G "MinGW Makefiles" .. -DBUILD_TESTING=ON
cmake --build .
```

2. Execute all tests:
```bash
ctest --verbose
```

3. Run specific test:
```bash
./tests/test_detection.exe
./tests/test_tracking.exe
./tests/test_alerts.exe
```

### Test Categories
1. **Detection Tests**
   - Model loading validation
   - Object detection accuracy

2. **Tracking Tests**
   - Object persistence
   - Velocity calculation

3. **Alert Tests**
   - Speed violation detection
   - Restricted area monitoring
   - Stopped vehicle alerts



## 📈 Performance Optimization

### Resource Usage
- Monitor Task Manager for:
  - CPU utilization
  - Memory consumption
  - GPU usage (if enabled)

## 🤝 Contributing
- **Workflow**:
  1. Pull the latest code: `git pull origin main`.
  2. Create a branch: `git checkout -b feature-name` (e.g., `detection-module`).
  3. Implement your changes in `src/` (e.g., detection in `core/detection.cpp`).
  4. Commit: `git add . && git commit -m "Added feature X"`.
  5. Push: `git push origin feature-name`.
  6. Submit a Pull Request (PR) on GitHub to merge into `main`.
- **Tasks**: Pick tasks from GitHub Issues (e.g., “Implement tracking”).
- **Documentation**: Update this guide or `tech_specs.md` if you add new setup steps or features.

## 🧯Troubleshooting
- **CMake Errors**:
  - “Generator not found”: Ensure MinGW’s `bin/` is in your PATH (e.g., `C:/MinGW/bin/`).
  - “OpenCV not found”: Verify `VisionTrack/include/` and `VisionTrack/lib/` exist and match your OpenCV version.
- **Build Errors**:
  - “Missing library”: Check that `lib/` has the correct `.a` files for MinGW (e.g., `libopencv_core.a`).
- **Runtime Errors**:
  - “DLL not found”: Copy `.dll` files from `VisionTrack/bin/` to `build/` (or `install/bin/`).
  - “Webcam not opening”: Test with a standalone OpenCV program to isolate driver issues.
- **Contact**: Ask the team lead (you!) via GitHub Issues if stuck.

## 📝 Notes
- This setup uses MinGW Makefiles for Windows. If using Linux/macOS, adjust the generator (e.g., `Unix Makefiles`) and library extensions (e.g., `.so` instead of `.dll`).
- Update this guide as new features are added (e.g., how to trigger alerts).
- Test videos are available in `data/upload/` directory
