# VisionTrack Project Directory Structure
```
└── 📁VisionTrack                       # Root directory of the VisionTrack project
    └── 📁config                        # Stores configuration files for runtime settings
        ├── config.sample.txt 
        ├── config.txt 
    └── 📁data                          # Test and uploaded videos
        └── 📁images
            ├── banner.png
        └── 📁test_videos
            └── 📁tracking
                ├── test1.mp4
        └── 📁upload
            ├── test1.webm
            ├── test2.webm
            ├── test3.mp4
            ├── test4.mp4
            ├── test5.mp4
    └── 📁docs                          # Documentation and UML diagrams
        └── 📁UML Diagrams
            ├── class_diagrams.puml
            ├── component_diagram.puml
            ├── sequence_diagram.puml
        ├── tech_specs.md
        ├── user_guide.md
    └── 📁models                        # YOLO models and class labels
        ├── coco.names
        ├── yolo11n.onnx
        ├── yolov8n.onnx
    └── 📁src                           # Source code for detection, tracking, and GUI
        └── 📁core
            ├── detection.cpp
            ├── detection.h
            ├── hungarian.cpp
            ├── hungarian.h
            ├── tracking.cpp
            ├── tracking.h
        └── 📁util
            ├── alerts.cpp
            ├── alerts.h
            ├── video.cpp
            ├── video.h
        ├── config.cpp
        ├── config.h
        ├── main.cpp
    └── 📁tests                         # Unit tests for various components
        ├── CMakeLists.txt
        ├── test_alerts.cpp
        ├── test_config.cpp
        ├── test_config.h
        ├── test_detection.cpp
        ├── test_tracking.cpp
    ├── .gitignore
    ├── CMakeLists.txt                   # Build configuration
    ├── LICENSE                          # License terms
    └── README.md                        # Project overview and setup instructions