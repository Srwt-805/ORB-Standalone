# ORB-Standalone

A lightweight, self-contained implementation of the **ORB (Oriented FAST and Rotated BRIEF)** feature extraction pipeline, designed for embedded systems, robotics, and hardware acceleration.

The project was developed as a foundation for building a lightweight Visual SLAM system without depending on OpenCV.

---

## Why ORB-Standalone?

OpenCV is an excellent general-purpose computer vision framework, but a complete OpenCV dependency is unnecessary when only a small set of vision operations is required.

ORB-Standalone provides the required ORB pipeline directly:

```text
Image
  ↓
Image Pyramid
  ↓
FAST Keypoint Detection
  ↓
Keypoint Scoring & Spatial Distribution
  ↓
Orientation Estimation
  ↓
Rotated BRIEF Descriptor
  ↓
ORB Features
```

This makes the implementation easier to control, optimize, port, and eventually accelerate in hardware.

---

## Features

* **Completely OpenCV-independent core**
* Self-contained C++ implementation
* Monocular image support
* Multi-scale image pyramid
* FAST-9 keypoint detection
* Adaptive FAST thresholding
* Spatial feature distribution using quadtree
* Intensity-centroid orientation
* Rotated BRIEF-256 descriptors
* Deterministic output
* No OpenCV runtime dependency in the core
* Designed with embedded/FPGA acceleration in mind

---

## OpenCV Independence

The ORB core does **not** use:

* `cv::Mat`
* `cv::ORB`
* `cv::FAST`
* `cv::BFMatcher`
* OpenCV image-processing functions

Images are handled through lightweight native data structures.

The optional webcam/demo application may use external libraries for camera input, but they are **not part of the ORB core**.

---

## Portability

The core avoids operating-system-specific APIs and is structured to make porting to embedded platforms straightforward.

It is therefore suitable as a foundation for:

* Embedded Linux
* ARM-based systems
* RTOS-based systems
* FPGA + CPU/SoC architectures
* Future bare-metal implementations

**Note:** the current reference implementation is validated on conventional desktop/Linux systems. A specific bare-metal target still requires platform-specific startup, memory management, image acquisition, and runtime support.

---

## Performance

ORB-Standalone is **not intended to outperform highly optimized OpenCV on desktop CPUs**.

Representative 640×480 performance:

| Implementation         | Approx. Time |
| ---------------------- | -----------: |
| OpenCV ORB             |      ~5.1 ms |
| ORB-SLAM3 ORBextractor |    ~18–25 ms |
| ORB-Standalone         |     ~25.2 ms |

The primary goals are:

**independence → portability → deterministic behavior → hardware readiness**

rather than maximum desktop CPU performance.

---

## Hardware Acceleration

The ORB pipeline contains several highly parallel operations that are suitable for FPGA implementation:

* FAST detection
* FAST scoring
* Non-maximum suppression
* Image pyramid generation
* Gaussian filtering
* BRIEF descriptor generation

The long-term goal is to accelerate these computationally intensive stages while keeping higher-level SLAM operations on the processor.

---

## Build

```bash
git clone <repository-url>
cd ORB-Standalone

mkdir build
cd build
cmake ..
make -j$(nproc)
```

Run the available demo/tests according to the project configuration.

---

## Project Structure

```text
ORB-Standalone/
├── include/        # ORB headers
├── src/            # ORB implementation
├── tests/          # Validation and testing
├── examples/       # Example applications
├── build/          # Build directory
├── CMakeLists.txt
├── README.md
└── LICENSE
```

---

## Current Status

**ORB feature extraction: implemented and validated.**

The next stage is to use this ORB frontend as the foundation for a lightweight **monocular Visual SLAM / Visual Odometry system**, followed by hardware acceleration.

---

## License

MIT License.
