/**
 * @file webcam_demo.cpp
 * @brief Live Webcam Demonstration Application for ORB-Standalone
 *
 * Demonstrates how an application captures video frames (using OpenCV VideoCapture),
 * converts them to a grayscale buffer, wraps them in NativeImage, and extracts ORB
 * features using the standalone, OpenCV-independent ORBExtractor library.
 *
 * Controls:
 *   [ESC] or [Q] : Clean exit
 *   [S]          : Save current frame screenshot and binary descriptors
 *   [D]          : Toggle on-screen HUD overlay
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <chrono>
#include <cmath>
#include <sys/stat.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ORBExtractor.h"
#include "ORBTypes.h"

// Color palette for visual multi-scale distinction
static const cv::Scalar OCTAVE_COLORS[8] = {
    cv::Scalar(0, 255, 0),     // Octave 0: Green (Scale 1.0)
    cv::Scalar(0, 255, 255),   // Octave 1: Yellow (Scale 1.20)
    cv::Scalar(0, 165, 255),   // Octave 2: Orange (Scale 1.44)
    cv::Scalar(0, 0, 255),     // Octave 3: Red (Scale 1.73)
    cv::Scalar(255, 0, 255),   // Octave 4: Magenta (Scale 2.07)
    cv::Scalar(255, 255, 0),   // Octave 5: Cyan (Scale 2.49)
    cv::Scalar(255, 128, 0),   // Octave 6: Light Blue (Scale 2.99)
    cv::Scalar(200, 200, 255)  // Octave 7: Light Pink (Scale 3.58)
};

static const float DEG2RAD = 3.14159265358979323846f / 180.0f;

static void ensureDirectory(const std::string& dirPath)
{
    mkdir(dirPath.c_str(), 0755);
}

static std::string padNumber(int num, int digits = 4)
{
    std::ostringstream ss;
    ss << std::setw(digits) << std::setfill('0') << num;
    return ss.str();
}

int main(int argc, char** argv)
{
    int cameraIndex = 0;
    int reqWidth = 640;
    int reqHeight = 480;

    if (argc == 2)
    {
        cameraIndex = std::atoi(argv[1]);
    }
    else if (argc == 3)
    {
        reqWidth = std::atoi(argv[1]);
        reqHeight = std::atoi(argv[2]);
    }
    else if (argc >= 4)
    {
        cameraIndex = std::atoi(argv[1]);
        reqWidth = std::atoi(argv[2]);
        reqHeight = std::atoi(argv[3]);
    }

    std::cout << "=======================================================\n";
    std::cout << "         ORB Standalone - Live Webcam Demo\n";
    std::cout << "=======================================================\n";

    // 1. Initialize Camera Source (Application-level)
    cv::VideoCapture cap;
    std::cout << "[Camera] Opening video capture device: /dev/video" << cameraIndex << "...\n";
    cap.open(cameraIndex);

    if (!cap.isOpened())
    {
        std::cerr << "\n[ERROR] Could not open camera device index " << cameraIndex << "!\n";
        std::cerr << "Troubleshooting:\n";
        std::cerr << "  1. Check connected cameras: ls -la /dev/video*\n";
        std::cerr << "  2. Ensure video permissions: sudo usermod -a -G video $USER\n";
        std::cerr << "  3. Specify an alternate camera device index: ./webcam_demo 1\n";
        return 1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, reqWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, reqHeight);

    double actualW = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double actualH = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actualFps = cap.get(cv::CAP_PROP_FPS);

    std::cout << "\nCamera Initialized Successfully:\n";
    std::cout << "  Device Index:       " << cameraIndex << "\n";
    std::cout << "  Resolution:         " << static_cast<int>(actualW) << " x " << static_cast<int>(actualH) << "\n";
    std::cout << "  Device FPS:         " << (actualFps > 0 ? actualFps : 30.0) << "\n";

    // 2. Initialize Standalone ORB Extractor
    ORB_Standalone::ORBSettings settings;
    settings.nFeatures = 1000;
    settings.scaleFactor = 1.2f;
    settings.nLevels = 8;
    settings.iniThFAST = 20;
    settings.minThFAST = 7;
    settings.patchSize = 31;
    settings.halfPatchSize = 15;
    settings.edgeThreshold = 19;
    settings.fastCellSize = 30;

    std::cout << "\nORB Extractor Configuration:\n";
    std::cout << "  Desired Features:   " << settings.nFeatures << "\n";
    std::cout << "  Pyramid Levels:     " << settings.nLevels << "\n";
    std::cout << "  Scale Factor:       " << settings.scaleFactor << "\n";
    std::cout << "  FAST Threshold:     " << settings.iniThFAST << " (fallback: " << settings.minThFAST << ")\n";
    std::cout << "  Patch Size:         " << settings.patchSize << " px\n";

    std::cout << "\nInteractive Controls:\n";
    std::cout << "  [ESC] or [Q] : Quit application\n";
    std::cout << "  [S]          : Save screenshot and binary descriptors\n";
    std::cout << "  [D]          : Toggle on-screen HUD overlay\n";
    std::cout << "=======================================================\n\n";

    ORB_Standalone::ORBExtractor extractor(settings);

    const std::string windowName = "ORB Standalone - Live Feature Extraction";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

    bool showOverlay = true;
    int screenshotCounter = 1;
    int framesSavedBanner = 0;
    std::string lastSavedMessage = "";

    double avgExtractionMs = 0.0;
    double avgFps = 0.0;
    size_t frameIndex = 0;

    cv::Mat frame;
    cv::Mat gray;
    std::vector<ORB_Standalone::NativeKeyPoint> keypoints;
    std::vector<ORB_Standalone::NativeDescriptor> descriptors;
    ORB_Standalone::ExtractionReport report;

    // Optional environment variable to limit frames for automated testing
    const char* envFrames = std::getenv("ORB_TEST_FRAMES");
    int maxTestFrames = envFrames ? std::atoi(envFrames) : -1;
    if (maxTestFrames > 0)
    {
        std::cout << "[Test Mode] Processing " << maxTestFrames << " frames for automated validation.\n";
    }

    while (true)
    {
        auto tLoopStart = std::chrono::high_resolution_clock::now();

        // A. Capture video frame
        if (!cap.read(frame) || frame.empty())
        {
            std::cerr << "[WARN] Failed to read frame from camera. Retrying...\n";
            cv::waitKey(10);
            continue;
        }

        // B. Convert frame to grayscale (Application responsibility)
        if (frame.channels() == 3)
        {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        }
        else if (frame.channels() == 4)
        {
            cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
        }
        else
        {
            gray = frame;
        }

        // C. Wrap in NativeImage view (Zero-copy wrapper for ORB core)
        ORB_Standalone::NativeImage nativeImg(gray.data, gray.cols, gray.rows, static_cast<int>(gray.step));

        // D. Execute Standalone ORB Extraction
        auto tOrbStart = std::chrono::high_resolution_clock::now();
        extractor.Extract(nativeImg, keypoints, descriptors, &report);
        auto tOrbEnd = std::chrono::high_resolution_clock::now();
        double extractionMs = std::chrono::duration<double, std::milli>(tOrbEnd - tOrbStart).count();

        // E. Render Keypoint Visualization
        cv::Mat display = frame.clone();

        for (const auto& kp : keypoints)
        {
            int oct = std::min(std::max(kp.octave, 0), 7);
            const cv::Scalar& color = OCTAVE_COLORS[oct];
            cv::Point2f center(kp.x, kp.y);

            // Scale circle radius with octave
            float radius = std::max(2.0f, 2.0f + oct * 1.5f);
            cv::circle(display, center, cvRound(radius), color, 1, cv::LINE_AA);

            // Orientation line pointing along intensity centroid angle
            float rad = kp.angle * DEG2RAD;
            float lineLength = 9.0f + oct * 2.0f;
            cv::Point2f tip(kp.x + lineLength * std::cos(rad),
                            kp.y + lineLength * std::sin(rad));

            cv::line(display, center, tip, color, 1, cv::LINE_AA);
            cv::circle(display, tip, 1, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        }

        // F. Compute FPS & smoothed metrics
        auto tLoopEnd = std::chrono::high_resolution_clock::now();
        double loopMs = std::chrono::duration<double, std::milli>(tLoopEnd - tLoopStart).count();
        double currentFps = (loopMs > 0.0) ? (1000.0 / loopMs) : 0.0;

        if (frameIndex == 0)
        {
            avgExtractionMs = extractionMs;
            avgFps = currentFps;
        }
        else
        {
            avgExtractionMs = 0.9 * avgExtractionMs + 0.1 * extractionMs;
            avgFps = 0.9 * avgFps + 0.1 * currentFps;
        }
        ++frameIndex;

        // G. Render On-Screen HUD Overlay
        if (showOverlay)
        {
            int hudWidth = 350;
            int hudHeight = 110;
            cv::Rect hudRect(10, 10, hudWidth, hudHeight);

            if (hudRect.x + hudRect.width <= display.cols && hudRect.y + hudRect.height <= display.rows)
            {
                cv::Mat roi = display(hudRect);
                roi = roi * 0.45; // Subtle translucent darkening
            }

            cv::rectangle(display, hudRect, cv::Scalar(80, 80, 80), 1);

            int baseY = 32;
            int stepY = 22;

            // Header line
            cv::putText(display, "ORB-STANDALONE FEATURE EXTRACTOR",
                        cv::Point(18, baseY), cv::FONT_HERSHEY_SIMPLEX, 0.48,
                        cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

            // Features & Resolution
            std::ostringstream ss1;
            ss1 << "Features: " << keypoints.size() << " / " << settings.nFeatures
                << "  |  Res: " << display.cols << "x" << display.rows;
            cv::putText(display, ss1.str(),
                        cv::Point(18, baseY + stepY), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                        cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

            // Timing & FPS
            std::ostringstream ss2;
            ss2 << "ORB Time: " << std::fixed << std::setprecision(1) << avgExtractionMs << " ms"
                << "  |  FPS: " << std::setprecision(1) << avgFps;
            cv::putText(display, ss2.str(),
                        cv::Point(18, baseY + stepY * 2), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                        cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

            // Key Controls
            cv::putText(display, "[Q/ESC] Exit  |  [S] Save  |  [D] HUD",
                        cv::Point(18, baseY + stepY * 3), cv::FONT_HERSHEY_SIMPLEX, 0.38,
                        cv::Scalar(180, 220, 255), 1, cv::LINE_AA);
        }

        // H. Save Banner Notification
        if (framesSavedBanner > 0)
        {
            --framesSavedBanner;
            cv::putText(display, lastSavedMessage,
                        cv::Point(20, display.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.60,
                        cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }

        cv::imshow(windowName, display);

        // I. Interactive Keyboard Handling
        int key = cv::waitKey(1);

        if (maxTestFrames > 0 && frameIndex >= static_cast<size_t>(maxTestFrames))
        {
            std::cout << "\n[Test Mode] Processed " << frameIndex << " frames successfully. Exiting.\n";
            break;
        }

        if (key == 27 || key == 'q' || key == 'Q')
        {
            std::cout << "\n[User] Exit command received. Shutting down.\n";
            break;
        }
        else if (key == 'd' || key == 'D')
        {
            showOverlay = !showOverlay;
        }
        else if (key == 's' || key == 'S')
        {
            ensureDirectory("output");
            ensureDirectory("output/webcam");

            std::string prefix = "output/webcam/frame_" + padNumber(screenshotCounter);
            std::string visPath = prefix + "_visual.png";
            std::string rawPath = prefix + "_raw.png";
            std::string binPath = prefix + "_desc.bin";

            cv::imwrite(visPath, display);
            cv::imwrite(rawPath, frame);

            std::ofstream binOut(binPath, std::ios::binary);
            if (binOut.is_open())
            {
                for (const auto& desc : descriptors)
                {
                    binOut.write(reinterpret_cast<const char*>(desc.ptr()), 32);
                }
                binOut.close();
            }

            std::string txtPath = prefix + "_features.txt";
            std::ofstream txtOut(txtPath);
            if (txtOut.is_open())
            {
                txtOut << "# ORB-Standalone Feature Data\n";
                txtOut << "# Frame: " << screenshotCounter << "\n";
                txtOut << "# Width: " << frame.cols << " Height: " << frame.rows << "\n";
                txtOut << "# Keypoints: " << keypoints.size() << "\n";
                txtOut << "# Format: index x y octave angle response size\n";
                for (size_t i = 0; i < keypoints.size(); ++i)
                {
                    const auto& kp = keypoints[i];
                    txtOut << i << " " << kp.x << " " << kp.y << " "
                           << kp.octave << " " << kp.angle << " "
                           << kp.response << " " << kp.size << "\n";
                }
                txtOut.close();
            }

            lastSavedMessage = "SAVED: frame_" + padNumber(screenshotCounter) + " (" +
                               std::to_string(keypoints.size()) + " features)";
            framesSavedBanner = 40;

            std::cout << "[SAVE] Saved frame " << screenshotCounter << " ("
                      << keypoints.size() << " keypoints, "
                      << descriptors.size() * 32 << " bytes descriptors) to output/webcam/\n";

            ++screenshotCounter;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    std::cout << "\n=======================================================\n";
    std::cout << "         Webcam Demo Session Completed\n";
    std::cout << "=======================================================\n";
    std::cout << "Total Frames Processed: " << frameIndex << "\n";
    std::cout << "Average Extraction Time:" << std::fixed << std::setprecision(2) << avgExtractionMs << " ms\n";
    std::cout << "Average Processing FPS: " << std::setprecision(1) << avgFps << " FPS\n";

    return 0;
}
