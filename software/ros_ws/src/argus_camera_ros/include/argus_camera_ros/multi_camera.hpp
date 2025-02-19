#ifndef MULTI_CAMERA_HPP
#define MULTI_CAMERA_HPP

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <NvBufSurface.h>

#include <atomic>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "argus_camera_ros/ring_buffer.hpp"

using namespace EGLStream;
using namespace Argus;

class MultiCamera {
   public:
    struct Config {
        const std::vector<int> device_ids;  // camera ids
        int sensor_mode;                    // Mode at which the camera captures (0-3)
        int image_width;                    // width at which image is resized to
        int image_height;                   // height at which image is resized to
        int framerate;                      // framerate at which captures will be made
        int buffer_size;  // internal buffer size if images can't be processed in time.
    };

    MultiCamera(Config &config);
    ~MultiCamera();

    bool init();
    void start_capture();
    void stop_capture();
    void wait();
    bool getProcessedFrames(std::vector<cv::Mat> &frames);

   private:
    void capture_thread_execution();
    void processing_thread_execution();

    Config config_;
    bool initialized;

    // Argus resources
    UniqueObj<CameraProvider> provider_;
    std::vector<UniqueObj<CaptureSession>> sessions_;
    std::vector<UniqueObj<OutputStream>> streams_;
    std::vector<UniqueObj<FrameConsumer>> consumers_;
    std::vector<UniqueObj<Request>> requests_;

    // Buffers
    RingBuffer<NvBufSurface **> raw_buffer_;
    // RingBuffer<std::vector<cv::Mat>> processed_buffer_;

    // Threads
    std::thread capture_thread_;
    std::thread processing_thread_;
    std::atomic<bool> running_{false};
};

#endif  // !MULTI_CAMERA_HPP
