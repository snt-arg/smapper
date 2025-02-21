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

/**
 * @brief MultiCamera class for managing multiple Argus cameras.
 *
 * This class encapsulates the functionality required to initialize and control multiple
 * cameras using the NVIDIA Argus API. It handles camera setup, frame capture, and frame
 * processing. The class utilizes separate threads for capturing raw frames and
 * processing them, and maintains internal buffers for both raw and processed frames.
 *
 * The API documentation can be found here:
 *    - https://docs.nvidia.com/jetson/archives/r36.3/ApiReference/index.html
 */
class MultiCamera {
   public:
    /**
     * @brief Configuration structure for MultiCamera.
     *
     * Contains the parameters used to configure the MultiCamera instance.
     */
    struct Config {
        const std::vector<int> device_ids;  ///< List of camera device IDs to be used.
        int sensor_mode;                    ///< Sensor mode for image capture (0-3).
        int image_width;                    ///< Target width for resized images.
        int image_height;                   ///< Target height for resized images.
        int framerate;                      ///< Framerate at which images are captured.
        int buffer_size;  ///< Size of the internal buffer for unprocessed images.
    };

    /**
     * @brief Constructs a new MultiCamera instance.
     *
     * @param config Configuration parameters for setting up the MultiCamera instance.
     */
    MultiCamera(Config &config);

    /**
     * @brief Destructs the MultiCamera instance.
     *
     * Stops the capture and processing threads, releases camera resources, and performs
     * cleanup.
     */
    ~MultiCamera();

    /**
     * @brief Initializes the MultiCamera instance.
     *
     * This method must be called before starting any capture.
     *
     * Sets up the Argus camera provider, configures camera devices, and prepares
     * internal resources such as capture sessions, streams, and frame consumers.
     *
     * @return true if initialization was successful; false otherwise.
     */
    bool init();

    /**
     * @brief Starts the camera capture process.
     *
     * Requires init() to be called first!
     *
     * Launches the capture thread, which continuously retrieves frames from the
     * cameras.
     */
    void start_capture();

    /**
     * @brief Stops the camera capture process.
     *
     * Signals both the capture and processing threads to terminate, and releases
     * related resources.
     */
    void stop_capture();

    /**
     * @brief Retrieves processed frames.
     *
     * Copies the most recent batch of processed frames from the internal processed
     * buffer to the provided vector.
     *
     * @param frames Reference to a vector where processed frames (as cv::Mat) will be
     * stored.
     * @return true if frames were successfully retrieved; false if no frames are
     * available or an error occurred.
     */
    bool get_frames(std::vector<cv::Mat> &frames);

   private:
    /**
     * @brief Execution function for the capture thread.
     *
     * Continuously captures raw frames from the cameras and stores them into the raw
     * buffer.
     */
    void capture_thread_execution();

    /**
     * @brief Execution function for the processing thread.
     *
     * Continuously processes raw frames from the raw buffer. For the moment, this
     * only converts raw frames into cv::Mat frames.
     */
    void processing_thread_execution();

    /**
     * @brief Blocks until the capture stream (LibArgus) has finished processing all
     * frames.
     *
     * This helper method is used internally to ensure the correct shutdown of this
     * class.
     */
    void wait_();

    /**
     * @brief Retrieves the requested camera devices.
     *
     * Queries the Argus camera provider for available camera devices matching the
     * specified IDs.
     *
     * @param ids Vector of device IDs requested.
     * @param provider_i Pointer to the camera provider interface.
     * @param cameras Vector to store pointers to the retrieved CameraDevice objects.
     * @return true if the requested cameras were successfully retrieved; false
     * otherwise.
     */
    bool get_requested_cameras_(std::vector<int> ids,
                                ICameraProvider *provider_i,
                                std::vector<CameraDevice *> &cameras);

    /**
     * @brief Retrieves available sensor modes for the cameras.
     *
     * For each camera in the provided vector, retrieves the available sensor modes.
     *
     * @param cameras Vector of camera devices.
     * @param sensor_modes Vector to store the sensor modes available for each camera.
     * @return true if sensor modes were successfully retrieved for all cameras; false
     * otherwise.
     */
    bool get_sensor_modes_(std::vector<CameraDevice *> &cameras,
                           std::vector<std::vector<ISensorMode *>> &sensor_modes);

    /**
     * @brief Sets up the cameras using the Argus CameraProvider.
     *
     * Configures the specified cameras by creating capture sessions, output streams,
     * and setting up frame consumers.
     *
     * @param provider_i Pointer to the camera provider interface.
     * @param cameras Vector of camera devices to be set up.
     * @return true if the cameras were successfully set up; false otherwise.
     */
    bool setup_cameras_(ICameraProvider *provider_i,
                        std::vector<CameraDevice *> &cameras);

    Config config_;    ///< Configuration parameters for the MultiCamera instance.
    bool initialized;  ///< Flag indicating whether the MultiCamera instance was
                       ///< successfully initialized.

    // Argus resources
    UniqueObj<CameraProvider> provider_;  ///< Argus camera provider.
    std::vector<UniqueObj<CaptureSession>>
        sessions_;  ///< Capture sessions for each camera.
    std::vector<UniqueObj<OutputStream>>
        streams_;  ///< Output streams associated with each capture session.
    std::vector<UniqueObj<FrameConsumer>>
        consumers_;  ///< Frame consumers for each output stream.
    std::vector<UniqueObj<Request>> requests_;  ///< Capture requests for each camera.

    // Buffers
    RingBuffer<NvBufSurface **>
        raw_buffer_;  ///< Buffer for storing raw captured frames.
    RingBuffer<std::vector<cv::Mat>>
        processed_buffer_;  ///< Buffer for storing processed frames.

    // Threads
    std::thread capture_thread_;  ///< Thread responsible for capturing frames.
    std::thread
        processing_thread_;  ///< Thread responsible for processing captured frames.
    std::atomic<bool> running_{false};  ///< Atomic flag indicating whether capture and
                                        ///< processing threads should continue running.
};

#endif  // !MULTI_CAMERA_HPP
