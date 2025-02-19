

#include <argus_camera_ros/multi_camera.hpp>
#include <array>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <thread>

#include "nvbufsurface.h"
using namespace std::chrono_literals;

MultiCamera::MultiCamera(Config &config)
    : config_(config),
      initialized(false),
      streams_(config_.device_ids.size()),
      sessions_(config_.device_ids.size()),
      requests_(config_.device_ids.size()),
      consumers_(config_.device_ids.size()),
      raw_buffer_(config_.buffer_size) {}

MultiCamera::~MultiCamera() {
    stop_capture();
    provider_.reset();
}

void MultiCamera::start_capture() {
    running_ = true;
    capture_thread_ = std::thread(&MultiCamera::capture_thread_execution, this);
    processing_thread_ = std::thread(&MultiCamera::processing_thread_execution, this);

    for (uint32_t i = 0; i < streams_.size(); i++) {
        ICaptureSession *iCaptureSession =
            interface_cast<ICaptureSession>(sessions_[i]);
        uint32_t frameId = iCaptureSession->repeat(requests_[i].get());
    }
}

void MultiCamera::stop_capture() {
    printf("REQUESTED STOP\n");
    running_ = false;
    raw_buffer_.stop();
    // processed_buffer_.stop();

    for (uint32_t i = 0; i < streams_.size(); i++) {
        ICaptureSession *iCaptureSession =
            interface_cast<ICaptureSession>(sessions_[i]);
        iCaptureSession->stopRepeat();
    }

    processing_thread_.join();
    capture_thread_.join();
}

void MultiCamera::wait() {
    for (uint32_t i = 0; i < streams_.size(); i++) {
        ICaptureSession *iCaptureSession =
            interface_cast<ICaptureSession>(sessions_[i]);
        iCaptureSession->waitForIdle();
    }
}

bool MultiCamera::getProcessedFrames(std::vector<cv::Mat> &frames) {
    return false;
    // return processed_buffer_.pop(frames);
}

bool MultiCamera::init() {
    provider_ = UniqueObj<CameraProvider>(CameraProvider::create());
    ICameraProvider *provider_i = interface_cast<ICameraProvider>(provider_);
    if (!provider_i) {
        std::cerr << "Failed to get Camera Provider interface" << std::endl;
    }

    std::vector<CameraDevice *> devices;
    provider_i->getCameraDevices(&devices);
    if (devices.empty()) {
        std::cerr << "Failed to get devices" << std::endl;
        return false;
    }

    // TODO: Make use of the device_ids from config

    for (int i = 0; i < devices.size(); i++) {
        CameraDevice *device = devices.at(i);

        Status status;
        sessions_[i].reset(provider_i->createCaptureSession(device, &status));

        ICaptureSession *iCaptureSession =
            interface_cast<ICaptureSession>(sessions_[i]);

        IEventProvider *iEventProvider = interface_cast<IEventProvider>(sessions_[i]);
        if (!iEventProvider || !iEventProvider) {
            std::cerr << "Failed to create CaptureSession" << std::endl;
            ;
            return false;
        }

        /* Create the OutputStream */
        UniqueObj<OutputStreamSettings> streamSettings(
            iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
        IEGLOutputStreamSettings *iEglStreamSettings =
            interface_cast<IEGLOutputStreamSettings>(streamSettings);
        if (!iEglStreamSettings) {
            std::cout << "Failed to create EglOutputStreamSettings" << std::endl;
            return false;
        }

        iEglStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
        iEglStreamSettings->setResolution(
            Size2D<uint32_t>(config_.image_width, config_.image_height));

        streams_[i].reset(iCaptureSession->createOutputStream(streamSettings.get()));

        /* Create capture request and enable the output stream */
        requests_[i].reset(iCaptureSession->createRequest());
        IRequest *iRequest = interface_cast<IRequest>(requests_[i]);
        if (!iRequest) {
            std::cerr << "Failed to create Request" << std::endl;
            return false;
        }
        iRequest->enableOutputStream(streams_[i].get());

        ISourceSettings *iSourceSettings =
            interface_cast<ISourceSettings>(iRequest->getSourceSettings());
        if (!iSourceSettings) {
            std::cerr << "Failed to get ISourceSettings interface" << std::endl;
            return false;
        }

        iSourceSettings->setFrameDurationRange(
            Range<uint64_t>(1e9 / config_.framerate));

        consumers_[i].reset(FrameConsumer::create(streams_[i].get()));
    }

    std::cout << "Finished initialization. Streams: " << streams_.size() << std::endl;

    initialized = true;
    return true;
}

void MultiCamera::capture_thread_execution() {
    IEGLOutputStream *egl_output_streams[config_.device_ids.size()];
    IFrameConsumer *frame_consumers[config_.device_ids.size()];

    for (int i = 0; i < streams_.size(); i++) {
        egl_output_streams[i] = interface_cast<IEGLOutputStream>(streams_[i]);
        frame_consumers[i] = interface_cast<IFrameConsumer>(consumers_[i]);

        if (!frame_consumers[i]) {
            std::cerr << "[Capture Thread] Failed to get IFrameConsumer Interface"
                      << std::endl;
            return;
        }

        if (egl_output_streams[i]->waitUntilConnected(1 * 100000000) !=
            Argus::STATUS_OK) {
            std::cerr << "Stream failed to connect." << std::endl;
            return;
        }
    }
    std::cout << "Capture ready" << std::endl;

    int dma_bufs[streams_.size()];
    NvBufSurface **buf_surfaces = new NvBufSurface *[streams_.size()];
    Size2D<uint32_t> resolution(config_.image_width, config_.image_height);

    while (running_) {
        // Acquire frames
        for (int i = 0; i < streams_.size(); i++) {
            UniqueObj<Frame> frame(frame_consumers[i]->acquireFrame());
            IFrame *iFrame = interface_cast<IFrame>(frame);
            if (!iFrame) {
                std::cout << "Failed to get iframe: " << std::endl;
                break;
            }

            NV::IImageNativeBuffer *iNativeBuffer =
                interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
            if (!iNativeBuffer) {
                delete[] buf_surfaces;
                std::cerr << "IImageNativeBuffer not supported by Image." << std::endl;
                return;
            }

            // INFO: this could be a point of failure.
            // If it fails, simply create the fd everytime
            if (!dma_bufs[i]) {
                dma_bufs[i] = iNativeBuffer->createNvBuffer(
                    egl_output_streams[i]->getResolution(),
                    NVBUF_COLOR_FORMAT_RGBA,
                    NVBUF_LAYOUT_PITCH);

                if (-1 ==
                    NvBufSurfaceFromFd(dma_bufs[i], (void **)(&buf_surfaces[i]))) {
                    delete[] buf_surfaces;
                    std::cerr << "Cannot get NvBufSurface from fd" << std::endl;
                    return;
                }
            } else if (iNativeBuffer->copyToNvBuffer(dma_bufs[i]) != STATUS_OK) {
                delete[] buf_surfaces;
                std::cerr << "Cannot get NvBufSurface from fd" << std::endl;
                return;
            }
        }
        raw_buffer_.push(buf_surfaces);
    }
    // Convert to NvBufSurface
    // Push surface to raw ring buffer
    // std::this_thread::sleep_for(1000ms);
}

void MultiCamera::processing_thread_execution() {
    std::cout << "Processing thread" << std::endl;
    while (running_) {
        std::cout << "[CONSUMER] available: " << raw_buffer_.size() << std::endl;
        NvBufSurface **buf_surfaces;
        if (!raw_buffer_.pop(buf_surfaces)) continue;

        cv::Mat combined(config_.image_height * 2, config_.image_width * 2, CV_8UC3);

        for (int i = 0; i < 4; i++) {
            // Map and sync each buffer for CPU access
            NvBufSurfaceMap(buf_surfaces[i], -1, 0, NVBUF_MAP_READ);
            NvBufSurfaceSyncForCpu(buf_surfaces[i], -1, 0);

            // Convert the buffer to a cv::Mat
            cv::Mat imgbuf(config_.image_height,
                           config_.image_width,
                           CV_8UC4,
                           buf_surfaces[i]->surfaceList->mappedAddr.addr[0]);

            // Convert from RGBA to BGR
            cv::Mat display_img;
            cvtColor(imgbuf, display_img, cv::COLOR_RGBA2BGR);

            // Unmap the buffer
            NvBufSurfaceUnMap(buf_surfaces[i], -1, 0);

            // Calculate the position (row and column) for the image
            int row = i / 2;  // 0 for top row, 1 for bottom row
            int col = i % 2;  // 0 for left column, 1 for right column

            // Define the region of interest (ROI) on the combined canvas
            cv::Rect roi(col * config_.image_width,
                         row * config_.image_height,
                         config_.image_width,
                         config_.image_height);

            // Copy the display image into the ROI
            display_img.copyTo(combined(roi));
        }

        // Display single image

        // NvBufSurfaceMap(buf_surfaces[0], -1, 0, NVBUF_MAP_READ);
        // NvBufSurfaceSyncForCpu(buf_surfaces[0], -1, 0);
        //
        // cv::Mat imgbuf = cv::Mat(config_.image_height,
        //                          config_.image_width,
        //                          CV_8UC4,
        //                          buf_surfaces[0]->surfaceList->mappedAddr.addr[0]);
        // cv::Mat display_img;
        // cvtColor(imgbuf, display_img, cv::COLOR_RGBA2BGR);
        //
        // NvBufSurfaceUnMap(buf_surfaces[0], -1, 0);

        cv::imshow("img", combined);
        cv::waitKey(1);

        // Pop surface from ring buffer
        // convert surface to cv::Mat
        // push to processed ring_buffer
    }
}
