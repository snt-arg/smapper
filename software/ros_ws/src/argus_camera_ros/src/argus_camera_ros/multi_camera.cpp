

#include <algorithm>
#include <argus_camera_ros/multi_camera.hpp>
#include <array>
#include <complex>
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
      raw_buffer_(config_.buffer_size),
      processed_buffer_(config_.buffer_size) {}

MultiCamera::~MultiCamera() {
    stop_capture();
    provider_.reset();
}

void MultiCamera::start_capture() {
    if (!initialized) return;
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
    running_ = false;
    raw_buffer_.stop();
    processed_buffer_.stop();

    for (uint32_t i = 0; i < streams_.size(); i++) {
        ICaptureSession *iCaptureSession =
            interface_cast<ICaptureSession>(sessions_[i]);
        iCaptureSession->stopRepeat();
    }

    processing_thread_.join();
    capture_thread_.join();

    wait_();
}

void MultiCamera::wait_() {
    for (uint32_t i = 0; i < streams_.size(); i++) {
        ICaptureSession *iCaptureSession =
            interface_cast<ICaptureSession>(sessions_[i]);
        iCaptureSession->waitForIdle();
    }
}

bool MultiCamera::get_frames(std::vector<cv::Mat> &frames) {
    return processed_buffer_.pop(frames);
}

bool MultiCamera::get_requested_cameras_(std::vector<int64> ids,
                                         ICameraProvider *provider_i,
                                         std::vector<CameraDevice *> &cameras) {
    std::vector<CameraDevice *> devices;
    provider_i->getCameraDevices(&devices);
    if (devices.empty()) {
        std::cerr << "Failed to get devices" << std::endl;
        return false;
    }

    if (ids.size() > devices.size()) {
        std::cerr << "There are only " << devices.size() << "cameras available"
                  << std::endl;
        return false;
    }

    for (int id : ids) {
        if (id < 0 || id >= devices.size()) {
            std::cerr << id << " is not a valid camera id" << std::endl;
            return false;
        }
    }

    for (int i = 0; i < devices.size(); i++) {
        if (std::find(config_.device_ids.begin(), config_.device_ids.end(), i) !=
            config_.device_ids.end()) {
            cameras.push_back(devices[i]);
        }
    }

    return true;
}

bool MultiCamera::get_sensor_modes_(
    std::vector<CameraDevice *> &cameras,
    std::vector<std::vector<SensorMode *>> &sensor_modes) {
    for (int i = 0; i < cameras.size(); i++) {
        ICameraProperties *properties = interface_cast<ICameraProperties>(cameras[i]);
        if (!properties) return false;

        std::vector<SensorMode *> modes;
        properties->getAllSensorModes(&modes);
        std::vector<SensorMode *> modes_i;

        sensor_modes.push_back(modes);
    }

    return true;
}

bool MultiCamera::setup_cameras_(ICameraProvider *provider_i,
                                 std::vector<CameraDevice *> &cameras) {
    std::vector<std::vector<SensorMode *>> sensor_modes;
    if (!get_sensor_modes_(cameras, sensor_modes)) return false;

    if (config_.sensor_mode > sensor_modes[0].size() || config_.sensor_mode < 0) {
        std::cout << "Sensor mode " << config_.sensor_mode << " not available."
                  << std::endl;
        // TODO: printout the available modes
        return false;
    }

    // TODO: This function can be decomposed into smaller sub-functions
    for (int i = 0; i < cameras.size(); i++) {
        CameraDevice *device = cameras[i];

        sessions_[i].reset(provider_i->createCaptureSession(device));

        ICaptureSession *capture_session_i =
            interface_cast<ICaptureSession>(sessions_[i]);

        IEventProvider *event_provider_i = interface_cast<IEventProvider>(sessions_[i]);
        if (!event_provider_i || !event_provider_i) {
            std::cerr << "Failed to create CaptureSession" << std::endl;
            return false;
        }

        UniqueObj<OutputStreamSettings> stream_settings(
            capture_session_i->createOutputStreamSettings(STREAM_TYPE_EGL));
        IEGLOutputStreamSettings *egl_stream_settings_i =
            interface_cast<IEGLOutputStreamSettings>(stream_settings);
        if (!egl_stream_settings_i) {
            std::cout << "Failed to create EglOutputStreamSettings" << std::endl;
            return false;
        }

        egl_stream_settings_i->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
        egl_stream_settings_i->setResolution(
            Size2D<uint32_t>(config_.image_width, config_.image_height));

        streams_[i].reset(capture_session_i->createOutputStream(stream_settings.get()));

        /* Create capture request and enable the output stream */
        requests_[i].reset(capture_session_i->createRequest());
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

        iSourceSettings->setSensorMode(sensor_modes[i][config_.sensor_mode]);

        iSourceSettings->setFrameDurationRange(
            Range<uint64_t>(1e9 / config_.framerate));

        consumers_[i].reset(FrameConsumer::create(streams_[i].get()));
    }

    return true;
}

bool MultiCamera::init() {
    provider_ = UniqueObj<CameraProvider>(CameraProvider::create());
    ICameraProvider *provider_i = interface_cast<ICameraProvider>(provider_);
    if (!provider_i) {
        std::cerr << "Failed to get Camera Provider interface" << std::endl;
    }

    std::vector<CameraDevice *> cameras;
    if (!get_requested_cameras_(config_.device_ids, provider_i, cameras)) return false;

    if (!setup_cameras_(provider_i, cameras)) return false;

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
}

void MultiCamera::processing_thread_execution() {
    while (running_) {
        NvBufSurface **buf_surfaces;
        if (!raw_buffer_.pop(buf_surfaces)) continue;

        std::vector<cv::Mat> processed_frames;
        for (int i = 0; i < streams_.size(); i++) {
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

            processed_frames.push_back(display_img);

            // Unmap the buffer
            NvBufSurfaceUnMap(buf_surfaces[i], -1, 0);
        }

        processed_buffer_.push(processed_frames);
    }
}
