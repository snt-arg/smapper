#include "argus_camera_ros/ring_buffer.hpp"

template <typename T, size_t capacity>
RingBuffer<T, capacity>::RingBuffer()
    : buffer_(capacity), capacity_(capacity), head_(0), tail_(0), size_(0) {}

template <typename T, size_t capacity>
bool RingBuffer<T, capacity>::push(const T &item, bool override) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (size_ >= capacity_) {
        if (override) return false;  // Buffer full
        tail_ = (tail_ + 1) & capacity_;
        --size_;
    }

    buffer_[head_] = item;
    head_ = (head_ + 1) % capacity_;
    ++size_;

    lock.unlock();
    cond_.notify_one();
    return true;
}

template <typename T, size_t capacity>
bool RingBuffer<T, capacity>::pop(T &item) {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait(lock, [&] { return size_ > 0 || !running_; });

    if (!running_) return false;  // Shutdown signal

    item = buffer_[tail_];
    tail_ = (tail_ + 1) % capacity_;
    --size_;
    return true;
}

template <typename T, size_t capacity>
void RingBuffer<T, capacity>::stop() {
    running_ = false;
    cond_.notify_all();
}
