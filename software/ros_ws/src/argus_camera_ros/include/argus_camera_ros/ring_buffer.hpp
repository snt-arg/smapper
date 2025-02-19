#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <vector>

/**
 * @brief A thread-safe ring (circular) buffer.
 *
 * This template class implements a fixed-capacity circular buffer that supports
 * concurrent access. It allows pushing items into the buffer and popping items
 * out. When the buffer is full, the push operation can either overwrite the
 * oldest item or fail based on the override flag.
 *
 * @tparam T The type of elements stored in the ring buffer.
 */
template <typename T>
class RingBuffer {
   public:
    /**
     * @brief Constructs a new RingBuffer object.
     *
     * @param capacity Internal ring buffer capacity.
     */
    RingBuffer(size_t capacity = 8)
        : buffer_(capacity), capacity_(capacity), head_(0), tail_(0), size_(0) {}

    /**
     * @brief Pushes an item into the ring buffer.
     *
     * Inserts the given item at the current write position (head) of the buffer.
     * If the buffer is full, the behavior depends on the override flag:
     * - If override is true, the oldest item in the buffer is discarded (by
     * advancing the tail) to make room for the new item.
     * - If override is false, the function returns false immediately without
     * inserting the item.
     *
     * The operation is thread-safe; a mutex guards against concurrent
     * modifications. After successfully pushing the item, one waiting thread is
     * notified.
     *
     * @param item The item to be pushed into the buffer.
     * @param override Flag to indicate whether to override the oldest item if the
     * buffer is full. When true, the push fails if the buffer is full. When
     * false, the oldest item is overwritten.
     * @return true if the item was successfully pushed into the buffer; false if
     * the buffer was full and override is true.
     */
    bool push(const T &item, bool override = false) {
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

    /**
     * @brief Pops (removes) an item from the ring buffer.
     *
     * Removes and returns the oldest item in the buffer. If the buffer is empty,
     * this function blocks until an item is available or until the buffer is
     * stopped.
     *
     * Thread safety is maintained using a mutex and condition variable. The
     * condition variable waits until either there is an item to pop or the buffer
     * has been signaled to stop.
     *
     * @param item Reference to store the popped item.
     * @return true if an item was successfully popped; false if the buffer was
     * stopped before an item could be retrieved.
     */
    bool pop(T &item) {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [&] { return size_ > 0 || !running_; });

        if (!running_) return false;  // Shutdown signal

        item = buffer_[tail_];
        tail_ = (tail_ + 1) % capacity_;
        --size_;
        return true;
    }

    /**
     * @brief Stops the ring buffer.
     *
     * Signals that the ring buffer is shutting down. This method sets a flag
     * indicating that no further operations should block, and notifies all
     * waiting threads so they can exit.
     */
    void stop() {
        running_ = false;
        cond_.notify_all();
    }

    int size() { return size_; }

   private:
    std::vector<T> buffer_;     ///< Internal storage for buffer elements.
    size_t capacity_;           ///< Maximum number of elements the buffer can hold.
    size_t head_;               ///< Index for the next write (push) operation.
    size_t tail_;               ///< Index for the next read (pop) operation.
    std::atomic<size_t> size_;  ///< Current number of elements in the buffer.
    std::mutex mutex_;          ///< Mutex to protect concurrent access.
    std::condition_variable
        cond_;  ///< Condition variable for coordinating waiting and notification.
    std::atomic<bool> running_{
        true};  ///< Flag indicating whether the buffer is active.
};

#endif  // !RING_BUFFER_HPP
