#pragma once
/*
 * Minimal lock-free SPSC (single-producer / single-consumer) ring buffer.
 * Safe for exactly ONE writer thread and ONE reader thread with no mutex —
 * this is what lets SensorAcquisitionThread push and DSPThread pop without
 * blocking each other. Do NOT use this if a buffer ever needs >1 writer or
 * >1 reader; you'd need a mutex-protected version instead.
 *
 * push() drops the newest sample when full rather than modifying the
 * consumer-owned tail index. This preserves SPSC ownership and makes every
 * overrun observable through get_drops().
 */
 
#include <cstdint>
#include <cstddef>
#include <cstring>
 
template <typename T, size_t CAPACITY>
class SpscRingBuffer {
public:
    SpscRingBuffer() : head_(0), tail_(0), drops_(0) {}
 
    // Producer side (called only from the owning writer thread)
    bool push(const T& item) {
        size_t next = (head_ + 1) % CAPACITY;
        if (next == tail_) {
            drops_++;
            return false; // do not modify the consumer-owned tail_
        }
        buffer_[head_] = item;
        head_ = next;
        return true;
    }
 
    // Consumer side (called only from the owning reader thread)
    bool pop(T& out) {
        if (tail_ == head_) return false;      // empty
        out = buffer_[tail_];
        tail_ = (tail_ + 1) % CAPACITY;
        return true;
    }
 
    size_t available() const {
        return (head_ >= tail_) ? (head_ - tail_) : (CAPACITY - tail_ + head_);
    }
 
    // Copies out `count` most-recent-window samples without consuming them —
    // used by FreezeIndexThread for overlapping FFT windows.
    void peek_window(T* out, size_t count) {
        size_t start = (head_ >= count) ? (head_ - count) : (CAPACITY + head_ - count);
        for (size_t i = 0; i < count; i++) {
            out[i] = buffer_[(start + i) % CAPACITY];
        }
    }
 
    // Advances the read pointer by `count` without copying (used after
    // peek_window to implement the sliding hop).
    void advance(size_t count) {
        tail_ = (tail_ + count) % CAPACITY;
    }
 
    uint32_t get_drops() const {
        return drops_;
    }
 
private:
    T buffer_[CAPACITY];
    volatile size_t head_;
    volatile size_t tail_;
    volatile uint32_t drops_;
};
 