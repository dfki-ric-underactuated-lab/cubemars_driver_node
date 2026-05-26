#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <limits>
#include <numeric>
#include <type_traits>
#include <vector>


// Classic position-only alpha-beta filter. Assumes constant velocity between updates.
// Uses the position measurement to correct both the position and velocity estimates.
template <typename T>
class AlphaBetaFilter {
 private:
  T alpha_;
  T beta_;
  T x_ = T{};
  T v_ = T{};
  bool initialized_ = false;

 public:
  AlphaBetaFilter(T alpha = T{1}, T beta = T{}) : alpha_(alpha), beta_(beta) {}

  void update(T measurement, T dt) {
    if (!initialized_ || dt <= T{}) {
      x_ = measurement;
      v_ = T{};
      initialized_ = true;
      return;
    }
    T x_pred = x_ + v_ * dt;
    T residual = measurement - x_pred;
    x_ = x_pred + alpha_ * residual;
    v_ = v_ + (beta_ / dt) * residual;
  }

  T position() const { return x_; }
  T velocity() const { return v_; }

  T get_alpha() const { return alpha_; }
  T get_beta() const { return beta_; }
  void set_alpha(T alpha) { alpha_ = alpha; }
  void set_beta(T beta) { beta_ = beta; }
};


// Streaming median filter for outlier rejection. Size 0 or 1 = pass-through.
// Uses std::nth_element on a small stack-allocated scratch buffer — O(N) per update,
// no allocations after construction. Practical for window sizes up to ~20.
template <typename T>
class MedianFilter {
 private:
  std::vector<T> buffer_;
  std::vector<T> scratch_;
  unsigned int idx_ = 0;
  unsigned int filled_ = 0;
  unsigned int size_;

 public:
  MedianFilter(unsigned int size = 0) : size_(size) {
    buffer_.resize(size);
    scratch_.resize(size);
  }

  T update(T value) {
    if (size_ <= 1) {
      return value;
    }
    buffer_[idx_] = value;
    idx_ = (idx_ + 1) % size_;
    if (filled_ < size_) {
      filled_++;
    }
    std::copy_n(buffer_.begin(), filled_, scratch_.begin());
    const auto mid = scratch_.begin() + filled_ / 2;
    std::nth_element(scratch_.begin(), mid, scratch_.begin() + filled_);
    return *mid;
  }

  void resize(unsigned int size) {
    size_ = size;
    buffer_.assign(size, T{});
    scratch_.assign(size, T{});
    idx_ = 0;
    filled_ = 0;
  }
};


template <typename T>
class MovingAverage {
 private:
  unsigned int buffersize_;
  unsigned int index_ = 0;
  std::vector<T> buffer_;
  T val_;

 public:
  MovingAverage(unsigned int buffersize = 1) : buffersize_(buffersize), val_(T{}) { buffer_.reserve(buffersize); };

  T update(T value) {
    if (buffer_.size() == buffersize_) {
      val_ -= buffer_[index_];
      buffer_[index_++] = value;
      val_ += value;
      index_ %= buffersize_;
      return val_ / buffersize_;
    } else if (buffer_.size() > 0) {
      buffer_.push_back(value);
      val_ += value;
      return val_ / buffer_.size();
    } else {
      buffer_.push_back(value);
      val_ = value;  // If there is a vector in it addition wont work due to sizes, thats why extra case
      return val_;
    }
  }

  T get() const { return val_ / buffer_.size(); }

  void resize(unsigned int size) {
    if (size == buffersize_) {
      return;
    } else if (size == buffer_.size()) {
      buffersize_ = size;
      return;
    } else if (size > buffer_.size()) {
      buffer_.reserve(size);
      std::rotate(buffer_.begin(), buffer_.begin() + index_, buffer_.end());
      index_ = 0;
      buffersize_ = size;
    } else {
      std::rotate(buffer_.begin(), buffer_.begin() + ((index_ - size) % buffer_.size()), buffer_.end());
      buffer_.resize(size);
      index_ = 0;
      val_ = std::accumulate(buffer_.begin(), buffer_.end(), T{});
    }
  }
};
