#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <limits>
#include <numeric>
#include <type_traits>
#include <vector>


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
