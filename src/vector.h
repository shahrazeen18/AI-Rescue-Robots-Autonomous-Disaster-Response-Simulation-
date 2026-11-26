// vector.h - hand-written dynamic array (no STL).
// Amortised O(1) pushBack, O(1) indexed access. Rule-of-five implemented so
// copies are deep and memory is always released (RAII).
#ifndef RR_VECTOR_H
#define RR_VECTOR_H

#include <cassert>

namespace rr {

template <typename T>
class Vector {
 public:
  Vector() : data_(nullptr), size_(0), cap_(0) {}

  explicit Vector(int n) : data_(nullptr), size_(0), cap_(0) { resize(n); }

  Vector(const Vector& o) : data_(nullptr), size_(0), cap_(0) {
    reserve(o.size_);
    for (int i = 0; i < o.size_; ++i) data_[i] = o.data_[i];
    size_ = o.size_;
  }

  Vector(Vector&& o) noexcept : data_(o.data_), size_(o.size_), cap_(o.cap_) {
    o.data_ = nullptr;
    o.size_ = 0;
    o.cap_ = 0;
  }

  Vector& operator=(const Vector& o) {
    if (this != &o) {
      Vector tmp(o);
      swap(tmp);
    }
    return *this;
  }

  Vector& operator=(Vector&& o) noexcept {
    if (this != &o) {
      delete[] data_;
      data_ = o.data_;
      size_ = o.size_;
      cap_ = o.cap_;
      o.data_ = nullptr;
      o.size_ = 0;
      o.cap_ = 0;
    }
    return *this;
  }

  ~Vector() { delete[] data_; }

  void swap(Vector& o) noexcept {
    T* d = data_;
    data_ = o.data_;
    o.data_ = d;
    int s = size_;
    size_ = o.size_;
    o.size_ = s;
    int c = cap_;
    cap_ = o.cap_;
    o.cap_ = c;
  }

  int size() const { return size_; }
  bool empty() const { return size_ == 0; }

  T& operator[](int i) {
    assert(i >= 0 && i < size_);
    return data_[i];
  }
  const T& operator[](int i) const {
    assert(i >= 0 && i < size_);
    return data_[i];
  }
  T& back() {
    assert(size_ > 0);
    return data_[size_ - 1];
  }
  const T& back() const {
    assert(size_ > 0);
    return data_[size_ - 1];
  }

  void reserve(int n) {
    if (n <= cap_) return;
    T* nd = new T[n];
    for (int i = 0; i < size_; ++i) nd[i] = static_cast<T&&>(data_[i]);
    delete[] data_;
    data_ = nd;
    cap_ = n;
  }

  void pushBack(const T& v) {
    if (size_ == cap_) {
      T copy = v;  // v may alias an element of this vector
      reserve(cap_ == 0 ? 8 : cap_ * 2);
      data_[size_++] = static_cast<T&&>(copy);
    } else {
      data_[size_++] = v;
    }
  }

  void popBack() {
    assert(size_ > 0);
    --size_;
    data_[size_] = T();
  }

  void clear() {
    for (int i = 0; i < size_; ++i) data_[i] = T();
    size_ = 0;
  }

  // Grow (value-initialised elements) or shrink to exactly n elements.
  void resize(int n) {
    assert(n >= 0);
    if (n > cap_) reserve(n);
    for (int i = size_; i < n; ++i) data_[i] = T();
    for (int i = n; i < size_; ++i) data_[i] = T();
    size_ = n;
  }

  void assign(int n, const T& v) {
    clear();
    reserve(n);
    for (int i = 0; i < n; ++i) data_[i] = v;
    size_ = n;
  }

  // Reverse in place.
  void reverse() {
    for (int i = 0, j = size_ - 1; i < j; ++i, --j) {
      T t = static_cast<T&&>(data_[i]);
      data_[i] = static_cast<T&&>(data_[j]);
      data_[j] = static_cast<T&&>(t);
    }
  }

 private:
  T* data_;
  int size_;
  int cap_;
};

}  

#endif
