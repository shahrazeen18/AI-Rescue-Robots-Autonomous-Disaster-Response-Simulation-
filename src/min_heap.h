// min_heap.h - hand-written binary min-heap (priority queue, no STL).
// Less(a, b) must be a strict weak ordering; top() is the element that is
// "least" under it. push/pop are O(log n), top is O(1).
// Used for Dijkstra's frontier and for the request dispatch order.
#ifndef RR_MIN_HEAP_H
#define RR_MIN_HEAP_H

#include "vector.h"

namespace rr {

template <typename T, typename Less>
class MinHeap {
 public:
  MinHeap() {}
  explicit MinHeap(const Less& less) : less_(less) {}

  bool empty() const { return a_.empty(); }
  int size() const { return a_.size(); }
  void clear() { a_.clear(); }

  const T& top() const { return a_[0]; }

  void push(const T& v) {
    a_.pushBack(v);
    int i = a_.size() - 1;
    while (i > 0) {
      int p = (i - 1) / 2;
      if (!less_(a_[i], a_[p])) break;
      swapAt(i, p);
      i = p;
    }
  }

  void pop() {
    int n = a_.size();
    a_[0] = a_[n - 1];
    a_.popBack();
    --n;
    int i = 0;
    for (;;) {
      int l = 2 * i + 1, r = l + 1, m = i;
      if (l < n && less_(a_[l], a_[m])) m = l;
      if (r < n && less_(a_[r], a_[m])) m = r;
      if (m == i) break;
      swapAt(i, m);
      i = m;
    }
  }

 private:
  void swapAt(int i, int j) {
    T t = a_[i];
    a_[i] = a_[j];
    a_[j] = t;
  }

  Vector<T> a_;
  Less less_;
};

}  // namespace rr

#endif
