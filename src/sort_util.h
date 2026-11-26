// sort_util.h - hand-written stable merge sort (no STL).
// O(n log n) time, O(n) extra space. Stability matters: records that share a
// time step must keep their order of appearance in the input file.
#ifndef RR_SORT_UTIL_H
#define RR_SORT_UTIL_H

#include "vector.h"

namespace rr {

// Sorts v ascending under strict ordering less(a, b); equal elements keep
// their original relative order.
template <typename T, typename Less>
void stableSort(Vector<T>& v, const Less& less) {
  int n = v.size();
  if (n < 2) return;
  Vector<T> tmp;
  tmp.resize(n);
  Vector<T>* src = &v;
  Vector<T>* dst = &tmp;
  for (int width = 1; width < n; width *= 2) {
    for (int lo = 0; lo < n; lo += 2 * width) {
      int mid = lo + width < n ? lo + width : n;
      int hi = lo + 2 * width < n ? lo + 2 * width : n;
      int i = lo, j = mid, k = lo;
      while (i < mid && j < hi) {
        // take from the right run only if strictly smaller -> stable
        if (less((*src)[j], (*src)[i])) {
          (*dst)[k++] = (*src)[j++];
        } else {
          (*dst)[k++] = (*src)[i++];
        }
      }
      while (i < mid) (*dst)[k++] = (*src)[i++];
      while (j < hi) (*dst)[k++] = (*src)[j++];
    }
    Vector<T>* t = src;
    src = dst;
    dst = t;
  }
  if (src != &v) {
    for (int i = 0; i < n; ++i) v[i] = (*src)[i];
  }
}

}  

#endif
