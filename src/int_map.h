// int_map.h - hand-written open-addressing hash table, int key -> int value.
// Used to translate external ids (arbitrary positive integers) into dense
// internal indices. Linear probing, power-of-two capacity, load factor <= 0.5.
// Expected O(1) insert / lookup. No deletion is needed by this project.
#ifndef RR_INT_MAP_H
#define RR_INT_MAP_H

#include "vector.h"

namespace rr {

class IntMap {
 public:
  IntMap() : count_(0) { slots_.resize(16); }

  int size() const { return count_; }

  // Inserts key->value. Returns false (and changes nothing) if key exists.
  bool insert(int key, int value) {
    if ((count_ + 1) * 2 > slots_.size()) grow();
    int i = probe(key);
    if (slots_[i].used) return false;
    slots_[i].used = true;
    slots_[i].key = key;
    slots_[i].value = value;
    ++count_;
    return true;
  }

  bool contains(int key) const { return slots_[probe(key)].used; }

  // Returns the value for key, or notFound if absent.
  int get(int key, int notFound = -1) const {
    const Slot& s = slots_[probe(key)];
    return s.used ? s.value : notFound;
  }

 private:
  struct Slot {
    int key;
    int value;
    bool used;
    Slot() : key(0), value(0), used(false) {}
  };

  static unsigned hashOf(int key) {
    unsigned x = static_cast<unsigned>(key);
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
  }

  // Index of the slot holding key, or of the first empty slot on its probe
  // sequence. Terminates because the table is never more than half full.
  int probe(int key) const {
    unsigned mask = static_cast<unsigned>(slots_.size()) - 1u;
    unsigned i = hashOf(key) & mask;
    while (slots_[static_cast<int>(i)].used &&
           slots_[static_cast<int>(i)].key != key) {
      i = (i + 1u) & mask;
    }
    return static_cast<int>(i);
  }

  void grow() {
    Vector<Slot> old(slots_);
    slots_.clear();
    slots_.resize(old.size() * 2);
    for (int i = 0; i < old.size(); ++i) {
      if (old[i].used) {
        int j = probe(old[i].key);
        slots_[j] = old[i];
      }
    }
  }

  Vector<Slot> slots_;
  int count_;
};

}  

#endif
