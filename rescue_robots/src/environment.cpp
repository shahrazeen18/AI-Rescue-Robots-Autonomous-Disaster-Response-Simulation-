#include "environment.h"

#include <cstring>

#include "min_heap.h"

namespace rr {

namespace {

struct HeapItem {
  i64 d;
  int v;
  HeapItem() : d(0), v(0) {}
  HeapItem(i64 dd, int vv) : d(dd), v(vv) {}
};

struct HeapLess {
  bool operator()(const HeapItem& x, const HeapItem& y) const {
    if (x.d != y.d) return x.d < y.d;
    return x.v < y.v;
  }
};

}  // namespace

int Environment::addLocation(int id, const char* name, int type) {
  int idx = locs_.size();
  if (!locIndex_.insert(id, idx)) return -1;
  Location l;
  l.id = id;
  std::strncpy(l.name, name, NAME_LEN - 1);
  l.name[NAME_LEN - 1] = '\0';
  l.type = type;
  l.stationUp = true;
  locs_.pushBack(l);
  return idx;
}

int Environment::addConnection(int id, int a, int b, int cost, bool blocked) {
  int idx = conns_.size();
  if (!connIndex_.insert(id, idx)) return -1;
  Connection c;
  c.id = id;
  c.a = a;
  c.b = b;
  c.cost = cost;
  c.blocked = blocked;
  c.initiallyBlocked = blocked;
  conns_.pushBack(c);
  return idx;
}

void Environment::finalize() {
  int n = locs_.size();
  offsets_.assign(n + 1, 0);
  for (int e = 0; e < conns_.size(); ++e) {
    ++offsets_[conns_[e].a + 1];
    ++offsets_[conns_[e].b + 1];
  }
  for (int v = 0; v < n; ++v) offsets_[v + 1] += offsets_[v];
  adj_.assign(offsets_[n], 0);
  Vector<int> fill;
  fill.assign(n, 0);
  // Edges are inserted in file order so adjacency order is deterministic.
  for (int e = 0; e < conns_.size(); ++e) {
    int a = conns_[e].a, b = conns_[e].b;
    adj_[offsets_[a] + fill[a]++] = e;
    adj_[offsets_[b] + fill[b]++] = e;
  }
  stations_.clear();
  for (int v = 0; v < n; ++v) {
    if (locs_[v].type == LOC_CHARGING) stations_.pushBack(v);
  }
}

bool Environment::setBlocked(int e, bool blocked) {
  if (conns_[e].blocked == blocked) return false;
  conns_[e].blocked = blocked;
  ++version_;
  return true;
}

int Environment::countBlocked() const {
  int n = 0;
  for (int e = 0; e < conns_.size(); ++e) {
    if (conns_[e].blocked) ++n;
  }
  return n;
}

bool Environment::setStationUp(int loc, bool up) {
  if (locs_[loc].stationUp == up) return false;
  locs_[loc].stationUp = up;
  ++version_;
  return true;
}

void Environment::shortestPaths(int src, Vector<i64>& dist,
                                Vector<int>& prevEdge,
                                const Vector<char>* targetMask,
                                int targetCount, bool stopAtStation) const {
  int n = locs_.size();
  dist.assign(n, INF);
  prevEdge.assign(n, -1);
  MinHeap<HeapItem, HeapLess> heap;
  dist[src] = 0;
  heap.push(HeapItem(0, src));
  ++searches_;
  const bool early = targetMask != nullptr || stopAtStation;
  int targetsLeft = targetMask ? targetCount : 0;
  bool needStation = stopAtStation;
  while (!heap.empty()) {
    HeapItem cur = heap.top();
    heap.pop();
    if (cur.d > dist[cur.v]) continue;  // stale entry (lazy deletion)
    ++settled_;
    if (early) {
      // cur.v is settled now: its dist/prevEdge are final.
      if (targetMask && (*targetMask)[cur.v]) --targetsLeft;
      if (needStation && stationUp(cur.v)) needStation = false;
      if (targetsLeft <= 0 && !needStation) return;
    }
    for (int k = offsets_[cur.v]; k < offsets_[cur.v + 1]; ++k) {
      int e = adj_[k];
      ++scanned_;
      if (conns_[e].blocked) continue;
      int w = conns_[e].a == cur.v ? conns_[e].b : conns_[e].a;
      i64 nd = cur.d + conns_[e].cost;
      if (nd < dist[w]) {
        dist[w] = nd;
        prevEdge[w] = e;
        heap.push(HeapItem(nd, w));
      }
    }
  }
}

void Environment::distanceToNearestStation(Vector<i64>& dist) const {
  int n = locs_.size();
  dist.assign(n, INF);
  ++searches_;
  MinHeap<HeapItem, HeapLess> heap;
  for (int i = 0; i < stations_.size(); ++i) {
    int s = stations_[i];
    if (!locs_[s].stationUp) continue;
    dist[s] = 0;
    heap.push(HeapItem(0, s));
  }
  while (!heap.empty()) {
    HeapItem cur = heap.top();
    heap.pop();
    if (cur.d > dist[cur.v]) continue;
    ++settled_;
    for (int k = offsets_[cur.v]; k < offsets_[cur.v + 1]; ++k) {
      int e = adj_[k];
      ++scanned_;
      if (conns_[e].blocked) continue;
      int w = conns_[e].a == cur.v ? conns_[e].b : conns_[e].a;
      i64 nd = cur.d + conns_[e].cost;
      if (nd < dist[w]) {
        dist[w] = nd;
        heap.push(HeapItem(nd, w));
      }
    }
  }
}

void Environment::pathToRoot(const Vector<int>& prevEdge, int node,
                             Vector<int>& out) const {
  out.clear();
  int v = node;
  while (prevEdge[v] != -1) {
    int e = prevEdge[v];
    out.pushBack(e);
    v = otherEnd(e, v);
  }
}

void Environment::pathFromRoot(const Vector<int>& prevEdge, int node,
                               Vector<int>& out) const {
  pathToRoot(prevEdge, node, out);
  out.reverse();
}

int Environment::countComponents() const {
  int n = locs_.size();
  Vector<char> seen;
  seen.assign(n, 0);
  Vector<int> queue;
  int comps = 0;
  for (int s = 0; s < n; ++s) {
    if (seen[s]) continue;
    ++comps;
    queue.clear();
    queue.pushBack(s);
    seen[s] = 1;
    for (int head = 0; head < queue.size(); ++head) {
      int v = queue[head];
      for (int k = offsets_[v]; k < offsets_[v + 1]; ++k) {
        int w = otherEnd(adj_[k], v);
        if (!seen[w]) {
          seen[w] = 1;
          queue.pushBack(w);
        }
      }
    }
  }
  return comps;
}

void Environment::print(FILE* f) const {
  std::fprintf(f, "Environment: %d locations, %d connections (%d blocked), "
                  "%d charging stations\n",
               numLocations(), numConnections(), countBlocked(),
               stations_.size());
  std::fprintf(f, "  Blocked connections:");
  int shown = 0;
  for (int e = 0; e < conns_.size(); ++e) {
    if (!conns_[e].blocked) continue;
    std::fprintf(f, " #%d(%d-%d)", conns_[e].id, locs_[conns_[e].a].id,
                 locs_[conns_[e].b].id);
    ++shown;
  }
  if (shown == 0) std::fprintf(f, " none");
  std::fprintf(f, "\n  Charging stations:");
  for (int i = 0; i < stations_.size(); ++i) {
    const Location& l = locs_[stations_[i]];
    std::fprintf(f, " %d(%s)", l.id, l.stationUp ? "UP" : "DOWN");
  }
  if (stations_.empty()) std::fprintf(f, " none");
  std::fprintf(f, "\n");
}

}  // namespace rr
