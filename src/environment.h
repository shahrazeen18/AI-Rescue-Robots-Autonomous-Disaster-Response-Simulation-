// environment.h - the disaster region as an undirected weighted graph.
//
// Representation: adjacency list in compressed (CSR) form. Locations and
// connections live in dense arrays indexed 0..n-1 (internal indices); the
// external ids from the input file are translated with a hash map (IntMap).
// Blocking/unblocking a connection flips one flag (O(1)); the adjacency
// structure itself is never modified, so no rebuild is ever needed.
#ifndef RR_ENVIRONMENT_H
#define RR_ENVIRONMENT_H

#include <cstdio>

#include "int_map.h"
#include "types.h"
#include "vector.h"

namespace rr {

const int NAME_LEN = 40;

struct Location {
  int id;
  char name[NAME_LEN];
  int type;        // LocType
  bool stationUp;  // only meaningful for LOC_CHARGING
  Location() : id(0), type(LOC_JUNCTION), stationUp(true) { name[0] = '\0'; }
};

struct Connection {
  int id;
  int a, b;  // internal location indices
  int cost;  // travel cost units (>= 1)
  bool blocked;
  bool initiallyBlocked;
  Connection() : id(0), a(0), b(0), cost(1), blocked(false),
                 initiallyBlocked(false) {}
};

class Environment {
 public:
  static constexpr i64 INF = 4000000000000000000LL;

  // --- construction (used by the scenario loader) ---
  // Return the new internal index, or -1 if the id already exists.
  int addLocation(int id, const char* name, int type);
  int addConnection(int id, int a, int b, int cost, bool blocked);
  // Builds the adjacency structure. Must be called once after all additions.
  void finalize();

  // --- queries ---
  int numLocations() const { return locs_.size(); }
  int numConnections() const { return conns_.size(); }
  const Location& location(int i) const { return locs_[i]; }
  const Connection& connection(int e) const { return conns_[e]; }
  int locationIndex(int id) const { return locIndex_.get(id, -1); }
  int connectionIndex(int id) const { return connIndex_.get(id, -1); }

  int degree(int v) const { return offsets_[v + 1] - offsets_[v]; }
  int adjacentEdge(int v, int k) const { return adj_[offsets_[v] + k]; }
  int otherEnd(int e, int node) const {
    return conns_[e].a == node ? conns_[e].b : conns_[e].a;
  }

  bool isBlocked(int e) const { return conns_[e].blocked; }
  // Return true if the state actually changed.
  bool setBlocked(int e, bool blocked);
  int countBlocked() const;

  bool isStation(int loc) const { return locs_[loc].type == LOC_CHARGING; }
  bool stationUp(int loc) const {
    return locs_[loc].type == LOC_CHARGING && locs_[loc].stationUp;
  }
  bool setStationUp(int loc, bool up);
  const Vector<int>& stations() const { return stations_; }

  // Monotone counter that increases on every change that can alter routes.
  int version() const { return version_; }

  // Operation counters for performance analysis (deterministic, unlike time).
  i64 searchesRun() const { return searches_; }
  i64 nodesSettled() const { return settled_; }
  i64 edgesScanned() const { return scanned_; }

  // --- routing ---
  // Dijkstra from src over OPEN connections only. dist[v] is the minimum
  // total cost (INF if unreachable); prevEdge[v] is the connection used to
  // reach v on its shortest path (-1 for src/unreachable). Deterministic:
  // ties are resolved by heap order (cost, node index) and adjacency order.
  //
  // Early exit (optional). If targetMask is given, the search stops as soon as
  // every node with targetMask[v] != 0 has been settled (targetCount = number
  // of such nodes); if stopAtStation is true it also waits until the nearest
  // UP charging station has been settled. Nodes settled before the stop have
  // exact dist/prevEdge values (identical to a full run); other nodes may hold
  // tentative values that are >= their true distance. With neither option the
  // whole reachable region is explored.
  void shortestPaths(int src, Vector<i64>& dist, Vector<int>& prevEdge,
                     const Vector<char>* targetMask = nullptr,
                     int targetCount = 0, bool stopAtStation = false) const;

  // Multi-source Dijkstra: dist[v] = cost from v to the nearest UP charging
  // station over OPEN connections (INF if none reachable). One search answers
  // "how far is the nearest charger" for every location at once.
  void distanceToNearestStation(Vector<i64>& dist) const;

  // Edges leading from `node` towards the root of the tree (prevEdge was
  // computed from the root). Result is in travel order node -> root.
  void pathToRoot(const Vector<int>& prevEdge, int node,
                  Vector<int>& out) const;
  // Edges leading from the root to `node` in travel order root -> node.
  void pathFromRoot(const Vector<int>& prevEdge, int node,
                    Vector<int>& out) const;

  // Number of connected components, ignoring blocked flags (structure only).
  int countComponents() const;

  // --- reporting ---
  void print(FILE* f) const;

 private:
  Vector<Location> locs_;
  Vector<Connection> conns_;
  IntMap locIndex_;
  IntMap connIndex_;
  Vector<int> offsets_;  // CSR row starts, size n+1
  Vector<int> adj_;      // connection indices grouped by location
  Vector<int> stations_; // internal indices of charging stations
  int version_ = 0;
  mutable i64 searches_ = 0;
  mutable i64 settled_ = 0;
  mutable i64 scanned_ = 0;
};

}  // namespace rr

#endif
