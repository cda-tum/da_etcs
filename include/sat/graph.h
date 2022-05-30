#ifndef GRAPH_H
#define GRAPH_H
#include "./edge.h"
#include <utility>
#include <vector>
#include <unordered_set>
#include <unordered_map>

typedef std::vector<std::vector<Edge*>> AdjacencyList;
typedef std::unordered_set<const Edge*> EdgeSet;
typedef std::unordered_map<size_t, EdgeSet> ttdMap;
typedef std::vector<const Edge*> pathType;

struct EdgeSpeedKey {
  int speed;
  const Edge *edge;
  EdgeSpeedKey(int speed, const Edge *edge) : speed(speed), edge(edge){};
  
  bool operator==(const EdgeSpeedKey& other) const {
    return speed == other.speed && (*edge)==(*other.edge);
  }
};

struct EdgeSpeedHasher{
  size_t operator()(const EdgeSpeedKey& key) const {
    return (51+(key.edge->hash()))*51 + std::hash<int>()(key.speed); 
  }
};


typedef std::unordered_map<EdgeSpeedKey, EdgeSet, EdgeSpeedHasher> reachableCacheType;
typedef std::unordered_map<int, std::vector<std::vector<const Edge*>>> chainCacheType;

class Graph {
 private:
  AdjacencyList adjList;
  std::vector<bool> isBoundaryList;
  const Edge& addEdge(int from, int to, Edge* edge);
  reachableCacheType reachableCache;
  chainCacheType chainCache;

public:
  int vertexCount;
  ttdMap ttds;
  std::vector<Edge*> edges;
  
  Graph(int vertexCount);
  Graph();
  
  //returns null if there is no edge from "from" to "to"
  const Edge* getEdge(int from, int to);
  
  void setBoundaries();
  
  const Edge& addEdge(int from, int to, int length, int maxSpeed, int ttd);

  void addNode(bool isBoundary);

  void addNode();
  
  void setBoundary(int node);

  bool isBoundary(int node) const;

  void prohibitTransition(int left, int mid, int right);

  EdgeSet reachableEdges(int startFrom, int startTo, int speed);
  EdgeSet reachableEdges(const Edge* start, int speed);

  //needed for movement constraint
  std::vector<std::vector<const Edge*>> chainsOfLength(int length);

  //needed for train paths
  std::vector<std::vector<const Edge*>> paths(const Edge* start, const Edge* stop);

  //compute furthest reachable index in path when starting from startindex with given speed
  size_t reachableInPath(const pathType& path, size_t startIndex, int speed);

  //get all edges in ttd ordered from one end to the other (direction not deterministic)
  std::vector<const Edge*> getTTD(int ttd);
};

#endif
