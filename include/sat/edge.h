#ifndef EDGE_H
#define EDGE_H

#include <algorithm>
#include <functional>
#include <vector>
#include <string>

class Edge {
public:
  const int id, from, to, length, ttd, maxSpeed;
  std::vector<Edge *> neighbours;

  Edge(int id, int from, int to, int length, int ttd, int maxSpeed);

  int getOtherVertex(int vertex) const;

  int operator[](int vertex) const;

  Edge *addNeighbour(Edge *neighbour);
  bool connectsTo(int vertex) const;
  int sharedVertex(const Edge *otherEdge) const;
  bool operator<(const Edge &e2) const;
  bool operator==(const Edge &e2) const;
  size_t hash() const;
  std::string toString() const;
};

#endif
