#include "../include/edge.h"
#include <sstream>

bool Edge::operator<(const Edge &e2) const { return length <= e2.length; }

int Edge::getOtherVertex(int vertex) const {
  return vertex == from ? to : from;
}

int Edge::operator[](int vertex) const { return getOtherVertex(vertex); }

bool Edge::connectsTo(int vertex) const {
  return vertex == to || vertex == from;
}

std::string Edge::toString() const {
  std::stringstream str;
  str << "from " << from << " to " << to;
  return str.str();
}

Edge *Edge::addNeighbour(Edge *neighbour) {
  neighbours.push_back(neighbour);
  return neighbours.back();
}

int Edge::sharedVertex(const Edge *otherEdge) const {
  if (otherEdge->connectsTo(from))
    return from;
  if (otherEdge->connectsTo(to))
    return to;
  return -1;
}

Edge::Edge(int id, int from, int to, int length, int ttd, int maxSpeed)
    : id(id), from(from), to(to), length(length), ttd(ttd), maxSpeed(maxSpeed),
      neighbours() {}

bool Edge::operator==(const Edge &e) const {
  return from == e.from && to == e.to;
}

size_t Edge::hash() const {
  return (51 + std::hash<int>()(from)) * 51 + std::hash<int>()(to);
}
