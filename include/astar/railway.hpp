#ifndef RAILWAY_HPP
#define RAILWAY_HPP

#include "astar/train.hpp"
#include "astar/graph.hpp"
#include <iostream>
#include <functional>
#include <unordered_map>
#include <set>

typedef Chain Move;
typedef Chain TrainPosition;
class Railway {
public:
  /* typedefs */
  typedef std::vector<Move> Route;
  typedef std::unordered_map<Edge, std::set<uint32_t>> VssMap;
  typedef std::pair<VssMap, std::vector<Route>> Schedule;
  
  Railway(const Graph& graph, std::vector<Train>& trains);

  void add_train(const Train& train);

  /* heuristically compute optimal schedule */
  Schedule optimal_schedule() const;
  
private:
  Graph graph;
  std::vector<Train> trains;

  std::vector<TrainPosition> covered_positions(const Move& move, const Train& train) const;
  
  friend std::ostream& operator<<(std::ostream& os, const Route& Route);
};

std::ostream& operator<<(std::ostream& os, const Railway::Route& route);
#endif
