#ifndef TRAIN_HPP
#define TRAIN_HPP

#include <stdint.h>
#include <vector>
#include "astar/graph.hpp"


struct Stop {
  EdgePosition pos;
  uint32_t arrival_time, departure_time;
  
  Stop(const Edge& track, int32_t to, uint32_t arrival_time);
};

class Train {
public:
  uint32_t speed, length;

  Train(uint32_t speed, uint32_t length);
  const Train& add_stop(const Stop& stop);
  
  inline const EdgePosition& start_position() const {return stops[0].pos;}
  inline const EdgePosition& end_position() const {return stops.back().pos;}
  inline uint32_t start_time() const {return stops[0].departure_time;}
private:
  std::vector<Stop> stops;
};


#endif
