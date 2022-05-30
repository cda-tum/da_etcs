#include "astar/train.hpp"


#include <algorithm>

#include "astar/utils.hpp"

// std::ostream operator<<(std::ostream& os, const Move& move) {

//   os << "start: " << move.start.edge << "[" << move.start.from << ", "
//      << move.start.to << "]" << std::endl;
//   os << "passed through: ";
//   for(const Edge& edge: move.passed_edges) {
//     os << edge << " ";
//   }
//   os << std::endl << "stop: " << move.end.edge << "[" << move.end.from << ", "
//      << move.end.to << "]" << std::endl;
// }

Stop::Stop(const Edge& edge, int32_t to, uint32_t arrival_time): pos({edge, to}), arrival_time(arrival_time), departure_time(arrival_time) {}


Train::Train(uint32_t speed, uint32_t length): speed(speed), length(length), stops() {}

const Train& Train::add_stop(const Stop& stop) {
  stops.push_back(stop);
  return *this;
}

