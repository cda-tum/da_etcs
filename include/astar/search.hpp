#ifndef DA_ETCS_INCLUDE_SEARCH_HPP_
#define DA_ETCS_INCLUDE_SEARCH_HPP_

#include <vector>
#include <unordered_map>

#include "astar/graph.hpp"
#include "astar/train.hpp"

typedef Chain Move;
typedef Chain TrainPosition;

class Search
{
public:
  typedef std::vector<Move> Route;
  typedef std::unordered_map<Edge, std::set<uint32_t>> VssMap;
  typedef std::pair<VssMap, std::vector<Route>> ETCSSolution;

  //! Default constructor
  //  Search();

  Search(const Graph& g, const std::vector<Train>& trains, uint32_t max_timesteps): graph(g), trains(trains), max_timesteps(max_timesteps) {};
  Search(const Graph& g, const std::vector<Train>& trains, uint32_t max_timesteps, double timeout): graph(g), trains(trains), max_timesteps(max_timesteps), timeout(timeout) {};

  // //! Copy constructor
  // Search(const Search &other);

  // //! Move constructor
  // Search(Search &&other) noexcept;

  // //! Destructor
  // virtual ~Search() noexcept;

  // //! Copy assignment operator
  // Search& operator=(const Search &other);

  // //! Move assignment operator
  //  Search& operator=(Search &&other) noexcept;

  ETCSSolution do_search();
    void print_statistics() const;
protected:
private:
  Graph graph;
  std::vector<Train> trains;
  uint32_t max_timesteps;
  
  double timeout = 1000.0*60.0*10.0; // default 10 minute timeout

  /* Search statistics */
  uint32_t expanded = 0;
  uint32_t duplicate_states = 0; 
  uint32_t ttd_collisions = 0;
  uint32_t train_collisions = 0;
  uint32_t corrected_positions = 0;
  double solve_time = 0.0;
  uint32_t num_vss = 0;
  uint32_t make_span = 0;
  uint32_t sum_times = 0;
  bool solved = false;
};

#endif /* DA_ETCS_INCLUDE_SEARCH_HPP_ */
