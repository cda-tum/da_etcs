#include "astar/railway.hpp"
#include "astar/utils.hpp"
#include <algorithm>
#include <math.h>
#include <memory>
#include <queue>
#include <unordered_set>

Railway::Railway(const Graph &graph, std::vector<Train> &trains)
    : graph(graph), trains(trains) {}

std::ostream &operator<<(std::ostream &os, const Railway::Route &route) {
  for (size_t time = 0; time < route.size(); time++) {
    os << "Time " << time << ": ";
    os << route[time] << std::endl;
  }
  return os;
}

struct TrainState {
  TrainPosition pos;
  std::shared_ptr<TrainState> prev_state; // needed to assemble route at the end
  uint32_t time;

  TrainState(const TrainPosition &pos, uint32_t time) : pos(pos), time(time) {}
  TrainState(const TrainPosition &pos,
             const std::shared_ptr<TrainState> &prev_state, uint32_t time)
      : pos(pos), prev_state(prev_state), time(time) {}
  TrainState(std::shared_ptr<TrainState> other)
      : pos(other->pos), prev_state(other), time(other->time) {}

  TrainPosition train_pos(const Graph &graph, uint32_t train_length) const {
    TrainPosition train_pos = graph.sub_chain_from_back(pos, train_length);
    uint32_t chain_length = graph.chain_length(train_pos);
    auto state = prev_state;
    while (train_length > chain_length && state != nullptr) {
      TrainPosition to_join =
          graph.sub_chain_from_back(state->pos, train_length);
      chain_length += graph.chain_length(to_join);
      train_pos = combine_chains(to_join, train_pos);
      state = state->prev_state;
    }
    return train_pos;
  }
  const EdgePosition &engine_pos() const { return pos.end; }

  inline uint32_t hash() const { return pos.hash(); }
  bool operator==(const TrainState &other) const { return pos == other.pos; }
  bool operator!=(const TrainState &other) const { return !((*this) == other); }
};

struct State {
  std::vector<std::shared_ptr<TrainState>> train_states;
  uint32_t
      local_moves; // if local_moves == trains.size() then state is a full state
  size_t hash;     // cache hash value
  std::unordered_map<Edge, std::pair<int8_t, uint32_t>> reserved_ttd;
  Railway::VssMap vsss;
  inline const TrainState &operator[](size_t train) const {
    return *train_states[train];
  }

  State(size_t num_trains) : train_states(num_trains), local_moves(0) {}

  State(const std::vector<std::shared_ptr<TrainState>> &train_states,
        uint32_t local_moves)
      : train_states(train_states), local_moves(local_moves) {
    compute_hash();
  }

  State(const State &other, const std::shared_ptr<TrainState> &new_train_state,
        size_t train_index)
      : train_states(other.train_states), local_moves(other.local_moves + 1),
        hash(0), reserved_ttd(other.reserved_ttd),
        vsss(other.vsss) { // TODO: consider changing signs
    train_states[train_index] = new_train_state;
    compute_hash();
    int8_t dir = new_train_state->pos.start.direction();
    reserved_ttd[new_train_state->pos.start.edge] = {dir, train_index};
    reserved_ttd[new_train_state->pos.end.edge] = {dir, train_index};
    for (const Edge &e : new_train_state->pos.covered_edges)
      reserved_ttd[e] = {dir, train_index};
  }

  State(const State &other, const std::shared_ptr<TrainState> &new_train_state,
        size_t train_index, const EdgePosition &vss)
      : train_states(other.train_states), local_moves(other.local_moves + 1),
        hash(0), reserved_ttd(other.reserved_ttd),
        vsss(other.vsss) { // TODO: consider changing signs
    train_states[train_index] = new_train_state;
    compute_hash();
    int8_t dir = new_train_state->pos.start.direction();
    reserved_ttd[new_train_state->pos.start.edge] = {dir, train_index};
    reserved_ttd[new_train_state->pos.end.edge] = {dir, train_index};
    for (const Edge &e : new_train_state->pos.covered_edges)
      reserved_ttd[e] = {dir, train_index};

    vsss[vss.edge].insert(vss.pos);
  }

  /* construct state from full state */
  State(const State &other)
      : train_states(other.train_states), local_moves(0), hash(other.hash),
        vsss(other.vsss) {}

  inline bool is_full_state() const {
    return local_moves == train_states.size();
  }

  int32_t reserved(const Edge &ttd) const {
    if (reserved_ttd.find(ttd) != reserved_ttd.end())
      return reserved_ttd.at(ttd).first;
    return 0;
  }

  bool operator==(const State &other) const {
    for (size_t train_index = 0; train_index < train_states.size();
         train_index++) {
      if ((*this)[train_index] != other[train_index])
        return false;
    }
    return true;
  }

private:
  void compute_hash() {
    uint32_t hashes[train_states.size()];
    for (size_t train_index = 0; train_index < train_states.size();
         train_index++) {
      hashes[train_index] = train_states[train_index]->pos.hash();
    }
    hash = jenkins_hash(hashes, train_states.size());
  }
};

struct StateHasher {
  size_t operator()(const std::shared_ptr<State> s) const { return s->hash; }
};

struct StateEquiv {
  bool operator()(const std::shared_ptr<State> s1,
                  const std::shared_ptr<State> s2) const {
    return (*s1) == (*s2);
  }
};

/* utility class for computing scores for A* search */
class Score {
private:
  using OptimalLengthMap = std::vector<std::vector<uint32_t>>;
  const std::vector<Train> &trains;
  const Graph &graph;
  OptimalLengthMap optimal_lengths;

public:
  Score(const std::vector<Train> &trains, const Graph &graph)
      : trains(trains), graph(graph),
        optimal_lengths(trains.size(),
                        std::vector<uint32_t>(graph.num_edges())) {
    /*###############################################################################
     */
    /* perform initial search from goal backward to determine length of optimal
     * route */
    /*###############################################################################
     */

    using BFSEntry = std::pair<uint32_t, Edge>;
    for (size_t train_index = 0; train_index < trains.size(); train_index++) {
      const Train &train = trains[train_index];

      std::unordered_set<Edge> closed_set;
      std::priority_queue<BFSEntry, std::vector<BFSEntry>,
                          std::greater<BFSEntry>>
          bfs_fringe;

      BFSEntry init = {0, train.end_position().edge};
      bfs_fringe.push(init);
      while (!bfs_fringe.empty()) {
        auto [curr_length, curr_edge] = bfs_fringe.top();
        bfs_fringe.pop();

        if (closed_set.find(curr_edge) !=
            closed_set.end()) // edge has been visited already
          continue;

        closed_set.insert(curr_edge);
        optimal_lengths[train_index][curr_edge] = curr_length;

        for (const Edge &adj : graph.adjacent_edges(curr_edge)) {
          bfs_fringe.push({curr_length + graph[adj].length, adj});
        }
      }
    }
    int train = 0;
    // for (auto edge_lengths : optimal_lengths) {
    //   std::cout << "TRAIN" << train++ << std::endl;
    //   for (size_t edge = 0; edge < edge_lengths.size(); edge++) {
    //     std::cout << "Edge " << edge << " length " << edge_lengths[edge]
    //               << std::endl;
    //   }
    // }
  }

  /* cost function for A* search */
  float g(const State &state) const {
    float cost = 0;
    for (auto train_state : state.train_states)
      cost += train_state->time;
    return cost;
  }

  uint32_t num_boundaries(const State &state) const {
    uint32_t cnt = 0;
    for (auto [_, partitions] : state.vsss)
      cnt += partitions.size();
    return cnt;
  }

  /* heuristic for A* search */
  float
  h(const State &state) const { // TODO: improve heuristic by single agent A*
    float estimate = 0;
    for (size_t train_index = 0; train_index < trains.size(); train_index++) {
      const EdgePosition &engine_pos = state[train_index].pos.end;
      uint32_t length_in_curr_edge = engine_pos.length();

      estimate += (optimal_lengths[train_index][engine_pos.edge] != 0) *
                  (optimal_lengths[train_index][engine_pos.edge] -
                   length_in_curr_edge) /
                  ((float)trains[train_index].speed);
    }
    return estimate;
  }

  float h(const State &state, uint32_t used_vss) const {
    float estimate = 0;
    for (size_t train_index = 0; train_index < trains.size(); train_index++) {
      const EdgePosition &engine_pos = state[train_index].pos.end;
      uint32_t length_in_curr_edge = engine_pos.length();
      float res = (optimal_lengths[train_index][engine_pos.edge] != 0) *
                  (optimal_lengths[train_index][engine_pos.edge] -
                   length_in_curr_edge) /
                  ((float)trains[train_index].speed);
      if (train_index == used_vss)
        res = std::ceil(res);
      estimate += res;
    }
    return estimate;
  }

  inline float operator()(const State &state) const {
    return g(state) + h(state);
  }

  inline float operator()(const State &state, uint32_t used_vss) const {
    return g(state) + std::ceil(h(state));
  }
};

Railway::Schedule Railway::optimal_schedule() const {
    Score score(trains, graph);
  /*###############################################################################
   */
  /*                               perform A* search */
  /*###############################################################################
   */
  using SearchPair = std::pair<float, std::shared_ptr<State>> ;
  std::unordered_set<std::shared_ptr<State>, StateHasher, StateEquiv>
      closed_set;
  std::priority_queue<SearchPair, std::vector<SearchPair>,
                      std::greater<SearchPair>>
      fringe;

  /* initialize search */
  /* every train starts at its first stop*/
  std::shared_ptr<State> init_state = std::make_shared<State>(trains.size());
  for (size_t train_index = 0; train_index < trains.size(); train_index++) {
    const Train &train = trains[train_index];
    auto train_state = std::make_shared<TrainState>(train.start_position(),
                                                    train.start_time());
    init_state->train_states[train_index] = train_state;
  }

  fringe.push({score(*init_state), init_state});

  uint32_t expanded = 0; // TODO: for debugging
  std::cout << "start search"
            << "\n";

  while (!fringe.empty()) {
    auto [curr_score, curr_state_ptr] = fringe.top();
    const State &curr_state = *curr_state_ptr;
    fringe.pop();
    expanded++;

    
    /* check if goal state reached */
    uint32_t reached_goal = 0;
    std::vector<bool> in_goal(trains.size(), false);
    for (size_t train_index = 0; train_index < trains.size(); train_index++) {
      Chain goal_pos =
          Chain(trains[train_index].end_position()); // TODO: goal as interval
     
      if (graph.collision(goal_pos, curr_state[train_index].pos)) {
        in_goal[train_index] = true;
        reached_goal++;
      }
    }

    if (reached_goal == trains.size()) {

      std::vector<Route> routes(trains.size());
      for (size_t train_index = 0; train_index < trains.size(); train_index++) {
        std::shared_ptr<TrainState> train_state =
            curr_state.train_states[train_index];
        for (auto train_state = curr_state.train_states[train_index];
             train_state != nullptr; train_state = train_state->prev_state) {
          routes[train_index].push_back(train_state->pos);
        }
        std::reverse(routes[train_index].begin(), routes[train_index].end());
      }
      std::vector<Railway::VssMap> ass;
      std::cout << "expanded " << expanded << " nodes" << std::endl;
      return {curr_state.vsss, routes};
    }

    /* Operator decomposition: successively expand local state until every train
     * has made move */
    if (curr_state.is_full_state()) {

      if (closed_set.find(curr_state_ptr) != closed_set.end()) {
        continue;
      }

      closed_set.insert(curr_state_ptr);
      std::shared_ptr<State> new_state_ptr =
          std::make_shared<State>(curr_state);
      fringe.push({score(*new_state_ptr), new_state_ptr});
      continue;
    }
    /* advance next train locally */

    const Train &train = trains[curr_state.local_moves];
    auto curr_train_state = curr_state.train_states[curr_state.local_moves];

    if (in_goal[curr_state.local_moves]) {
      
      TrainPosition goal_pos =
          graph.sub_chain_from_back(curr_train_state->pos, train.length);

      bool collision = false; // TODO: duplication
      for (size_t i = 0; i < curr_state.local_moves; i++) {
        Chain other_train_pos =
            curr_state[i].train_pos(graph, trains[i].length);
        if (graph.collision(goal_pos, curr_state[i].pos) ||
            graph.collision(goal_pos, other_train_pos)) {
          collision = true;
          break;
        }
      }

      if (collision)
        continue;

      auto new_train_state = std::make_shared<TrainState>(curr_train_state);
      auto new_state = std::make_shared<State>(curr_state, new_train_state,
                                               curr_state.local_moves);
      fringe.push({score(*new_state), new_state});
      continue;
    }

    Graph::SubGraph reachable_graph = graph.sub_graph(
        curr_train_state->engine_pos(), train.speed, curr_state.vsss);

    for (auto it = reachable_graph.begin(); it != reachable_graph.end(); ++it) {
      Chain &path = *it;

      int32_t dir = path.end.direction(); // TODO: enough to check end?
      if (curr_state.reserved(path.end.edge) == -dir) 
        continue;
      
      /* check for collisions */
      bool collision = false;
      bool generate_vss = false;
      for (size_t i = 0; i < curr_state.local_moves; i++) {
        Chain other_train_pos =
            curr_state[i].train_pos(graph, trains[i].length);

        if (graph.collision(path, curr_state[i].pos) ||
            graph.collision(path, other_train_pos)) {
          if (curr_state[i].pos.end.direction() == path.end.direction()) {
            if (!graph.covers(path, curr_state[i].pos.start)) {
              collision = true;
              break;
            }

            // std::cout << "collision between " << path << " and "
            //           << curr_state[i].pos << " at time "
            //           << curr_train_state->time << std::endl;

            // std::cout << "collision between " << path << " and "
            //           << other_train_pos << " at time "
            //           << curr_train_state->time << std::endl
            //           << std::endl;
            // path = graph.truncate(path, curr_state[i].pos.start);
            generate_vss = true;
          } else
            collision = true;
          //          std::cout << "collision between " << path << " and " <<
          //          curr_state[i].pos << std::endl;
          break;
        }
      }
      if (collision)
        continue;

      // std::cout << "no collision" << "\n";
      // std::cout << curr_train_state->pos << "\n";

      auto new_train_state = std::make_shared<TrainState>(
          path, curr_train_state, curr_train_state->time + 1);
      std::shared_ptr<State> new_state;
      if (generate_vss) {

        EdgePosition vss = {path.end.edge,
                            graph.pos_from_left(path.end.edge, path.end.pos)};
        auto new_state = std::make_shared<State>(curr_state, new_train_state,
                                                 curr_state.local_moves, vss);
        fringe.push({score(*new_state, curr_state.local_moves), new_state});
      } else {
        auto new_state = std::make_shared<State>(curr_state, new_train_state,
                                                 curr_state.local_moves);
        fringe.push({score(*new_state), new_state});
      }
    }
  }

  std::cout << "expanded " << expanded << std::endl;
  Schedule schedule;
  return schedule;
}
