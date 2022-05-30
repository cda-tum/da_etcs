#include "astar/search.hpp"
#include "astar/utils.hpp"

#include <algorithm>
#include <ctime>
#include <limits>
#include <math.h>
#include <memory>
#include <queue>
#include <vector>

struct State {
  std::vector<TrainPosition> positions;
  std::vector<int32_t> time;
  std::shared_ptr<State> prev;
  Search::VssMap vss;
  uint32_t local_moves = 0;

  State(size_t num_trains)
      : positions(num_trains), time(num_trains, 0), prev(nullptr){};
  size_t hash() const {
    std::vector<uint32_t> pos_hashes(positions.size());
    for (size_t i = 0; i < positions.size(); i++)
      pos_hashes[i] = positions[i].hash();
    return jenkins_hash(pos_hashes.data(), positions.size());
  }

  bool operator==(const State &other) const {
    for (size_t i = 0; i < positions.size(); i++)
      if (other.positions[i] != positions[i])
        return false;
    return true;
  }
};

struct StateHasher {
  size_t operator()(const std::shared_ptr<State> s) const { return s->hash(); }
};

struct StateEquiv {
  bool operator()(const std::shared_ptr<State> s1,
                  const std::shared_ptr<State> s2) const {

    return (*s1) == (*s2);
  }
};

std::shared_ptr<State> advance_to_full_state(const State &state, const Graph &g,
                                             const std::vector<Train> trains) {
  std::shared_ptr<State> full_state =
      std::make_shared<State>(state.positions.size());

  for (size_t i = 0; i < state.positions.size(); i++) {
    full_state->positions[i] =
        g.sub_chain_from_back(state.positions[i], trains[i].length);
  }
  full_state->prev = state.prev;
  full_state->local_moves = 0;
  full_state->time = state.time;
  full_state->vss = state.vss;
  return full_state;
}

std::shared_ptr<State> advance_local_state(const std::shared_ptr<State> state,
                                           const Move &move) {
  std::shared_ptr<State> local_state =
      std::make_shared<State>(state->positions.size());
  for (size_t i = 0; i < state->positions.size(); i++) {
    if (i == state->local_moves)
      continue;
    local_state->positions[i] = state->positions[i];
    local_state->time[i] = state->time[i];
  }

  local_state->positions[state->local_moves] = move;
  local_state->local_moves = state->local_moves + 1;
  local_state->time[state->local_moves] = state->time[state->local_moves] + 1;
  local_state->vss = state->vss;

  if (state->local_moves == 0) // previous state is full state
    local_state->prev = state;
  else
    local_state->prev = state->prev;
  return local_state;
}

enum class CollisionType { none, ttd_front, ttd_back, train };
struct CollisionInfo {
  CollisionType type;
  EdgePosition collision_midpoint;
  Move corrected_move;

  CollisionInfo(CollisionType type, EdgePosition pos, const Move &m)
      : type(type), collision_midpoint(pos), corrected_move(m){};

  CollisionInfo(CollisionType type, EdgePosition pos)
      : type(type), collision_midpoint(pos){};
  CollisionInfo(CollisionType type, TrainPosition pos)
      : type(type), corrected_move(pos){};

  CollisionInfo(CollisionType type) : type(type){};
};

bool ttd_collision_with_moved_trains(const State &state, const Move &move,
                                     const Graph &g) {
  for (size_t i = 0; i < state.local_moves; i++) {
    Move other_train_pos = state.positions[i];
    if (g.edge_collision(move, other_train_pos)) {
      return true;
    }
  }
  return false;
}

EdgePosition collision_midpoint(const State &state, const Move &m1,
                                const Move &m2, const Graph &g) {
  int32_t from1, to1, from2, to2;
  from1 = g.pos_from_left(m1.start.edge, m1.start.pos);
  to1 = g.pos_from_left(m1.end.edge, m1.end.pos);
  from2 = g.pos_from_left(m2.start.edge, m2.start.pos);
  to2 = g.pos_from_left(m2.end.edge, m2.end.pos);

  if (m1.start.edge == m2.end.edge && m1.end.edge == m2.start.edge) {
    int32_t mid =
        from1 > to2 ? to2 + (from1 - to2) / 2 : to1 + (from2 - to1) / 2;
    return {m1.start.edge, mid};
  } else if (m1.start.edge == m2.end.edge) {
    return {m1.start.edge, to2 + (from1 - to2) / 2};
  }
  return {m1.end.edge, to1 + (from2 - to1) / 2};
}

CollisionInfo collision(const State &state, const Move &move, const Graph &g) {
  CollisionInfo ret(CollisionType::none, move);
  for (size_t i = 0; i < state.local_moves; i++) {
    Move other_train_pos = state.positions[i];

    if (g.edge_collision(ret.corrected_move, other_train_pos)) {
      
      /* Trains moving in opposite directions, uncorrectable collision */
      if (ret.corrected_move.direction() != other_train_pos.direction())
        return CollisionInfo(CollisionType::ttd_front);

      if (g.collision(ret.corrected_move, other_train_pos)) {
        if (ret.corrected_move.end.edge == other_train_pos.start.edge) {
          Move new_move = g.truncate(ret.corrected_move, other_train_pos.start);

          ret = CollisionInfo(CollisionType::ttd_back, other_train_pos.start,
                              new_move);
          continue;
        }
        else{          
          return CollisionInfo(CollisionType::train);
        }
      }

      //      Find collision edge and find midpoint to introduce TTD
 
      EdgePosition pos =
          collision_midpoint(state, ret.corrected_move, other_train_pos, g);
      ret =  CollisionInfo(CollisionType::ttd_back, pos, ret.corrected_move);
      continue;
      
    }
  }
  return ret;
}

CollisionInfo collision_against_back(const State &state, const Move &move,
                                     const Graph &g) {
  for (size_t i = state.local_moves + 1; i < state.positions.size(); i++) {
    Move other_train_pos = state.positions[i];
    if (g.collision(move, other_train_pos)) {
      if (move.end.edge == other_train_pos.start.edge) {
        Move new_move = g.truncate(move, other_train_pos.start);
        return CollisionInfo(CollisionType::ttd_back, other_train_pos.start,
                             new_move);
      }
      return CollisionInfo(CollisionType::train);
    }
  }
  return CollisionInfo(CollisionType::none);
}

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
    /* perform initial search from goal backward to determine length of
     * optimal route */
    /*###############################################################################
     */

    using BFSEntry = std::pair<uint32_t, Edge>;
    for (size_t train_index = 0; train_index < trains.size(); train_index++) {
      const Train &train = trains[train_index];

      std::unordered_set<Edge> closed_set;
      std::priority_queue<BFSEntry, std::vector<BFSEntry>,
                          std::greater<BFSEntry>>
          bfs_fringe;

      BFSEntry init = {std::abs(train.end_position().pos),
                       train.end_position().edge};
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

      // for (int i = 0; i < optimal_lengths[train_index].size(); i++) {
      //   std::cout << i << " " << optimal_lengths[train_index][i] << "\n";
      // }
      // std::cout << ""
      //           << "\n";
    }
    //    int train = 0;
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
    for (auto t : state.time)
      cost += t;
    // for (auto t : state.time)
    //   cost = t > cost? t : cost;
    return cost;
  }

  // uint32_t num_boundaries(const State &state) const {
  //   uint32_t cnt = 0;
  //   for (auto [_, partitions] : state.vsss)
  //     cnt += partitions.size();
  //   return cnt;
  //  }

  /* heuristic for A* search */
  float
  h(const State &state) const { // TODO: improve heuristic by single agent A*
    float estimate = 0;
    // for (size_t train_index = 0; train_index < trains.size(); train_index++)
    // {
    //   const EdgePosition &engine_pos = state.positions[train_index].end;
    //   uint32_t length_in_curr_edge = engine_pos.length();
    //   float train_estimate =  (optimal_lengths[train_index][engine_pos.edge]
    //   != 0) *
    //               (optimal_lengths[train_index][engine_pos.edge] -
    //                length_in_curr_edge) /
    //               ((float)trains[train_index].speed);
    //   estimate = train_estimate > estimate ? train_estimate : estimate;
    // }
    for (size_t train_index = 0; train_index < trains.size(); train_index++) {
      const EdgePosition &engine_pos = state.positions[train_index].end;
      uint32_t length_in_curr_edge = engine_pos.length();
      int length_estimate =
          (optimal_lengths[train_index][engine_pos.edge] != 0) *
          (optimal_lengths[train_index][engine_pos.edge] - length_in_curr_edge);
      float res = 0;
      if (length_estimate > 0)
        res = length_estimate / ((float)trains[train_index].speed);
      estimate += res;
    }
    return estimate;
  }

  float h(const State &state, uint32_t used_vss) const {
    float estimate = 0;
    for (size_t train_index = 0; train_index < trains.size(); train_index++) {
      const EdgePosition &engine_pos = state.positions[train_index].end;
      uint32_t length_in_curr_edge = engine_pos.length();
      int length_estimate =
          (optimal_lengths[train_index][engine_pos.edge] != 0) *
          (optimal_lengths[train_index][engine_pos.edge] - length_in_curr_edge);
      float res = 0;
      if (length_estimate > 0)
        res = length_estimate / ((float)trains[train_index].speed);
      if (train_index == used_vss)
        res = std::ceil(res);
      estimate += res;
    }
    return estimate;
  }

  inline float operator()(const State &state) const {
    return g(state) + h(state);
  }

  inline float operator()(const State &state, bool used_vss) const {
    return g(state) + h(state);
    if (used_vss)
      return g(state) + std::ceil(h(state));
    return g(state) + h(state);
  }
};

uint32_t sum_vss(const Search::VssMap &vss, const Graph &g) {
  uint32_t num_vss = 0;
  for (Edge e = 0; e < g.num_edges(); e++) {
    num_vss += 1;
    if (vss.find(e) != vss.end())
      num_vss += vss.at(e).size();
  }
  return num_vss;
}
Search::ETCSSolution Search::do_search() {
  auto equ = [](const std::vector<TrainPosition> &p1,
                const std::vector<TrainPosition> &p2) {
    for (size_t i = 0; i < p1.size(); i++)
      if (p1[i] != p2[i])
        return false;
    return true;
  };
  auto hasher = [](const std::vector<TrainPosition> &p) {
    std::vector<uint32_t> pos_hashes(p.size());
    for (size_t i = 0; i < p.size(); i++)
      pos_hashes[i] = p[i].hash();
    return jenkins_hash(pos_hashes.data(), p.size());
  };

  using SearchPair = std::pair<float, std::shared_ptr<State>>;
  std::unordered_set<std::vector<TrainPosition>, decltype(hasher),
                     decltype(equ)>
      closed_set(10000, hasher, equ); // TODO: initial bucket size
  std::priority_queue<SearchPair, std::vector<SearchPair>,
                      std::greater<SearchPair>>
      fringe;
  Score score(trains, graph);

  std::clock_t c_start = std::clock();

  std::shared_ptr<State> initial = std::make_shared<State>(trains.size());
  for (size_t i = 0; i < trains.size(); i++) {
    initial->positions[i] = trains[i].start_position();
  }
  fringe.push({score(*initial), initial});

  std::vector<bool> reached_goal(trains.size(), false);

  while (!fringe.empty()) {
    
    std::clock_t c_interm = std::clock();
    solve_time = 1000.0 * (c_interm - c_start) / CLOCKS_PER_SEC;
    if (solve_time >= timeout) {
      break;
    }

    auto [curr_score, curr_state_ptr] = fringe.top();
    const State &curr_state = *curr_state_ptr;
    fringe.pop();
    expanded++;

    /* check if goal state has been reached */
    std::fill(reached_goal.begin(), reached_goal.end(), false);
    for (size_t i = 0; i < trains.size(); i++) {
      Chain goal_pos =
          Chain(trains[i].end_position()); // TODO: goal as interval

      if (graph.collision(goal_pos, curr_state.positions[i])) {
        reached_goal[i] = true;
      }
    }

    if (curr_state.time[curr_state.local_moves] == max_timesteps)
      continue;

    if (std::all_of(reached_goal.begin(), reached_goal.end(),
                    [](bool v) { return v; })) {

      std::vector<Route> routes(trains.size());

      std::shared_ptr<State> curr = curr_state_ptr;
      for (auto t : curr->time)
        sum_times += t;

      while (curr != nullptr) {
        for (size_t i = 0; i < trains.size(); i++)
          routes[i].push_back(curr->positions[i]);
        curr = curr->prev;
      }
      for (size_t i = 0; i < trains.size(); i++) {
        std::reverse(routes[i].begin(), routes[i].end());
      }
      //      std::vector<Search::VssMap> ass;
      std::clock_t c_end = std::clock();
      solve_time = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
      num_vss = sum_vss(curr_state.vss, graph);
      make_span = routes[0].size();
      solved = true;
      return {curr_state.vss, routes};
    }

    /* All trains have moved, advance to next full state */
    if (curr_state.local_moves == trains.size()) {
      std::shared_ptr<State> next_full_state =
          advance_to_full_state(curr_state, graph, trains);

      if (closed_set.find(next_full_state->positions) != closed_set.end()) {

        duplicate_states++;
        continue;
      }
      bool all_started = true;
      for (size_t i = 0; i < trains.size(); i++) {
        if (trains[i].start_time() > next_full_state->time[i]) {
          all_started = false;
          break;
        }
      }
      if (all_started)
        closed_set.insert(next_full_state->positions);

      fringe.push({score(*next_full_state), next_full_state});
      continue;
    }

    const Train &curr_train = trains[curr_state.local_moves];

    /* Advance next train locally */
    if (trains[curr_state.local_moves].start_time() >
        curr_state.time[curr_state.local_moves]) {
      auto next_state = advance_local_state(
          curr_state_ptr, curr_state.positions[curr_state.local_moves]);
      fringe.push({score(*next_state), next_state});

      continue;
    }
    if (reached_goal[curr_state.local_moves]) { // If train in goal, try to
                                                // not move
      if (!ttd_collision_with_moved_trains(
              curr_state, curr_state.positions[curr_state.local_moves],
              graph)) {
        std::shared_ptr<State> next_state = advance_local_state(
            curr_state_ptr, curr_state.positions[curr_state.local_moves]);
        next_state->time[curr_state.local_moves]--; // TODO: wrong?

        fringe.push({score(*next_state), next_state});
      }
    }

    /* check all possible moves */
    Graph::SubGraph reachable_graph =
        graph.sub_graph(curr_state.positions[curr_state.local_moves].end,
                        curr_train.speed, {}); // TODO
                                               // add
                                               // whole
                                               // move
    int prev_furthest_edge = -1;
    bool prev_collision = false;
    
    for (auto it = reachable_graph.begin(); it != reachable_graph.end(); ++it) {
      
      Chain path =
          combine_chains(curr_state.positions[curr_state.local_moves], *it);
      //           std::cout << path << "\n\n";
      bool loop = false;
      for(size_t i = 0; i < path.covered_edges.size(); i++) {
        for(size_t j = i+1; j < path.covered_edges.size(); j++) {
          if (path.covered_edges[i] == path.covered_edges[j]) {
            loop = true;
            break;
          }
        }
        if(loop)
          break;
      }
      if(loop)
        break;
      // prevent redundant checks for collision
      if (prev_collision) {
        bool same_edge = false;
        if (path.start.edge == prev_furthest_edge)
          continue;
        for (auto &e : path.covered_edges) {
          if (e == prev_furthest_edge) {
            same_edge = true;
            break;
          }
        }
        if (same_edge)
          continue;
      }

      prev_furthest_edge = path.end.edge;
      prev_collision = false;

      CollisionInfo coll_info = collision(curr_state, path, graph);
      bool used_vss = false;
      std::shared_ptr<State> next_state;

      switch (coll_info.type) {
      case CollisionType::ttd_front: {
        ttd_collisions++;
        prev_collision = true;
        continue;
      }
      case CollisionType::train: {
        train_collisions++;
        prev_collision = true;
        continue;
      }
      case CollisionType::ttd_back: {
        corrected_positions++;
        next_state =
            advance_local_state(curr_state_ptr, coll_info.corrected_move);
        next_state->vss[coll_info.collision_midpoint.edge].insert(
            std::abs(coll_info.collision_midpoint.pos));
        used_vss = true;
        break;
      }
      case CollisionType::none: {
        next_state = advance_local_state(curr_state_ptr, path);
        break;
      }
      }
    //       std::cout << "collision" << "\n";
    // std::cout << ret.corrected_move << "\n";
    // std::cout << other_train_pos << "\n\n";
      // If train has to move after reaching goal, the time spent has to be
      // accounted for, may actually be unnecessary ?
      if (reached_goal[curr_state.local_moves]) {
        next_state->time[curr_state.local_moves] =
            *std::max_element(next_state->time.begin(), next_state->time.end());
      }
      fringe.push({score(*next_state, used_vss), next_state});

      /* We also have to check collisions with trains that haven't moved.
       If a collision occurs that collision might be adjusted by moving a
       shorter distance.*/
      coll_info = collision_against_back(curr_state, path, graph);
      if (coll_info.type != CollisionType::ttd_back) {
        continue;
      }
      corrected_positions++;
      next_state =
          advance_local_state(curr_state_ptr, coll_info.corrected_move);
    }
  }
  std::clock_t c_end = std::clock();
  solve_time = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
  ETCSSolution s;
  return s;
}

void Search::print_statistics() const {
  if (solve_time >= timeout) {
    std::cout << "TIMEOUT"
              << "\n";
    return;
  }
  std::cout << (solved ? "SOLVED" : "UNSOLVED") << "\n";
  std::cout << "EXPANDED NODES: " << expanded << "\n";
  std::cout << "DUPLICATE STATES GENERATED: " << duplicate_states << "\n";
  std::cout << "TTD COLLISIONS: " << ttd_collisions << "\n";
  std::cout << "CORRECTED POSITIONS: " << corrected_positions << "\n";

  std::cout << "TRAIN COLLISIONS: " << train_collisions << "\n";
  std::cout << "SOLVE TIME[ms]: " << solve_time << "\n";
  std::cout << "NUM VSS: " << num_vss << "\n";
  std::cout << "MAKESPAN: " << make_span << "\n";
  std::cout << "SUM_TIMES: " << sum_times << "\n";
}
