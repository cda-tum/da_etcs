#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <memory>
#include <set>
#include <stdint.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using Edge = uint32_t;
using Vertex = uint32_t;

class EdgeLabel {
  friend class Graph;

public:
  Vertex left, right;
  uint32_t length, speed_limit;
  std::vector<Edge> adj;

private:
  void remove_adj_edge(Edge e);
  friend std::ostream &operator<<(std::ostream &os, const EdgeLabel &label);
};

/*
 * Position of train on edge are described by an interval defined relative to
 * vertices If from, to >= 0 then position is defined relative to left vertex If
 * from, to <= 0 then position is defined relative to right vertex
 */
struct EdgePosition {
  Edge edge;
  //>= 0 -> distance from left vertex
  //< 0 -> distance from right vertex
  int32_t pos;

  EdgePosition() = default;
  EdgePosition(const Edge &edge, int32_t pos);
  EdgePosition(const EdgePosition &other);
  EdgePosition &operator=(const EdgePosition &other) = default;

  bool is_left_right() const;
  bool is_right_left() const;
  /* +1 if left->right, -1 if right->left*/
  int8_t direction() const;

  bool operator==(const EdgePosition &other) const;
  bool operator!=(const EdgePosition &other) const;
  inline size_t hash() const { return (((size_t)edge) << 32) | pos; }

  inline uint32_t length() const { return std::abs(pos); }
};

std::ostream &operator<<(std::ostream &os, const EdgePosition &pos);

namespace std {
template <> struct hash<EdgePosition> {
  size_t operator()(const EdgePosition &pos) const { return pos.hash(); }
};
} // namespace std

class Chain;

class Graph {
  friend class EdgeLabel;

public:
  /* Constructors */
  Graph();
  Graph(const uint32_t num_vertices, const uint32_t num_edges);
  Graph(const uint32_t num_vertices);

  /* Muting Operations */
  Edge add_edge(uint32_t left, uint32_t right, uint32_t length,
                uint32_t speed_limit);
  void invalidate_turn(Edge e1, Edge e2);

  /* Data access */
  inline const EdgeLabel &operator[](const Edge &edge) const {
    return labels[edge];
  }
  inline const EdgeLabel &get_label(const Edge &edge) const {
    return labels[edge];
  }
  inline uint32_t get_length(const Edge &edge) const {
    return labels[edge].length;
  }
  inline size_t num_edges() const { return labels.size(); }
  inline const std::vector<Edge> &adjacent_edges(const Edge &edge) const {
    return labels[edge].adj;
  }
  bool connects_left(const Edge &e1, const Edge &e2) const;
  bool connects_right(const Edge &e1, const Edge &e2) const;
  std::vector<Edge> left_adjacent(const Edge &edge) const;
  std::vector<Edge> right_adjacent(const Edge &edge) const;
  bool are_adjacent(const Edge& e1, const Edge& e2) const;

  /* utility */

  /* Returns int based on how e1 connectes to e2
   * +1 if e1 connects to e2s left edge
   * -1 if e2 connects to e1s right edge
   * 0 if e1 and e2 are not connected
   */
  int connectivity(const Edge &e1, const Edge &e2) const;
  uint32_t pos_from_left(const Edge &e, int32_t pos) const;
  std::pair<uint32_t, uint32_t> pos_from_left(const Edge &e, int32_t from,
                                              int32_t to) const;
  Chain sub_chain_from_back(const Chain &chain, uint32_t length) const;
  Chain truncate(const Chain &chain, EdgePosition new_end) const;
  uint32_t chain_length(const Chain &chain) const;

  bool single_edge_collision(const Chain &single_edge, const Chain &c2) const;
  bool collision(const Chain &c1, const Chain &c2) const;
  bool covers(const Chain &c, const EdgePosition &e_pos) const;

  bool edge_collision(const Chain &c1, const Chain &c2) const;

  struct SubGraph {
  private:
    using SubGraphStructure =
        std::unordered_map<EdgePosition, std::unordered_set<EdgePosition>>;

    SubGraphStructure graph;
    EdgePosition start;
    const Graph &parent;

  public:
    SubGraph(const SubGraphStructure &g, const EdgePosition &start,
             const Graph &parent)
        : graph(g), start(start), parent(parent){};

    struct PathIterator {
      using iterator_category = std::forward_iterator_tag;
      using difference_type = std::ptrdiff_t;
      using value_type = Chain;
      using pointer = std::shared_ptr<Chain>;
      using reference = Chain &;

      PathIterator(const SubGraph &graph);
      PathIterator(const SubGraph &graph, bool end);

      inline reference operator*() const { return *curr_chain; }
      inline pointer operator->() { return curr_chain; }

      PathIterator &operator++();
      PathIterator operator++(int);

      friend bool operator==(const PathIterator &a, const PathIterator &b);
      friend bool operator!=(const PathIterator &a, const PathIterator &b);

    private:
      const SubGraph &graph;
      std::vector<std::pair<uint32_t, EdgePosition>> stack;
      uint32_t curr_depth = 0;
      EdgePosition curr_pos;
      std::vector<EdgePosition> curr_path;
      uint32_t depth = 0;
      pointer curr_chain;
      bool paused = false;

      void next_path();
    };

    PathIterator begin() const;
    PathIterator end() const;
  };

  Graph::SubGraph sub_graph(
      const EdgePosition &pos, uint32_t chain_length,
      const std::unordered_map<Edge, std::set<uint32_t>> &partitions) const;

private:
  std::vector<EdgeLabel> labels;
  std::vector<std::vector<Edge>> adj;

  bool overlapping(const EdgePosition &e1, const EdgePosition &e2) const;
  bool overlapping(const Edge &e, int32_t l1, int32_t r1, int32_t l2,
                   int32_t r2) const;
  bool overlapping(const Edge &e, int32_t l, int32_t r, int32_t x) const;

  /*assumes starti and endi are connected and distinct*/
  bool overlapping(const EdgePosition &start1, const EdgePosition &end1,
                   const EdgePosition &start2, const EdgePosition &end2) const;
};

std::ostream &operator<<(std::ostream &os, const EdgeLabel &label);

struct Chain {
  EdgePosition start, end;
  std::vector<Edge> covered_edges;

  Chain() = default;
  Chain(const EdgePosition &start, const EdgePosition &end);
  Chain(const EdgePosition &start);
  Chain(const EdgePosition &start, const EdgePosition &end,
        const std::vector<Edge> &covered_edges);
  Chain(const Chain &other);
  Chain(const std::vector<EdgePosition> &raw_chain);
  Chain &operator=(const Chain &other) = default;

  inline bool is_single_edge() const { return start.edge == end.edge; }
  bool operator==(const Chain &other) const;
  bool operator!=(const Chain &other) const;
  int8_t direction() const;
  size_t hash() const;
};

std::ostream &operator<<(std::ostream &os, const Chain &chain);

Chain combine_chains(const Chain &c1, const Chain &c2);

#endif
