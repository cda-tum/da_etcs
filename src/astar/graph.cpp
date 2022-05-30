#include "astar/graph.hpp"
#include "astar/utils.hpp"
#include <algorithm>
#include <iostream>
#include <set>
#include <unordered_set>

EdgePosition::EdgePosition(const Edge &edge, int32_t pos)
    : edge(edge), pos(pos) {}

EdgePosition::EdgePosition(const EdgePosition &other)
    : edge(other.edge), pos(other.pos) {}

bool EdgePosition::operator==(const EdgePosition &other) const {
  return edge == other.edge && pos == other.pos;
}

bool EdgePosition::operator!=(const EdgePosition &other) const {
  return !((*this) == other);
}

int8_t EdgePosition::direction() const { return bool_to_sign(pos >= 0); }

bool EdgePosition::is_left_right() const { return pos >= 0; }

bool EdgePosition::is_right_left() const { return pos < 0; }

std::ostream &operator<<(std::ostream &os, const EdgePosition &pos) {
  os << "[" << pos.edge << " " << pos.pos << "]";
  return os;
}

Chain::Chain(const EdgePosition &start, const EdgePosition &end)
    : start(start), end(end) {}

Chain::Chain(const EdgePosition &start) : start(start), end(start) {}

Chain::Chain(const EdgePosition &start, const EdgePosition &end,
             const std::vector<Edge> &covered_edges)
    : start(start), end(end), covered_edges(covered_edges) {}

Chain::Chain(const Chain &other)
    : start(other.start), end(other.end), covered_edges(other.covered_edges) {}

Chain::Chain(const std::vector<EdgePosition> &raw_chain)
    : start(*raw_chain.begin()), end(*(raw_chain.end() - 1)) {
  if (raw_chain.size() <= 2)
    return;

  if (raw_chain.size() == 3) {
    if (raw_chain[1].edge != raw_chain[0].edge &&
        raw_chain[1].edge != raw_chain[raw_chain.size() - 1].edge)
      covered_edges.push_back(raw_chain[1].edge);
    return;
  }

  if (raw_chain[1].edge != raw_chain[0].edge)
    covered_edges.push_back(raw_chain[1].edge);
  if (raw_chain[raw_chain.size() - 2].edge !=
      raw_chain[raw_chain.size() - 1].edge)
    covered_edges.push_back(raw_chain[raw_chain.size() - 2].edge);

  for (size_t i = 2; i < raw_chain.size() - 2; i++)
    covered_edges.push_back(raw_chain[i].edge);
}

bool Graph::connects_left(const Edge &e1, const Edge &e2) const {
  return labels[e1].left == labels[e2].right ||
         labels[e1].left == labels[e2].left;
}
bool Graph::connects_right(const Edge &e1, const Edge &e2) const {
  return labels[e1].right == labels[e2].right ||
         labels[e1].right == labels[e2].left;
}
std::vector<Edge> Graph::left_adjacent(const Edge &edge) const {
  std::vector<Edge> adj_left;
  for (const Edge &e : labels[edge].adj) {
    if (connects_left(edge, e))
      adj_left.push_back(e);
  }
  return adj_left;
}

std::vector<Edge> Graph::right_adjacent(const Edge &edge) const {
  std::vector<Edge> adj_right;
  for (const Edge &e : labels[edge].adj) {
    if (connects_right(edge, e))
      adj_right.push_back(e);
  }
  return adj_right;
}

bool Graph::are_adjacent(const Edge& e1, const Edge& e2) const {
  for (auto e : this->adj[e1])
    if(e == e2)
      return true;
  return false;
}

Chain Graph::sub_chain_from_back(const Chain &chain, uint32_t length) const {
  const EdgePosition &end = chain.end;
  const EdgePosition &start = chain.start;
  const std::vector<Edge> &covered_edges = chain.covered_edges;
  if (length <= end.length()) {
    int32_t start_pos = end.pos - (length * end.direction());
    EdgePosition new_start(end.edge, start_pos);
    return Chain(new_start, end);
  }

  uint32_t new_chain_length = end.length();
  EdgePosition new_end(end);
  std::vector<Edge> new_covered_edges;
  int direction = end.direction();
  Edge prev_edge = end.edge;

  for (auto it = covered_edges.rbegin();
       new_chain_length < length && it != covered_edges.rend(); it++) {
    Edge e = *it;
    new_chain_length += labels[e].length;
    direction *= this->connectivity(prev_edge, e);
    new_covered_edges.push_back(e);
    prev_edge = e;
  }
  if (new_chain_length >= length) {
    Edge new_start_edge = new_covered_edges.back();
    new_covered_edges.pop_back();
    std::reverse(new_covered_edges.begin(), new_covered_edges.end());
    Edge new_start_pos =
        labels[new_start_edge].length -
        (length - (new_chain_length - labels[new_start_edge].length));
    new_start_pos *= direction;
    EdgePosition new_start(new_start_edge, new_start_pos);
    return Chain(new_start, end, new_covered_edges);
  }

  std::reverse(new_covered_edges.begin(), new_covered_edges.end());
  direction *= this->connectivity(prev_edge, start.edge);
  uint32_t length_diff = length - new_chain_length;
  int32_t new_start_pos = direction * (labels[start.edge].length - length_diff);
  EdgePosition new_start(start.edge, new_start_pos);
  return Chain(new_start, end, new_covered_edges);
}

Chain Graph::truncate(const Chain &chain, EdgePosition new_end) const {
  if (chain.start.edge == new_end.edge)
    return Chain(chain.start, new_end);

  if (chain.end.edge == new_end.edge) // TODO: what if new_end beyond old end?
    return Chain(chain.start, new_end, chain.covered_edges);

  std::vector<Edge> new_covered;
  for (const Edge &e : chain.covered_edges) {
    if (new_end.edge == e)
      break;
    new_covered.push_back(e);
  }
  return Chain(chain.start, new_end, new_covered);
}

bool Chain::operator==(const Chain &other) const {
  if (start != other.start || end != other.end ||
      covered_edges.size() != other.covered_edges.size())
    return false;

  for (size_t index = 0; index < covered_edges.size(); index++) {
    if (covered_edges[index] != other.covered_edges[index])
      return false;
  }
  return true;
}
bool Chain::operator!=(const Chain &other) const { return !((*this) == other); }

size_t Chain::hash() const {
  std::vector<uint32_t> nums = covered_edges;
  nums.push_back(start.edge);
  nums.push_back(start.pos);
  nums.push_back(end.edge);
  nums.push_back(end.pos);
  return jenkins_hash(nums);
}

int8_t Chain::direction() const {
  return end.direction();
}

uint32_t Graph::pos_from_left(const Edge &e, int32_t pos) const {
  return pos + labels[e].length * (pos < 0);
}

std::pair<uint32_t, uint32_t> Graph::pos_from_left(const Edge &e, int32_t from,
                                                   int32_t to) const {
  uint32_t new_from;
  if (from == 0 && to < 0)
    new_from = labels[e].length;
  else {
    new_from = pos_from_left(e, from);
  }
  if(to >= 0)
    return {new_from, pos_from_left(e, to)};
  return {pos_from_left(e, to), new_from};
}

bool Graph::overlapping(const EdgePosition &e1, const EdgePosition &e2) const {
  if (e1.edge != e2.edge)
    return false;

  if (e1.direction() != e2.direction())
    return e1.length() + e2.length() > labels[e1.edge].length;

  return true;
}

bool Graph::overlapping(const Edge &e, int32_t l1, int32_t r1, int32_t l2,
                        int32_t r2) const {
  auto [from1, to1] = pos_from_left(e, l1, r1);
  auto [from2, to2] = pos_from_left(e, l2, r2);

  /* check if intervals overlap */
  if (from2 > from1 && from2 < to1)
    return true;
  if (to2 > from1 && to2 < to1)
    return true;
  return from1 >= from2 && to1 <= to2;
}

bool Graph::overlapping(const Edge &e, int32_t l, int32_t r, int32_t x) const {

  auto [from1, to1] = pos_from_left(e, l, r);
  uint32_t x_left = pos_from_left(e, x);

  if (to1 < from1)
    std::swap(from1, to1);
  
  return from1 < x_left && x_left < to1;
}

bool Graph::overlapping(const EdgePosition &start1, const EdgePosition &end1,
                        const EdgePosition &start2,
                        const EdgePosition &end2) const {
  bool start_end_coll = false;
  bool end_start_coll = false;
  if (start1.edge == end2.edge) {
    start_end_coll = end2.pos > start1.pos;
  }
  if (end1.edge == start2.edge) {
    end_start_coll = end1.pos > start2.pos;
  }

  if (overlapping(start1, start2) || start_end_coll || end_start_coll ||
      overlapping(end1, end2))
    return true;

  return false;
}

bool Graph::single_edge_collision(const Chain &single_edge,
                                  const Chain &c2) const {
  if (c2.is_single_edge()) { // check intervals overlapping
    if (c2.start.edge != single_edge.start.edge)
      return false;

    return overlapping(single_edge.start.edge, single_edge.start.pos,
                       single_edge.end.pos, c2.start.pos, c2.end.pos);
  }
  // check if ends of c2 overlap with single_edge
  if (c2.start.edge == single_edge.start.edge)
    return overlapping(single_edge.start.edge, single_edge.start.pos,
                       single_edge.end.pos, c2.start.pos,
                       c2.start.direction() * this->get_length(c2.start.edge));

  if (c2.end.edge == single_edge.start.edge)
    return overlapping(single_edge.start.edge, single_edge.start.pos,
                       single_edge.end.pos, 0, c2.end.pos);

  // c2 must cover single_edge
  for (const Edge &edge : c2.covered_edges) {
    if (edge == single_edge.start.edge)
      return true;
  }
  return false;
}

bool Graph::collision(const Chain &c1, const Chain &c2) const {
  if(c1.end.edge == c2.end.edge && c1.end.pos == c2.end.pos)
    return true;
  if(c1.start.edge == c2.start.edge && c1.start.pos == c2.start.pos)
    return true;
  if (c1.is_single_edge()) {
    return this->single_edge_collision(c1, c2);
  }
  if (c2.is_single_edge()) {
    return this->single_edge_collision(c2, c1);
  }
  if (overlapping(c1.start, c1.end, c2.start, c2.end))
    return true;
  for (const Edge &e1 : c1.covered_edges) { // TODO: might be a performance
                                            // issue
    for (const Edge &e2 : c2.covered_edges) {
      if (e1 == e2)
        return true;
    }
    if (e1 == c2.start.edge || e1 == c2.end.edge)
      return true;
  }
  for (const Edge &e2 : c2.covered_edges) {
    if (e2 == c1.start.edge || e2 == c1.end.edge)
      return true;
  }
  return false;
}

bool Graph::covers(const Chain &c,
                   const EdgePosition &e_pos) const { // TODO: broken
  if (c.end.edge == e_pos.edge)
    return true;
  if (c.start.edge == e_pos.edge)
    return true;
  for (const Edge &e : c.covered_edges)
    if (e == e_pos.edge)
      return true;
  return false;
}

bool Graph::edge_collision(const Chain &c1, const Chain &c2) const {
  if (c1.start.edge == c2.start.edge || c1.start.edge == c2.end.edge ||
      c1.end.edge == c2.start.edge || c1.end.edge == c2.end.edge)
    return true;
  for (const Edge &e1 : c1.covered_edges) { // TODO: might be a performance
                                            // issue
    for (const Edge &e2 : c2.covered_edges) {
      if (e1 == e2)
        return true;
    }
    if (e1 == c2.start.edge || e1 == c2.end.edge)
      return true;
  }
  for (const Edge &e2 : c2.covered_edges) {
    if (e2 == c1.start.edge || e2 == c1.end.edge)
      return true;
  }
  return false;
}

std::ostream &operator<<(std::ostream &os, const Chain &chain) {
  os << "start: [" << chain.start.edge << " " << chain.start.pos << "] ";
  for (const Edge &edge : chain.covered_edges)
    os << "[" << edge << "] ";
  os << "[" << chain.end.edge << " " << chain.end.pos << "] : end";
  return os;
}

void EdgeLabel::remove_adj_edge(Edge e) {
  size_t erase_index = 0;
  for (; erase_index < adj.size(); erase_index++) {
    if (adj[erase_index] == e)
      break;
  }
  if (erase_index == adj.size())
    return;

  adj.erase(adj.begin() + erase_index);
}
Graph::Graph() : labels(0), adj(0) {}
Graph::Graph(const uint32_t num_vertices, const uint32_t num_edges)
    : labels(num_vertices), adj(num_edges) {}

Graph::Graph(const uint32_t num_vertices) : labels(0), adj(num_vertices) {}

Edge Graph::add_edge(uint32_t left, uint32_t right, uint32_t length,
                     uint32_t speed_limit) {
  labels.push_back({left, right, length, speed_limit, std::vector<Edge>()});
  Edge new_edge = labels.size() - 1;
  EdgeLabel &new_label = labels.back();

  std::unordered_set<Edge> added;

  for (Edge adj : adj[left]) {
    if (added.find(adj) != added.end())
      continue;
    added.insert(adj);
    new_label.adj.push_back(adj);
    labels[adj].adj.push_back(new_edge);
  }

  for (Edge adj : adj[right]) {
    if (added.find(adj) != added.end())
      continue;
    added.insert(adj);
    new_label.adj.push_back(adj);
    labels[adj].adj.push_back(new_edge);
  }

  adj[left].push_back(new_edge);
  adj[right].push_back(new_edge);

  return new_edge;
}

void Graph::invalidate_turn(Edge e1, Edge e2) {
  EdgeLabel &label1 = labels[e1];
  EdgeLabel &label2 = labels[e2];
  label1.remove_adj_edge(e2);
  label2.remove_adj_edge(e1);
}

int Graph::connectivity(const Edge &e1, const Edge &e2) const {
  const EdgeLabel &l1 = labels[e1];
  const EdgeLabel &l2 = labels[e2];

  if (l1.right == l2.left || l1.left == l2.right)
    return 1;

  if (l1.left == l2.left || l1.right == l2.right)
    return -1;

  return 0;
}

std::ostream &operator<<(std::ostream &os, const EdgeLabel &label) {
  os << "edge from " << label.left << " to " << label.right;
  return os;
}

Graph::SubGraph Graph::sub_graph(
    const EdgePosition &pos, uint32_t chain_length,
    const std::unordered_map<Edge, std::set<uint32_t>> &partitions) const {
  std::unordered_map<EdgePosition, std::unordered_set<EdgePosition>> sub_graph;
  std::vector<std::pair<uint32_t, EdgePosition>> stack;
  stack.push_back({0, pos});

  // TODO: account for speed limit

  while (!stack.empty()) {
    auto [curr_length, curr_pos] = stack.back();
    stack.pop_back();

    if (curr_length >= chain_length)
      continue;

    if (curr_pos.length() <
        labels[curr_pos.edge].length) { // TODO is this needed?
      uint32_t max_length = chain_length - curr_length;
      uint32_t abs_pos = curr_pos.pos * curr_pos.direction() + max_length;
      int32_t signed_pos = std::min(abs_pos, labels[curr_pos.edge].length);
      signed_pos *= curr_pos.direction();

      EdgePosition new_pos(curr_pos.edge, signed_pos);

      sub_graph[curr_pos].insert(new_pos);
      uint32_t new_length = curr_length + std::abs(new_pos.pos - curr_pos.pos);
      stack.push_back({new_length, new_pos});

      /* we get the same set of adjacent positions
       *if we only look at reachable positions from further down the edge
       */
      continue;
    }

    std::vector<Edge> adjacent;
    if (curr_pos.pos >= 0)
      adjacent = this->right_adjacent(curr_pos.edge);
    else
      adjacent = this->left_adjacent(curr_pos.edge);

    uint32_t length_diff = chain_length - curr_length;

    for (const Edge adj : adjacent) {
      if (adj == curr_pos.edge)
        continue;

      int32_t pos = std::min(labels[adj].length, length_diff);
      pos *= this->connectivity(curr_pos.edge, adj) * curr_pos.direction();

      if (partitions.count(adj) != 0) {
        std::set<uint32_t> positions = partitions.at(adj);
        int32_t boundary_pos =
            ((int32_t)*positions.begin()) - ((int32_t)get_length(adj));
        EdgePosition boundary = EdgePosition(adj, boundary_pos);
        sub_graph[{curr_pos}].insert(boundary);
        EdgePosition prev = boundary;

        for (auto &it = ++positions.begin(); it != positions.end(); it++) {
          boundary_pos = ((int32_t)*it) - ((int32_t)get_length(adj));
          if (std::abs(boundary_pos) > std::abs(pos)) {
            sub_graph[prev].insert(EdgePosition(adj, pos));
            break;
          }
          boundary = EdgePosition(adj, boundary_pos);
          sub_graph[prev].insert(boundary);
          prev = boundary;
        }
      } else {
        EdgePosition new_pos({adj, pos});
        // TODO: don't visit duplicates
        sub_graph[curr_pos].insert(new_pos);
        stack.push_back({curr_length + std::abs(pos), new_pos});
      }
    }
  }
  return Graph::SubGraph(sub_graph, pos, *this);
}

uint32_t Graph::chain_length(const Chain &chain) const {
  uint32_t length = 0;
  for (const Edge &e : chain.covered_edges) {
    length += labels[e].length;
  }

  return length + chain.end.length() -
         (chain.start.length() * bool_to_sign(chain.is_single_edge()));
}

Graph::SubGraph::PathIterator Graph::SubGraph::begin() const {
  return PathIterator(*this);
}

Graph::SubGraph::PathIterator Graph::SubGraph::end() const {
  return PathIterator(*this, true);
}

Graph::SubGraph::PathIterator::PathIterator(const SubGraph &graph, bool end)
    : graph(graph), curr_pos(graph.start), curr_chain(nullptr) {}

Graph::SubGraph::PathIterator::PathIterator(const SubGraph &graph)
    : graph(graph), curr_pos(graph.start) {
  stack.push_back({depth, graph.start});
  next_path();
}

Graph::SubGraph::PathIterator &Graph::SubGraph::PathIterator::operator++() {
  next_path();
  return *this;
}

Graph::SubGraph::PathIterator Graph::SubGraph::PathIterator::operator++(int) {
  PathIterator temp = *this;
  ++(*this);
  return temp;
}

bool operator==(const Graph::SubGraph::PathIterator &a,
                const Graph::SubGraph::PathIterator &b) {
  return a.curr_chain == b.curr_chain;
}
bool operator!=(const Graph::SubGraph::PathIterator &a,
                const Graph::SubGraph::PathIterator &b) {
  return !(a == b);
}

void Graph::SubGraph::PathIterator::next_path() {
  while (!stack.empty() || paused) {
    if (!paused) {
      auto pair = stack.back();
      curr_depth = pair.first;
      curr_pos = pair.second;
      stack.pop_back();

      while (depth > curr_depth) {
        depth--;
        curr_path.pop_back();
        //      curr_chain = std::make_shared<Chain>(curr_path);
      }
      curr_path.push_back(curr_pos);
      depth++;

      if (!(curr_path[0] == *(curr_path.end() - 1) &&
            graph.parent.get_length(curr_path.back().edge) !=
                curr_path.back().length())) {

        curr_chain = std::make_shared<Chain>(curr_path);

        
        //    std::cout << *curr_chain<< std::endl;

        paused = true;
        return;
      }
    }
    paused = false;
    if (graph.graph.count(curr_pos) == 0)
      continue;

    for (const EdgePosition &adj : graph.graph.at(curr_pos))
      stack.push_back({depth, adj});
  }

  curr_chain = nullptr;
}

Chain combine_chains(const Chain &c1, const Chain &c2) {
  if (c1.end == c2.start) {
    std::vector<Edge> covered;
    covered.reserve(c2.covered_edges.size() + c1.covered_edges.size() + 1);
    covered.insert(covered.end(), c1.covered_edges.begin(),
                   c1.covered_edges.end());
    if (c2.start.edge != c2.end.edge && c1.start.edge != c1.end.edge) {
      covered.push_back(c2.start.edge);
    }
          covered.insert(covered.end(), c2.covered_edges.begin(),
                     c2.covered_edges.end());
    return Chain(c1.start, c2.end, covered);

  } else if (c1.start == c2.end) {
    std::vector<Edge> covered;
    covered.reserve(c2.covered_edges.size() + c1.covered_edges.size() + 1);
    covered.insert(covered.end(), c2.covered_edges.begin(),
                   c2.covered_edges.end());
    if (c1.start.edge != c1.end.edge) {
      covered.push_back(c1.start.edge);
      covered.insert(covered.end(), c1.covered_edges.begin(),
                     c1.covered_edges.end());
    }
    return Chain(c1.start, c2.end, covered);
  } else if (c2.start != c1.end) {
    return c1;
  }
  return Chain({0, 0}, {0, 0}); // TODO: probably an issue
}
