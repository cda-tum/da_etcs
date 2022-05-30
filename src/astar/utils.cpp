#include "astar/utils.hpp"

size_t jenkins_hash(const uint32_t *nums, size_t nums_size) {
  size_t hash = 0;
  for (size_t i = 0; i < nums_size; i++) {
    hash += nums[i];
    hash += hash << 10;
    hash ^= hash >> 6;
  }
  hash += hash << 3;
  hash ^= hash >> 11;
  hash += hash << 15;
  return hash;
}

size_t jenkins_hash(const std::vector<uint32_t> &nums) {
  return jenkins_hash(nums.data(), nums.size());
}

/* helpful for branchless programming */
int32_t bool_to_sign(bool b) { return b * 2 - 1; }

template <typename T> T pop(std::vector<T> &stack) {
  T val = stack.back();
  stack.pop_back();
  return val;
}

template <typename T> void push(std::vector<T> &stack, T &val) {
  stack.push_back(val);
}

template <typename T> void push(std::vector<T> &stack, const T &val) {
  stack.push_back(val);
}

template <typename T> T top(std::vector<T> &stack) { return stack.back(); }
