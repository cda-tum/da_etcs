#include <vector>
#include <stdint.h>
#include <cstddef>

size_t jenkins_hash(const uint32_t* nums, size_t nums_size);

size_t jenkins_hash(const std::vector<uint32_t>& nums);

int32_t bool_to_sign(bool b);

template<typename T>
T pop(std::vector<T>& stack);

template<typename T>
void push(std::vector<T>& stack, T& val);

template<typename T>
void push(std::vector<T>& stack, const T& val);

template<typename T>
T top(std::vector<T>& stack);
