//#include "../include/railway.hpp"
#include "astar/search.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

// Railway *parse_railway(const std::string &file_path) {
//   std::ifstream file(file_path);
//   if (!file.is_open()) {
//     std::cerr << "Instance-file not found" << std::endl;
//     file.close();
//     std::exit(-1);
//   }

//   int num_nodes, max_timesteps;
//   file >> num_nodes >> max_timesteps;

//   Graph g = Graph(num_nodes);

//   std::string s;
//   getline(file, s);
//   getline(file, s);
//   if (!s.empty()) {
//     std::cerr << "Ill-formed instance file" << std::endl;
//     file.close();
//     std::exit(-1);
//   }

//   // read TTDs
//   while (getline(file, s) && !s.empty()) {
//     int from, to, length, speed_limit;
//     std::stringstream tmp(s);
//     tmp >> from >> to >> length >> speed_limit;
//     g.add_edge(from, to, length, speed_limit);
//   }

//   while (getline(file, s) && !s.empty()) {
//     int ttd1, ttd2;
//     std::stringstream tmp(s);
//     tmp >> ttd1 >> ttd2;
//     g.invalidate_turn(ttd1, ttd2);
//   }

//   std::vector<Train> trains;
//   // read trains
//   while (getline(file, s)) {
//     int speed, length, time, ttd, to;
//     std::stringstream tmp(s);
//     tmp >> speed >> length >> time >> ttd >> to;
//     Stop stop(ttd, to, time);
//     Train train(speed, length);
//     train.add_stop(stop);
//     // read trainstops
//     while (getline(file, s) && !s.empty()) {
//       int arrival_time, ttd, to;
//       std::stringstream tmp2(s);
//       tmp2 >> arrival_time >> ttd >> to;
//       stop = Stop(ttd, to, arrival_time);
//       train.add_stop(stop);
//     }
//     trains.push_back(train);
//     if (file.eof())
//       break;
//   }

//   file.close();

//   return new Railway(g, trains);
// }

Search *parse_search(const std::string &file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Instance-file not found" << std::endl;
    file.close();
    std::exit(-1);
  }

  int num_nodes, max_timesteps;
  file >> num_nodes >> max_timesteps;

  Graph g = Graph(num_nodes);

  std::string s;
  getline(file, s);
  getline(file, s);
  if (!s.empty()) {
    std::cerr << "Ill-formed instance file" << std::endl;
    file.close();
    std::exit(-1);
  }

  // read TTDs
  while (getline(file, s) && !s.empty()) {
    int from, to, length, speed_limit;
    std::stringstream tmp(s);
    tmp >> from >> to >> length >> speed_limit;
    g.add_edge(from, to, length, speed_limit);
  }

  while (getline(file, s) && !s.empty()) {
    int ttd1, ttd2;
    std::stringstream tmp(s);
    tmp >> ttd1 >> ttd2;
    g.invalidate_turn(ttd1, ttd2);
  }

  std::vector<Train> trains;
  // read trains
  while (getline(file, s)) {
    int speed, length, time, ttd, to;
    std::stringstream tmp(s);
    tmp >> speed >> length >> time >> ttd >> to;
    Stop stop(ttd, to, time);
    Train train(speed, length);
    train.add_stop(stop);
    // read trainstops
    while (getline(file, s) && !s.empty()) {
      int arrival_time, ttd, to;
      std::stringstream tmp2(s);
      tmp2 >> arrival_time >> ttd >> to;
      stop = Stop(ttd, to, arrival_time);
      train.add_stop(stop);
    }
    trains.push_back(train);
    if (file.eof())
      break;
  }

  file.close();

  return new Search(g, trains, max_timesteps);
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cout << "No file provided" << std::endl;
    std::exit(-1);
  }

  // Railway *railway = parse_railway(argv[1]);

  // auto [assignments, routes] = railway->optimal_schedule();
  // for (size_t i = 0; i < routes.size(); i++) {
  //   std::cout << "Route of train  " << i << std::endl;
  //   int time = 0;
  //   for (auto m : routes[i])
  //     std::cout << "time " << time++ << ": " << m << std::endl;
  //   std::cout << std::endl;
  // }
  // std::cout << "VSS:"
  //           << "\n";

  // for (auto [k, v] : assignments) {
  //   for (uint32_t boundary_pos : v)
  //     std::cout << "Vss at " << k << " " << boundary_pos << std::endl;
  // }

  Search *search = parse_search(argv[1]);
  auto [assignments, routes] = search->do_search();
  for (size_t i = 0; i < routes.size(); i++) {
    std::cout << "Route of train  " << i << std::endl;
    int time = 0;
    for (auto m : routes[i])
      std::cout << "time " << time++ << ": " << m << std::endl;
    std::cout << std::endl;
  }
  std::cout << "VSS:"
            << "\n";

  for (auto [k, v] : assignments) {
    for (uint32_t boundary_pos : v)
      std::cout << "Vss at " << k << " " << boundary_pos << std::endl;
  }
  search->print_statistics();
}
