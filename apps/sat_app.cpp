#include "z3++.h"
#include <vector>
#include "../include/graph.h"
#include "../include/edge.h"
#include "../include/train.h"
#include "../include/instance.h"
#include <iostream>
#include <sstream>
#include <fstream>

using namespace z3;

Instance* parseInstance(const std::string& pathToFile) {
  std::ifstream file(pathToFile);
  if(! file.is_open()) {
    std::cerr << "Instance-file not found" << std::endl;
    file.close();
    std::exit(-1);
  }

  int numNodes, maxTimeSteps;
  file >> numNodes >> maxTimeSteps;
  Graph g(numNodes);

  std::string s;
  getline(file, s);
  getline(file, s);
  if(!s.empty()) {
    std::cerr << "Ill-formed instance file" << std::endl;
    file.close();
    std::exit(-1);
  }
  
  //read edges
  while(getline(file, s) && !s.empty()) {
    int from, to, length, maxSpeed, ttd;
    std::stringstream tmp(s);
    tmp >> from >> to >> length >> maxSpeed >> ttd;
    g.addEdge(from, to, length, maxSpeed, ttd);
  }
  g.setBoundaries();

  while(getline(file, s) && !s.empty()) {
    int left, mid, right;
    std::stringstream tmp(s);
    tmp >> left >> mid >> right;
    g.prohibitTransition(left, mid, right);
  }
  
  std::vector<Train> trains;
  int trainId = 0;
  //read trains
  while(getline(file, s)) {
    int speed, length, depTime, from, to;
    std::stringstream tmp(s);
    tmp >> speed >> length >> depTime >> from >> to;
    Train train(trainId++, speed, length, depTime, g.getEdge(from, to));
    //read trainstops
    while(getline(file, s) && !s.empty()) {
      int arrivalTime, from, to;
      std::stringstream tmp2(s);
      tmp2 >> arrivalTime >> from >> to;
      train.addStop(arrivalTime, g.getEdge(from, to));
    }
    trains.push_back(train);
    if(file.eof()) break;
  }
  file.close();

  return new Instance(g, trains, maxTimeSteps, new context);
}

void printInstance(Instance* instance) {
    if(instance->solve()) {
      std::cout << "SAT" << std::endl;
      instance->printVSS();
      std::cout << std::endl;
      instance->printTrainRoute();
    } else {
      std::cout << "UNSAT" << std::endl;
    }
}

void printStatistics(Instance* instance) {
  if(instance->solve()) {
    std::cout << "SAT" << "\n";
    std::cout << "VSS: " << instance->num_vss() << "\n";
    std::cout << "MAKESPAN: " << instance->makespan() << "\n";
    std::cout << "Sum Times: " << instance->sum_times() << "\n";

  } else {
    std::cout << "UNSAT" << "\n";
  }
  std::cout << "CONSTRAINT TIME: " << instance->constraint_t << "\n";
  std::cout << "SOLVE TIME: " << instance->solve_t << "\n";
}

int main(int argc, char* argv[]) {
  if(argc < 2) {
    std::cout << "No file provided" << std::endl;
    std::exit(-1);
  }
  Instance* instance;
  if(argv[1][0] == '-') {
    if(argc < 3) {
      std::cout << "No file provided" << std::endl;
      std::exit(-1);
    }
    if(argv[1][1] == 'o') {
      instance = new OptimizeInstance(*parseInstance(argv[2]));
    } else if(argv[1][1] == 'f') {
      instance = new FixedVssInstance(*parseInstance(argv[2]));
    } else if(argv[1][1] == 's') {
      instance = new FixedScheduleInstance(*parseInstance(argv[2]));
    } else{
      std::cerr << "Unknown option" << std::endl;
      std::exit(-1);
    }
    //printInstance(instance);
    printStatistics(instance);
  } else {
    
    instance = new FixedVssInstance(*parseInstance(argv[1]));
    std::cout << "No Vss layout:" << std::endl;
    printInstance(instance);
    instance = new FixedScheduleInstance(*instance);
    std::cout << "Fixed Schedule:" << std::endl;
    printInstance(instance);
    instance = new OptimizeInstance(*instance);
    std::cout << "Optimize layout and schedule:" << std::endl;
    printInstance(instance);
  }
}


