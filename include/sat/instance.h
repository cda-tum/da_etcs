#ifndef INSTANCE_H
#define INSTANCE_H

#include "z3++.h"
#include "./graph.h"
#include "./train.h"
#include <vector>

using namespace z3;

typedef std::vector<std::vector<expr_vector>> OccupiedType;
typedef std::vector<int> vssType;
typedef std::vector<vssType> vssListType;
typedef std::vector<const Edge*> trainPathType;


class Instance {
 public:
  Instance(Graph g, std::vector<Train> trains, uint32_t maxTimeSteps, context* c);
  Instance(const Instance& otherInstance);

  void printVSS();
  uint32_t num_vss();
  vssListType getVSS(int ttd);
  bool solve();
  void printTrainRoute();
  uint32_t makespan();
  uint32_t sum_times();

  double constraint_t = 0;
  double solve_t = 0;
  
 protected:

  Graph graph;
  std::vector<Train> trains;
  uint32_t maxTimeSteps;
  context* c;
  optimize solver;
  //mapping train -> time -> edge -> variable
  OccupiedType occupiedVars;
  //mapping node -> variable
  expr_vector borderVars;

  std::vector<std::vector<pathType>> trainPaths;
  std::vector<expr> constraints;
  
  void computeConstraints();
  void computeLengthConstraint(const Train& train);
  void computeVssConstraint(const Train& train, const Edge* edge);
  void computeCollisionConstraint(const Train& train);
  void computeReachableConstraint(const Train& train);
  virtual void enforceConstraints();
  
  
  virtual void enforceBoundaryConstraints();
  virtual void enforceStopConstraint();
  void enforceReachableConstraint(const Train& train);
  void enforceCollisionConstraint(const Train& train, const Edge* edge, const EdgeSet& reachable);
  void enforceCollisionConstraint(const Train& train);
  void enforceLengthConstraint(const Train& train);
  
  void minimizeVss();
  expr sumOfBoolVec(const expr_vector& bools);
};

class FixedVssInstance: public Instance {
  using Instance::Instance;
 public:
  FixedVssInstance(const Instance& otherInstance);
 private:
 void enforceBoundaryConstraints();
};

class FixedScheduleInstance: public Instance {
  using Instance::Instance;
  
 public:
  FixedScheduleInstance(const Instance& otherInstance);

 private:
  void enforceStopConstraint();
};

class OptimizeInstance: public Instance {
  using Instance::Instance;

 public:
  OptimizeInstance(const Instance& otherInstance);

 private:
  void enforceConstraints();
  void enforceStopConstraint();
  void minimizeTrainSchedule();
};
#endif

