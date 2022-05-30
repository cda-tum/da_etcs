#include "../include/graph.h"
#include <queue>
#include <set>
#include <iostream>
#include <algorithm>

Graph::Graph(int vertexCount): adjList(vertexCount), isBoundaryList(vertexCount, false), vertexCount(vertexCount){}

Graph::Graph(): adjList(), isBoundaryList(), vertexCount(0) {}

const Edge& Graph::addEdge(int from, int to, Edge* edge) {
  for(Edge* neighbour: adjList[from]) {
    edge->addNeighbour(neighbour)->addNeighbour(edge);
  }
  for(Edge* neighbour: adjList[to]) {
    edge->addNeighbour(neighbour)->addNeighbour(edge);
  }
  adjList[from].push_back(edge);
  adjList[to].push_back(edge);
  edges.push_back(edge);
  ttds[edge->ttd].insert(edge);
  return *adjList[from].back();
}

const Edge& Graph::addEdge(int from, int to, int length, int maxSpeed, int ttd) {
  static int edgeId = 0;
  Edge *newEdge = new Edge(edgeId++, from, to, length, ttd, maxSpeed);
  return addEdge(from, to, newEdge);
}

void Graph::addNode(bool isBoundary) {
  isBoundaryList.push_back(isBoundary);
  adjList.push_back(std::vector<Edge*>());
  vertexCount++;
}

void Graph::addNode() {
  addNode(false);  
}

void Graph::setBoundary(int node) {
  isBoundaryList[node] = true;
}

void Graph::setBoundaries() {
  for(int node = 0; node < vertexCount; node++) {
    if(adjList[node].size() != 2 || adjList[node][0]->ttd != adjList[node][1]->ttd ) {
      isBoundaryList[node] = true;
    }
    
  }
  
}
  
bool Graph::isBoundary(int node) const {
  return isBoundaryList[node];  
}

const Edge* Graph::getEdge(int from, int to) {
  for(size_t i = 0; i < adjList.size(); i++) {
    if(adjList[from][i]->connectsTo(to)) {
      return (adjList[from][i]);
    }
  }
  throw "Edge not found!";
}

void Graph::prohibitTransition(int left, int mid, int right) {
  Edge* leftEdge;
  Edge* rightEdge;
  for(Edge* edge: adjList[mid]) {
    if(edge->connectsTo(left)) leftEdge = edge;
    if(edge->connectsTo(right)) rightEdge = edge;
  }
  leftEdge->neighbours.erase(std::find(leftEdge->neighbours.begin(), leftEdge->neighbours.end(), rightEdge));
  rightEdge->neighbours.erase(std::find(rightEdge->neighbours.begin(), rightEdge->neighbours.end(), leftEdge));
}


EdgeSet Graph::reachableEdges(int from, int to, int speed) {
  const Edge* start = getEdge(from, to);
  return reachableEdges(start, speed);
}

EdgeSet Graph::reachableEdges(const Edge* start, int speed) {
  auto cacheEntry = reachableCache.find({speed, start});
  if(cacheEntry != reachableCache.end()) {
    return cacheEntry->second;
  }
  
  EdgeSet reachable;
  std::priority_queue<std::pair<int, const Edge*>> toVisit;
  toVisit.push({speed, start});
  
  while(toVisit.size() > 0) {
    std::pair<int, const Edge*> currentPair = toVisit.top();
    toVisit.pop();
    int currentLength = currentPair.first;
    const Edge* currentEdge = currentPair.second;
    currentLength = std::min(currentLength, currentEdge->maxSpeed);
    
    reachable.insert(currentEdge);
    
    if(currentLength <= 0)
      continue;

    for(const Edge* neighbour: currentEdge->neighbours) {
      if(reachable.find(neighbour) == reachable.end()) {
        toVisit.push(std::make_pair(currentLength - currentEdge->length, neighbour));
      }
    }
  }
  reachableCache.insert({{speed, start}, reachable});
  return reachable;
}

//returns index of farthest edge that can be reached with speed
size_t Graph::reachableInPath(const pathType& path, size_t startIndex, int speed) {
  while(speed > 0 && startIndex < path.size()-1) {
    speed = std::min(speed, path[startIndex]->maxSpeed);
    speed -= path[startIndex++]->length;
  }
  return startIndex;
}


std::vector<std::vector<const Edge*>> Graph::paths(const Edge* start, const Edge* stop) {
  std::vector<std::vector<const Edge*>> paths;
  std::vector<std::pair<int, const Edge*>> dfsStack; //store depth at which edge has been found to know how often we need to backtrack
  dfsStack.push_back({0, start});
  std::vector<const Edge*> currPath;
  int dfsDepth = 0;
 
  while(dfsStack.size() > 0) {
    auto [curDepth, currEdge] = dfsStack.back();
    dfsStack.pop_back();
    while(dfsDepth > curDepth) {
      currPath.pop_back();
      dfsDepth--;
    }
    dfsDepth++;

    int sharedNode = -1;
    if(currPath.size() > 0) {
      const Edge* prev = currPath.back();
      sharedNode = currEdge->sharedVertex(prev);
      
    }
    currPath.push_back(currEdge);
    
    if(currEdge == stop) { //finnished path
      std::vector<const Edge*> finishedPath(currPath);
      paths.push_back(finishedPath);
      continue;
    }

    for(const Edge* neighbour: currEdge->neighbours) {
      bool onPath = false;

      //cant make sudden turns in a fork
      if(neighbour->connectsTo(sharedNode)) {
        continue;
      }
      
      for(const Edge* pathEdge: currPath) {
        if(pathEdge == neighbour) {
          onPath = true;
          break;
        }
      }
      if(onPath)
        continue;
      
      dfsStack.push_back({dfsDepth, neighbour});
    }
  }
  return paths;
}

std::vector<std::vector<const Edge*>> Graph::chainsOfLength(int length) {
  auto cacheEntry = chainCache.find(length);
  if(cacheEntry != chainCache.end()) {
    return cacheEntry->second;
  }
  
  std::vector<std::vector<const Edge*>> chains;
  for(const Edge* start: edges) {
    std::vector<std::pair<int, std::vector<const Edge*>>> dfsStack;
    std::vector<const Edge*> startChain;
    startChain.push_back(start);
    dfsStack.push_back({length-start->length, startChain});
    while(! dfsStack.empty()) {
      auto [chainLength, chain] = dfsStack.back();
      dfsStack.pop_back();
      if(chainLength <= 0) {
        chains.push_back(chain);
        continue;
      }
    
      for(const Edge* neighbour: chain.back()->neighbours) {
        std::vector<const Edge*> newChain(chain);
        newChain.push_back(neighbour);
        dfsStack.push_back({chainLength - neighbour->length, newChain});
      }
    }
  }
  chainCache.insert({length, chains});
  return chains;
}

std::vector<const Edge *> Graph::getTTD(int ttd) {
  EdgeSet ttdEdgesUnordered = ttds[ttd];
  std::vector<const Edge *> ttdOrdered;
  const Edge *currEdge;
  int currNode = -1;
  for (const Edge *edge : ttdEdgesUnordered) {
    if (isBoundary(edge->from)) {
      currEdge = edge;
      currNode = edge->from;
      break;
    } else if (isBoundary(edge->to)) {
      currEdge = edge;
      currNode = edge->to;
      break;
    }
  }

  ttdOrdered.push_back(currEdge);
  while (!isBoundary((currNode = currEdge->getOtherVertex(currNode)))) {
    for (const Edge *neighbour : currEdge->neighbours) {
      if (neighbour->connectsTo(currNode)) {
        currEdge = neighbour;
        ttdOrdered.push_back(neighbour);

        break;
      }
    }
  }
  return ttdOrdered;
}
