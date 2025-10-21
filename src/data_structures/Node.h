// src/data_structures/Node.h

#ifndef NODE_H
#define NODE_H

#include <cmath>
#include <limits>
#include <vector>
#include <utility>
#include <algorithm> 

struct Node {
    int x, y;
    
    // Dijkstra's/A* Costs
    double g_cost; 
    double h_cost; 
    double f_cost; 
    
    // BFS/General Status
    bool visited; 
    
    Node* parent;

    Node(int _x, int _y) 
        : x(_x), y(_y), 
          g_cost(std::numeric_limits<double>::infinity()), 
          h_cost(0.0), 
          f_cost(std::numeric_limits<double>::infinity()), 
          visited(false),
          parent(nullptr) {}
};

// Comparator for Dijkstra's* Min-Priority Queue
struct CompareNode {
    bool operator()(const Node* a, const Node* b) const {
        return a->f_cost > b->f_cost; 
    }
};

using Grid = std::vector<std::vector<int>>; 
using WeightGrid = std::vector<std::vector<int>>; 

#endif // NODE_H