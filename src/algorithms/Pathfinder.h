// algorithms/Pathfinder.h

// src/algorithms/Pathfinder.h

#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "data_structures/Node.h" 
#include <vector>
#include <queue> 
#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>

class Pathfinder {
private:
    // General Helpers
    std::vector<std::pair<int, int>> reconstruct_path(Node* goal);
    void cleanup_nodes(std::vector<Node*>& nodes);

    // Dijkstra's/A* Search Core
    std::vector<std::pair<int, int>> search_weighted(Node* start, Node* goal, const WeightGrid& weights);

    // BFS Search Core
    std::vector<std::pair<int, int>> search_unweighted(Node* start, Node* goal, const Grid& grid);

public:
    // Public API for Dijkstra's (requires a weight grid)
    std::vector<std::pair<int, int>> dijkstra(Node* start, Node* goal, const WeightGrid& weights);
    
    // Public API for Breadth-First Search (requires a simple grid for walls)
    std::vector<std::pair<int, int>> bfs(Node* start, Node* goal, const Grid& grid);
};

#endif // PATHFINDER_H