// src/main.cpp

#include "algorithms/Pathfinder.h" 
#include <iostream>
#include <vector>
#include <string>

// Helper function to display the path
void print_path(const std::vector<std::pair<int, int>>& path, const std::string& name, bool weighted) {
    std::cout << "-------------------------------------------\n";
    std::cout << "           " << name << " Path " << (weighted ? "(Weighted)" : "(Unweighted)") << "\n"; 
    std::cout << "-------------------------------------------\n";
    
    if (path.empty()) {
        std::cout << "Path not found.\n";
        return;
    }
    
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << "(" << path[i].first << ", " << path[i].second << ")";
        if (i < path.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << "\nTotal steps (path length): " << path.size() - 1 << "\n";
}

int main() {
    // 1. Define the GRID and WEIGHTS
    
    // Simple Grid for BFS (1 = open, 0 = wall)
    Grid unweighted_grid = {
        {1, 1, 1, 0, 1},
        {1, 0, 1, 0, 1},
        {1, 1, 1, 1, 1},
        {0, 0, 0, 1, 1},
        {1, 1, 1, 1, 1}
    };
    
    // Weighted Grid for Dijkstra's (Value = traversal cost, 0 = wall)
    // Note: Path through (0,1) is expensive (10), path through (1,2) is cheap (1)
    // 1 - You can go, 0 can not go same logic as BFS 
    WeightGrid weighted_grid = {
        {1, 10, 1, 0, 1},
        {1, 0, 1, 0, 1},
        {1, 1, 5, 1, 1},
        {0, 0, 0, 1, 1},
        {1, 1, 1, 1, 1}
    };


    // 2. Define Start (0, 0) and Goal (4, 4) coordinates
    Node start_coords(0, 0); 
    Node goal_coords(4, 4); 

    Pathfinder pf;

    // --- BFS Search Example (Unweighted) ---
    std::vector<std::pair<int, int>> bfs_path = pf.bfs(&start_coords, &goal_coords, unweighted_grid);
    print_path(bfs_path, "BFS", false);

    // --- Dijkstra's Algorithm Example (Weighted) ---
    // Dijkstra's will find the cheapest path, not just the shortest number of steps.
    Node dijkstra_start(0, 0); 
    Node dijkstra_goal(4, 4); 
    std::vector<std::pair<int, int>> dijkstra_path = pf.dijkstra(&dijkstra_start, &dijkstra_goal, weighted_grid);
    print_path(dijkstra_path, "Dijkstra's", true);

    return 0;
}
