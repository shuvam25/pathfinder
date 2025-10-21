// src/algorithms/Pathfinder.cpp

#include "Pathfinder.h" 

// --- General Helpers ---

std::vector<std::pair<int, int>> Pathfinder::reconstruct_path(Node* goal) {
    std::vector<std::pair<int, int>> path;
    Node* current = goal;
    while (current != nullptr) {
        path.push_back({current->x, current->y});
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void Pathfinder::cleanup_nodes(std::vector<Node*>& nodes) {
    for (Node* node : nodes) {
        delete node;
    }
    nodes.clear();
}

// --- BFS Implementation (Unweighted Search) ---

std::vector<std::pair<int, int>> Pathfinder::search_unweighted(Node* start, Node* goal, const Grid& grid) {
    // Initialization setup (similar to previous BFS code, omitted for brevity but required)
    
    // NOTE: This implementation relies on the helper functions and node management 
    // defined in the previous complete BFS code. 
    // Key components are: std::queue, visited=true check, and parent tracking.

    // ... (Code from the previous complete BFS solution goes here)
    
    // --- TEMPORARY placeholder to make it compile with the provided Node.h ---
    int rows = grid.size();
    int cols = grid[0].size();
    std::vector<std::vector<Node*>> all_nodes(rows, std::vector<Node*>(cols, nullptr));
    std::vector<Node*> nodes_to_clean;

    auto get_node = [&](int x, int y) -> Node* {
        if (x < 0 || x >= rows || y < 0 || y >= cols || grid[x][y] == 0) return nullptr;
        
        if (all_nodes[x][y] == nullptr) {
            all_nodes[x][y] = new Node(x, y);
            nodes_to_clean.push_back(all_nodes[x][y]);
        }
        return all_nodes[x][y];
    };

    Node* actual_start = get_node(start->x, start->y);
    Node* actual_goal = get_node(goal->x, goal->y);

    if (!actual_start || !actual_goal) { cleanup_nodes(nodes_to_clean); return {}; }
    
    std::queue<Node*> frontier; 
    frontier.push(actual_start);
    actual_start->visited = true;

    while (!frontier.empty()) {
        Node* current = frontier.front();
        frontier.pop();

        if (current == actual_goal) {
            std::vector<std::pair<int, int>> path = reconstruct_path(current);
            cleanup_nodes(nodes_to_clean);
            return path;
        }

        int dx[] = {0, 0, 1, -1};
        int dy[] = {1, -1, 0, 0};

        for (int i = 0; i < 4; ++i) {
            Node* neighbor = get_node(current->x + dx[i], current->y + dy[i]);
            if (neighbor && !neighbor->visited) {
                neighbor->visited = true;
                neighbor->parent = current;
                frontier.push(neighbor);
            }
        }
    }
    
    cleanup_nodes(nodes_to_clean);
    return {};
}

// --- Dijkstra's Implementation (Weighted Search) ---

std::vector<std::pair<int, int>> Pathfinder::search_weighted(Node* start, Node* goal, const WeightGrid& weights) {
    if (weights.empty()) return {};

    int rows = weights.size();
    int cols = weights[0].size();
    
    // Node storage mechanism
    std::vector<std::vector<Node*>> all_nodes(rows, std::vector<Node*>(cols, nullptr));
    std::vector<Node*> nodes_to_clean;

    auto get_node = [&](int x, int y) -> Node* {
        // Check boundaries and if it's a blocked path (weight 0)
        if (x < 0 || x >= rows || y < 0 || y >= cols || weights[x][y] == 0) return nullptr;
        
        if (all_nodes[x][y] == nullptr) {
            all_nodes[x][y] = new Node(x, y);
            nodes_to_clean.push_back(all_nodes[x][y]);
        }
        return all_nodes[x][y];
    };

    Node* actual_start = get_node(start->x, start->y);
    Node* actual_goal = get_node(goal->x, goal->y);

    if (!actual_start || !actual_goal) { cleanup_nodes(nodes_to_clean); return {}; }

    // 1. Initialization
    actual_start->g_cost = 0;
    actual_start->f_cost = 0; // f=g for Dijkstra's (h=0)

    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_set;
    open_set.push(actual_start);

    // 2. Main Loop
    while (!open_set.empty()) {
        Node* current = open_set.top();
        open_set.pop();

        if (current == actual_goal) {
            std::vector<std::pair<int, int>> path = reconstruct_path(current);
            cleanup_nodes(nodes_to_clean);
            return path;
        }

        // Neighbors: 4-way movement
        int dx[] = {0, 0, 1, -1};
        int dy[] = {1, -1, 0, 0};

        for (int i = 0; i < 4; ++i) {
            int next_x = current->x + dx[i];
            int next_y = current->y + dy[i];

            Node* neighbor = get_node(next_x, next_y);
            
            if (neighbor) {
                double edge_weight = (double)weights[next_x][next_y];
                double new_g_cost = current->g_cost + edge_weight;

                // Relaxation Step
                if (new_g_cost < neighbor->g_cost) {
                    neighbor->parent = current;
                    neighbor->g_cost = new_g_cost;
                    neighbor->f_cost = new_g_cost; // f = g + 0 (h=0)
                    open_set.push(neighbor);
                }
            }
        }
    }
    
    cleanup_nodes(nodes_to_clean);
    return {}; 
}

// --- Public API ---

std::vector<std::pair<int, int>> Pathfinder::dijkstra(Node* start, Node* goal, const WeightGrid& weights) {
    return search_weighted(start, goal, weights);
}

std::vector<std::pair<int, int>> Pathfinder::bfs(Node* start, Node* goal, const Grid& grid) {
    return search_unweighted(start, goal, grid);
}