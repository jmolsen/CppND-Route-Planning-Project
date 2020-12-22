#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Calculate the heuristic.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Use straight line distance between noe and end_node
    return node->distance(*end_node);
}


// Expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // FindNeighbors will populate the current_node->neighbors vector with all the neighbors.
    current_node->FindNeighbors();
    current_node->visited = true;
    for (RouteModel::Node* neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = neighbor->distance(*current_node) + current_node->g_value;
        neighbor->visited = true;
        open_list.emplace_back(neighbor);
    }
}


// Sort the open_list according to the sum of the h value and g value and return the next node with the lowest sum.
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node* a, RouteModel::Node* b) {
        return (a->h_value + a->g_value) > (b->h_value + b->g_value);
    });
    RouteModel::Node* next = open_list.back();
    open_list.pop_back();
    return next;
}


// Return the final path found from the A* search.
// Update the distance variable with the total distance traveled.
// The returned vector will have the start node as the first element
// of the vector and the end node will be the last element.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.emplace_back(*current_node);
    while (current_node != start_node) {
        RouteModel::Node* parent_node = current_node->parent;
        distance += current_node->distance(*parent_node);
        path_found.insert(path_found.begin(), *parent_node);
        current_node = parent_node;
    }
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// A* Search algorithm
// Store the final path in the m_Model.path attribute. This path will then be displayed on the map tile.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    while (current_node != end_node) {
        // Add all of the neighbors of the current node to the open_list.
        AddNeighbors(current_node);
        // Sort the open_list and return the next node.
        current_node = NextNode();
    }
    // Current node is now end_node.
    m_Model.path = ConstructFinalPath(current_node);
}