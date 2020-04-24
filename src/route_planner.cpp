#include "route_planner.h"
#include <algorithm>

using std::sort;
using std::vector;
using std::reverse;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}

// Calculate the manhattan distance
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

// Function to expand the current node by adding all unvisited neighbors to the open list
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
  current_node->FindNeighbors();

  for (auto neighbour : current_node->neighbors) {
    neighbour->parent = current_node;
    neighbour->h_value = CalculateHValue(neighbour);
    neighbour->g_value = current_node->g_value + current_node->distance(*neighbour);

    open_list.push_back(neighbour);
    neighbour->visited = true;
  }
}

// Function to sort the open list and return the next node
RouteModel::Node *RoutePlanner::NextNode() {
  
  sort(open_list.begin(), open_list.end(), [] (const RouteModel::Node * a, const RouteModel::Node * b) {
    // Compare the F values of two nodes
    float f1 = a->g_value + a->h_value; // f1 = g1 + h1
    float f2 = b->g_value + b->h_value; // f2 = g2 + h2
    return f1 > f2; 
  });

  auto next_node = open_list.back();
  open_list.pop_back();

  return next_node;
}

// Function to return the final path found from A* search*/
vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  vector<RouteModel::Node> path_found;
  RouteModel::Node parent_node;

  // Iterate threw the chain of parents of nodes until the starting node is found (parent == nullptr)
  while (current_node->parent != nullptr) {
    path_found.push_back(*current_node);
    parent_node = *(current_node->parent);
    distance += current_node->distance(parent_node);
    current_node = current_node->parent;
  }

  // Add last node (init point) to the path
  path_found.push_back(*current_node);

  // Reverse vector to correct nodes order
  reverse(path_found.begin(), path_found.end());

  // Multiply the distance by the scale of the map to get meters
  distance *= m_Model.MetricScale();

  return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

}