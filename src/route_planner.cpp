#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // By using the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Storing the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);
    
}


// Implementing the CalculateHValue method.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Completed the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    //Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();

    // - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
    for(auto i : current_node->neighbors)
    {
        i->parent = current_node ;
        i->h_value = CalculateHValue(i);
        i->g_value = current_node->g_value + current_node->distance(*i);
    
    // - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
        open_list.push_back(i);
        i->visited = true ;
    }

}


RouteModel::Node *RoutePlanner::NextNode() {
  // - Sort the open_list according to the sum of the h value and g value.
  sort(open_list.begin(), open_list.end(),
    // Lambda comparator function
       [](auto const &a, auto const &b) {
         return (a->g_value + a->h_value) > (b->g_value + b->h_value);
       });

  // Create a pointer to the node in the list with the lowest sum.
  auto node = open_list.back();

  // Remove that node from the open_list.
  open_list.pop_back();

  // Return the pointer.
  return node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while (current_node->parent != nullptr) 
    {
    path_found.push_back(*current_node);
    distance += current_node->distance(*current_node->parent);
    current_node = current_node->parent;
    }
    path_found.push_back(*current_node);

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// the A* Search algorithm 


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    start_node->visited = true ;
    open_list.push_back(start_node);

    while (open_list.size() > 0)
    {
        current_node = NextNode();

        if(current_node->distance(*end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(current_node);

            return ;

        }
        else{
            AddNeighbors(current_node);
        }
    }
     
}