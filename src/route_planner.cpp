#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto *neighbor: current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = this->CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);

        current_node->visited = true;
        neighbor->visited = true;
        this->open_list.push_back(neighbor);
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(),
            // Lambda expression begins
         [](RouteModel::Node *a, RouteModel::Node *b) {
             return ((a->g_value + a->h_value) > (b->g_value + b->h_value));
         } // end of lambda expression
    );

    auto current = open_list.back();
    open_list.pop_back();

    return current;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node != this->start_node) {
        this->distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }

    path_found.push_back(*(this->start_node));
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end());
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;

    // TODO: Implement your solution here.
    while (current_node->x != this->end_node->x && current_node->y != this->end_node->y) {
        RoutePlanner::AddNeighbors(current_node);
        current_node = RoutePlanner::NextNode();
    }

    if (current_node->x == this->end_node->x && current_node->y == this->end_node->y) {
        std::cout << "end node: " << current_node->x << " " << current_node->y;
        this->m_Model.path = RoutePlanner::ConstructFinalPath(current_node);
    }

}