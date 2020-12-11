#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto n : current_node->neighbors) {
        n->parent = current_node;
        n->h_value = CalculateHValue(n);
        n->g_value = current_node->g_value + current_node->distance(*n);
        n->visited = true;
        open_list.emplace_back(n);
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), RoutePlanner::Compare);
    RouteModel::Node * node = open_list.back();
    open_list.pop_back();
    return node;
}

bool RoutePlanner::Compare(RouteModel::Node * n1, RouteModel::Node * n2) {
    float f1 = n1->h_value + n1->g_value;
    float f2 = n2->h_value + n2->g_value;

    return f1 > f2;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    
    path_found.emplace_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    start_node->h_value = CalculateHValue(start_node);
    open_list.emplace_back(start_node);

    while (open_list.size() > 0) {
        current_node = NextNode();
        if (current_node->distance(*end_node) == 0) break;
        AddNeighbors(current_node);
    }

    m_Model.path = ConstructFinalPath(current_node);
}