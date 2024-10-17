#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW6.h"
#include <queue>
class MyAStarAlgo : public amp::AStar {
    public:
        struct NodeInfo {
        double g_cost;      // 시작점에서 현재 노드까지의 실제 비용
        double f_cost;      // 총 비용 (g + h)
        int parent;         // 경로 재구성을 위한 부모 노드
    };
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
    void initialize_start_node(int start_node, const amp::SearchHeuristic& heuristic, 
                           std::unordered_map<int, NodeInfo>& node_info, 
                           std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>>& open_set, 
                           int goal_node);
    void reconstruct_path(int current_node, const std::unordered_map<int, NodeInfo>& node_info, 
                      MyAStarAlgo::GraphSearchResult& result, int start_node);
};
