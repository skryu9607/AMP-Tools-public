#include "MyAStar.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>

// NodeInfo 구조체: 각 노드의 경로 정보

// 경로 결과를 초기화하는 함수
MyAStarAlgo::GraphSearchResult initialize_result() {
    return {false, {}, 0.0};  // 초기화된 결과 반환
}

// 초기 노드 설정 함수
void MyAStarAlgo::initialize_start_node(int start_node, const amp::SearchHeuristic& heuristic, 
                           std::unordered_map<int, NodeInfo>& node_info, 
                           std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>>& open_set, 
                           int goal_node) {
    node_info[start_node] = {0.0, heuristic(start_node), -1};  // 시작 노드 설정
    open_set.push({node_info[start_node].f_cost, start_node});  // 시작 노드를 우선순위 큐에 추가
}

// 경로 재구성 함수: 목표 노드에서 시작 노드까지 경로 추적
void MyAStarAlgo::reconstruct_path(int current_node, const std::unordered_map<int, NodeInfo>& node_info, 
                      MyAStarAlgo::GraphSearchResult& result, int start_node) {
    while (current_node != start_node) {
        result.node_path.push_back(current_node);
        current_node = node_info.at(current_node).parent;  // 부모 노드를 따라 경로 추적
    }
    result.node_path.push_back(start_node);  // 시작 노드 추가
    std::reverse(result.node_path.begin(), result.node_path.end());  // 경로를 반전시켜 올바른 순서로 정렬
}

// A* 알고리즘 구현
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Search: " << problem.init_node << " -> " << problem.goal_node << std::endl;

    // 결과 객체 초기화
    GraphSearchResult result = initialize_result();
    
    // 우선순위 큐 및 해시맵 선언
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> open_set;
    std::unordered_map<int, NodeInfo> node_info;  // 각 노드의 비용 정보 저장
    std::unordered_set<int> closed_set;           // 탐색 완료된 노드 추적
    
    // 시작 노드 설정
    initialize_start_node(problem.init_node, heuristic, node_info, open_set, problem.goal_node);
    int i = 0;
    // A* 탐색 루프
    while (!open_set.empty()) {
        i += 1;
        // 우선순위 큐에서 가장 비용이 낮은 노드 선택
        auto [current_f_cost, current_node] = open_set.top();
        open_set.pop();

        // 목표 노드에 도달했을 때 경로를 재구성
        if (current_node == problem.goal_node) {
            result.success = true;
            result.path_cost = node_info[current_node].g_cost;
            reconstruct_path(current_node, node_info, result, problem.init_node);
            result.print();
            return result;
        }

        // 현재 노드를 closed set에 추가
        closed_set.insert(current_node);

        // 현재 노드의 이웃 노드 탐색
        const auto& neighbors = problem.graph->children(current_node);
        const auto& edge_costs = problem.graph->outgoingEdges(current_node);
        for (std::size_t i = 0; i < neighbors.size(); ++i) {
            int neighbor = neighbors[i];
            double edge_cost = edge_costs[i];

            // 이미 탐색된 노드는 건너뜀
            if (closed_set.count(neighbor)) continue;

            // 이웃 노드로 가는 비용 계산
            double tentative_g_cost = node_info[current_node].g_cost + edge_cost;

            // 더 나은 경로가 발견되었거나, 아직 방문되지 않은 노드라면
            if (!node_info.count(neighbor) || tentative_g_cost < node_info[neighbor].g_cost) {
                // 노드 정보 갱신 및 우선순위 큐에 추가
                node_info[neighbor] = {tentative_g_cost, tentative_g_cost + 0, current_node};
                open_set.push({node_info[neighbor].f_cost, neighbor});
            }

        }
        std::cout << "Num of iterations : " << i << std::endl;
    }

    // 경로를 찾지 못한 경우
    std::cout << "Path not found!" << std::endl;
    
    result.print();
    return result;  // 실패 결과 반환
}
