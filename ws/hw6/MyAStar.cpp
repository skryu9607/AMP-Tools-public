#include "MyAStar.h"

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object
    result.node_path.push_back(problem.init_node);
    result.node_path.push_back(problem.goal_node);
    result.path_cost += 1.0;
    amp::Node currNode = 0;
    for(auto child : problem.graph->children(currNode)){
        double h = heuristics(0);
        std::cout << child <<std::endl;
    }
    for (int i = 0; i <problem.graph->children(currNode).size();i++){
        std::cout <<< " The cost of edge " << currNode << "-" 
        <<problem.graph->children(currNode)[i]<<"is" << problem.graph->outgoingEdges(currNode)[i] <<std::endl;

    }
    while(!result.success){
        result.success = true;
    }

    // Only Node.path is valid, the cost is calculated 
    result.print();
    return result;
}
