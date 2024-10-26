// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <chrono> 

using namespace amp;
void runBenchmarks() {
    // Define the different (p_goal, stepsize) combinations to test
    std::vector<std::pair<double, double>> benchmarks = {
        {0.05, 0.5}, {0.05, 1.0}, {0.1, 0.5}, {0.1, 1.0}
    };

    // Open a file to save the benchmark results
    std::ofstream resultsFile("/Users/seung/MotionPlanning/AMP_new/AMP-Tools-public/rrt_benchmark_results.csv");
    resultsFile << "p_goal,stepsize,run,computation_time,path_length,valid_solution\n";

    // Iterate over each (p_goal, stepsize) combination
    for (const auto& [n, r] : benchmarks) {
        std::cout << "Running RRT benchmarks for p_goal = " << n << ", stepsize = " << r << std::endl;

        // Repeat 100 times for each (p_goal, stepsize) combination
        for (int run = 0; run < 100; ++run) {
            Problem2D problem = HW5::getWorkspace1();
            MyRRT rrt;
            rrt.setParameters(n, r); 
            std::unordered_map<amp::Node, Eigen::Vector2d> nodePositions;

            // Measure computation time
            auto start_time = std::chrono::high_resolution_clock::now();
            //auto graphPtr = rrt.generateGoalBiasedRRTTree(problem, n, r, 5000, 0.25, nodePositions);
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> computation_duration = end_time - start_time;
            double computation_time = computation_duration.count();

            // Generate the path using RRT
            amp::Path2D path = rrt.plan(problem);
            std::cout << path.waypoints.size() << std::endl;

            // Check if the path is valid (non-empty)
            double path_length = 0.0;
            bool valid_solution = true;
            if (path.waypoints.size() ==2){
                valid_solution = false;
            }
            if (valid_solution) {
                // Calculate path length
                for (size_t i = 1; i < path.waypoints.size(); ++i) {
                    path_length += (path.waypoints[i] - path.waypoints[i - 1]).norm();
                }
            }

            // Save results to file
            resultsFile << n << "," << r << "," << run << "," << computation_time << ","
                        << path_length << "," << valid_solution << "\n";
        }
    }

    resultsFile.close();
    std::cout << "RRT Benchmarking completed. Results saved to rrt_benchmark_results.csv." << std::endl;
}
int main(int argc, char** argv) {
    HW7::hint(); // Consider implementing an N-dimensional planner 

    // Example of creating a graph and adding nodes for visualization
    // graph : std::shared_ptr<amp::Graph<double>>
    //std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    // // Node : std::amp<amp::Node, Eigen::Vector2d>.
    //std::map<amp::Node, Eigen::Vector2d> nodes;
    // // V : std::vector<Eigen::Vector2d>, Edges : std::vector<std::tuple <amp::Node,amp::Node,double>>
    // std::vector<Eigen::Vector2d> points = {{3, 3}, {4, 5}, {5, 3}, {6, 5}, {5, 7}, {7, 3}}; // Points to add to the graph
    // for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map
    // std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {{0, 4, 1}, {0, 5, 1}, {4, 5, 1}, {1, 2, 1}, {1, 3, 1}, {2, 3, 1}}; // Edges to connect
    // for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); // Connect the edges in the graph
    // graphPtr->print();
    // Test PRM on Workspace1 of HW2

    // Problem2D problem = HW5::getWorkspace1();
    // MyPRM prm;
    // std::unordered_map<amp::Node, Eigen::Vector2d> nodePositions; 
    // auto graphPtr = prm.generateRoadmap(problem, nodePositions); // nodePositions을 인자로 전달


    // std::map<amp::Node, Eigen::Vector2d> nodesPRM;
    // for (const auto& [node, position] : nodePositions) {
    //     nodesPRM[node] = position;
    // }
    // amp::Path2D path = prm.plan(problem);
    // Visualizer::makeFigure(problem, path, *graphPtr, nodesPRM);
    //graphPtr -> print();
    runBenchmarks();
    Problem2D problem = HW2::getWorkspace2();
    MyPRM prm;
    prm.setParameters(200, 2.5); // Example parameters
    std::unordered_map<amp::Node, Eigen::Vector2d> nodePositions;
    auto graphPtr = prm.generateRoadmap(problem, nodePositions);

    std::map<amp::Node, Eigen::Vector2d> nodesPRM;
    for (const auto& [node, position] : nodePositions) {
        nodesPRM[node] = position;
    }
    
    amp::Path2D path = prm.plan(problem);
    std::cout<<path.waypoints.size()<<std::endl;
    Visualizer::makeFigure(problem, path, *graphPtr, nodesPRM);


    // Generate a random problem and test RRT
    problem = HW5::getWorkspace1();
    MyRRT rrt;
    path = rrt.plan(problem);
    std::unordered_map<amp::Node, Eigen::Vector2d> nodePositionsRRT;
    double r = 0.50; // Step size
    double p_goal = 0.05; // Goal bias probability
    size_t maxIterations = 5000; // Maximum number of iterations
    double epsilon = 0.25;
    // Step 2: Generate the RRT tree and node positions separately
    graphPtr = rrt.generateGoalBiasedRRTTree(problem, r, p_goal, maxIterations, epsilon, nodePositionsRRT); // Renamed to rrtGraphPtr

    // Step 3: Generate node map for visualization
    std::map<amp::Node, Eigen::Vector2d> nodesRRT;
    for (const auto& [node, position] : nodePositionsRRT) {
        nodesRRT[node] = position;
    }
    // Step 4: Visualize the RRT graph
    Visualizer::makeFigure(problem, path , *graphPtr, nodesRRT);
    graphPtr ->print();
    Visualizer::showFigures();
    // Define the `path` variable by calling `plan()`
    
    HW7::generateAndCheck(rrt, path, problem);
    // Grade method
    HW7::grade<MyPRM, MyRRT>("SeungKeol.Ryu@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}
