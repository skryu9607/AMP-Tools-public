// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"
#include <fstream>
#include <vector>
#include <iostream>
#include <chrono>
//#include "Time.h"
//#include "../hw7/MySamplingBasedPlanners.h"

using namespace amp;
// Extend goal bias RRT
// Homogenous, 2D disk R
// Same speed, so same step.

// Inputs : a step size r, a goal bias probability p_goal, maximum number of iterations n, Workspace obstacles,
// workspace boundaries, m robots, and their corresponding initial position, and goal position,
// and epsilon for termination condition

// Output : a valid path for every robot from x_start_i to x_goal_i,
// the size of RRT, and the computation time. 


void timer_example() {
    double startTime;
    amp::Timer timer("timer");
    for (int i=0; i < 5; ++i) {
        startTime = timer.now(TimeUnit::ms);  
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time elapsed: " << Profiler::getTotalProfile("timer") << std::endl;
}

void benchmarkPlanner(size_t numAgents, size_t numRuns, double stepSize, double goalBiasProb, size_t maxIterations, double epsilon) {
    std::vector<double> computationTimes;
    std::vector<size_t> treeSizes;

    for (size_t i = 0; i < numRuns; ++i) {
        MultiAgentProblem2D problem = HW8::getWorkspace1(numAgents);
        
        MyCentralPlanner central_planner;
        auto start = std::chrono::high_resolution_clock::now();

        MultiAgentPath2D path = central_planner.plan(problem);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> computationTime = end - start;

        // computation time 기록
        computationTimes.push_back(computationTime.count());

        // RRT 트리의 노드 개수 기록
        treeSizes.push_back(central_planner.getNodePositionSize()); 
    }

    // 결과를 파일로 저장
    std::ofstream outFile("/Users/seung/MotionPlanning/AMP_new/AMP-Tools-public/benchmark_results_agents_" + std::to_string(numAgents) + ".csv");
    outFile << "ComputationTime,TreeSize\n";
    for (size_t i = 0; i < numRuns; ++i) {
        outFile << computationTimes[i] << "," << treeSizes[i] << "\n";
    }
    outFile.close();
    std::cout << "Benchmark for " << numAgents << " agents completed. Data saved to benchmark_results_agents_" << numAgents << ".csv\n";
}
void benchmarkDecentralizedPlanner(size_t numAgents, size_t numRuns, double stepSize, double goalBiasProb, size_t maxIterations, double epsilon) {
    std::vector<double> computationTimes;

    for (size_t i = 0; i < numRuns; ++i) {
        MultiAgentProblem2D problem = HW8::getWorkspace1(numAgents);
        
        MyDecentralPlanner decentralized_planner;
        auto start = std::chrono::high_resolution_clock::now();

        MultiAgentPath2D path = decentralized_planner.plan(problem);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> computationTime = end - start;

        // computation time 기록
        computationTimes.push_back(computationTime.count());
    }

    // 결과를 파일로 저장
    std::ofstream outFile("/Users/seung/MotionPlanning/AMP_new/AMP-Tools-public/decentralized_benchmark_results_agents_" + std::to_string(numAgents) + ".csv");
    outFile << "ComputationTime\n";
    for (size_t i = 0; i < numRuns; ++i) {
        outFile << computationTimes[i] << "\n";
    }
    outFile.close();
    std::cout << "Benchmark for " << numAgents << " agents completed. Data saved to decentralized_benchmark_results_agents_" << numAgents << ".csv\n";
}
int main(int argc, char** argv) {
    // Run timer example (useful for benchmarking)
    //timer_example();
    // 벤치마크 설정
    size_t numRuns = 100;       // 100회 실행
    double stepSize = 0.5;      // 스텝 사이즈
    double goalBiasProb = 0.05; // 목표 편향 확률
    size_t maxIterations = 10000; // 최대 반복 횟수
    double epsilon = 0.25;      // 목표 근접 허용 오차

    // // 에이전트 수를 3에서 6까지 변경하여 벤치마크 수행
    // for (size_t numAgents = 1; numAgents <= 6; ++numAgents) {
    //     benchmarkPlanner(numAgents, numRuns, stepSize, goalBiasProb, maxIterations, epsilon);
    // }
    // for (size_t numAgents = 1; numAgents <= 6; ++numAgents) {
    //     benchmarkDecentralizedPlanner(numAgents, numRuns, stepSize, goalBiasProb, maxIterations, epsilon);
    // }
    // Initialize Workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());
    // HW8:getWorkspace 1 and (2) < - number of agents. 
    // number of agents is defined in "the problem"
    MultiAgentProblem2D problem = HW8::getWorkspace1(2);
    
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    // Solve using a centralized approach
    MyCentralPlanner central_planner;
    MultiAgentPath2D path = central_planner.plan(problem);
    bool isValid = HW8::check(path, problem, collision_states);

    Visualizer::makeFigure(problem, path, collision_states);
    // Solve using a decentralized approach
    MyDecentralPlanner decentral_planner;
    MultiAgentPath2D path_de = decentral_planner.plan(problem);
    collision_states = {{}};
    HW8::generateAndCheck(decentral_planner, path_de, problem, collision_states);
    Visualizer::makeFigure(problem, path_de, collision_states);

    // Visualize and grade methods
    Visualizer::showFigures();
    HW8::grade<MyCentralPlanner, MyDecentralPlanner>("seungkeol.ryu@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}
