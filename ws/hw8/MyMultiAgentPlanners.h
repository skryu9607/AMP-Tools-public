#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 


class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override; 
        size_t getNodePositionSize() const;
    private:
    std::unordered_map<int, Eigen::VectorXd> nodePositions;
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
        amp::Path2D rrtplan(
    const amp::Problem2D& problem, 
    const std::vector<std::vector<Eigen::Vector2d>>& dynamicObstacles,
    double minDist,size_t maxiterations);
        size_t getNodePositionSize() const;
    private:
    std::unordered_map<int, Eigen::VectorXd> nodePositions;
};
