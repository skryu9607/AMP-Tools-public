#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

class MyPRM : public amp::PRM2D {
    public:
        MyPRM() : numSamples(500), connectionRadius(1.5) {}

        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        double Random(double min_val, double max_val);
        void setParameters(size_t n, double r) {
        size_t numSamples = n;
        double connectionRadius = r;
    }
        //bool doLineSegmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                           //  const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
        //bool lineCollision(const Eigen::Vector2d& seg_p1, const Eigen::Vector2d& seg_p2,
            //const amp::Polygon& polygon);
        //double crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b);
        std::vector<amp::Node> aStar(std::shared_ptr<amp::Graph<double>>& graphPtr, amp::Node startNode, amp::Node goalNode, const std::unordered_map<amp::Node, Eigen::Vector2d>& nodePositions);
        double heuristic(const Eigen::Vector2d& a, const Eigen::Vector2d& b);
        double getEdgeWeight(std::shared_ptr<amp::Graph<double>>& graphPtr, amp::Node src, amp::Node dst);
        std::shared_ptr<amp::Graph<double>> generateRoadmap(const amp::Problem2D& problem,std::unordered_map<amp::Node, Eigen::Vector2d>& nodePositions);
    private: 
    size_t numSamples; 
    double connectionRadius; 
};

class MyRRT : public amp::GoalBiasRRT2D {
private:
    std::unordered_map<int, Eigen::Vector2d> nodes; // Stores node ID and its position
    std::vector<std::pair<int, int>> edges; // Stores pairs of connected node IDs
    double p_goal;  
    double stepsize; 

public:
    // Constructor
    MyRRT() : p_goal(0.05), stepsize(0.5) {}

    // Set parameters
    void setParameters(double p, double r) {
        p_goal = p; 
        stepsize = r; 
    }

    void addNode(int nodeId, const Eigen::Vector2d& position);
    const std::unordered_map<int, Eigen::Vector2d>& getNodes() const;
    const Eigen::Vector2d& getNode(int nodeId) const;
    void addEdge(int srcId, int dstId);
    std::vector<std::pair<amp::Node, amp::Node>> getEdges(const std::shared_ptr<amp::Graph<double>>& graphPtr) const;

    std::shared_ptr<amp::Graph<double>> generateGoalBiasedRRTTree(
        const amp::Problem2D& problem, 
        double r, double p_goal, size_t maxIterations, double epsilon, 
        std::unordered_map<amp::Node, Eigen::Vector2d>& nodePositions);
    virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
};
