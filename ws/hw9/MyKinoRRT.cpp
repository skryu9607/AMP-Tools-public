#include "MyKinoRRT.h"
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <random>
#include <utility>
#include <map>
#include <tuple>
#include <Eigen/Dense>


// Kinodynamic RRT
// 1. T < - create tree rooted at x0
// 2 .  while soultion not found do
// x_rand < - statesample()
// x_near < - nearest state in T to x_rand according to distance,rho
// generating lambda <- Generate local trajectory
//     "Generating local trajectory"
//     for i = 1, ... ,m do
//         u < - sample random control in U
//         lambda <- x(t) = x_near + Integration f over delt 
//         di < - rho (x_rand, lambda)
//     return lambda_i with minimum di
//     if IsSubTrejactoryValid(lambda, 0, step) then
//         x_new <- lambda(step)
//         add configuration x_new and edge(x_near,x_new) to T
//     if rho(x_new,x_goal) ~= 0 then
//         return solution trajectory from root to x_new.

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};
double MyKinoRRT::rho(const Eigen::VectorXd& point1, const Eigen::VectorXd& point2){
    return (point1-point2).norm();
}
Eigen::VectorXd MyKinoRRT::randomsample(const std::vector<std::pair<double, double>>& bounds) {
    static std::random_device rd;
    static std::mt19937 gen(rd());

    Eigen::VectorXd randomValues(bounds.size());
    for (size_t i = 0 ; i < bounds.size();i++){
        randomValues[i] = std::uniform_real_distribution<double> (bounds[i].first,bounds[i].second)(gen);
    }
    return randomValues;
}
std::multimap <int, std::tuple<std::vector<double>,double, double>> MyKinoRRT::GeneratingSample
(const amp::KinodynamicProblem2D& problem, size_t m, 
const Eigen::VectorXd& x_near, const Eigen::VectorXd& x_rand){
    size_t i;
    std::vector<std::pair<double, double>> UBounds = problem.u_bounds;
    std::vector<std::pair<double, double>> dtBounds = problem.dt_bounds;
    std::multimap <int, std::tuple<std::vector<double>,double,double >> SortedSampleChunck;
    for (i = 0; i <m; i++){
        std::vector<double> ControlInputs = randomsample(UBounds);
        double dtInputs = randomsample(dtBounds);
        x_new = propagate(x_near,ControlInputs, dtInputs);
        double distance = rho(x_new, x_rand);
        SortedSampleChunck.insert({distance, {ControlInputs, distance,dtInputs}});
    }
    return SortedSampleChunck;
    // The first one is the minimum distance. 
}

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    amp::KinoPath path;
    Eigen::VectorXd state = problem.q_init;
    path.waypoints.push_back(state);
    // <std::pair<double, double>> q_bounds, u_bounds, dt_bounds
    size_t maxIterations = 10000;size_t sampleNum = 10;
    for (int i = 0; i < maxIterations; i++) {
        Eigen::VectorXd control = Eigen::VectorXd::Random(problem.q_init.size());
        agent.propagate(state, control, 1.0);
        path.waypoints.push_back(state);
        // Compute the trajectory length
        path.controls.push_back(control);
        path.durations.push_back(1.0);
        // Summation of time duration
    }
    path.valid = true;
    return path;
}
