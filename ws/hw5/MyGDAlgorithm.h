#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
		double dist (Eigen::Vector2d q, Eigen::Vector2d q_heading) const;
		double dist2 (Eigen::Vector2d q, Eigen::Vector2d q_heading) const;
		double U_att(Eigen::Vector2d q, Eigen::Vector2d q_heading) const;
		double pointToSegmentDistance(const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& P) const;
		double disti(const amp::Polygon obstacle, const Eigen::Vector2d& P) const ;
		double U_rep1(Eigen::Vector2d q, amp::Problem2D problem) const;
		Eigen::Vector2d gradient(Eigen::Vector2d q, amp::Problem2D problem) const;
		std::vector<int> getNearestObstacleIndices(Eigen::Vector2d pos, amp::Problem2D problem, int k) const;
 
	private:
		double d_star, zetta, Q_star, eta;
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
	// A constructor that takes a MyGDAlgorithm object by reference
    	MyPotentialFunction(const MyGDAlgorithm& gd_algorithm,amp::Problem2D& problem) : gd_algorithm(gd_algorithm),problem(problem) 
		{}
		
		// Returns the potential function value (height) for a given 2D point. 
        virtual double operator()(const Eigen::Vector2d& q) const override {
        return gd_algorithm.U_rep1(q, problem) + gd_algorithm.U_att(q, problem.q_goal);
        }

		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const {
			Eigen::Vector2d gradient_q = gd_algorithm.gradient(q,problem);
            return gradient_q;
		}

	private:
		const MyGDAlgorithm& gd_algorithm;
		amp::Problem2D& problem; 
		
};
