#include "MyGDAlgorithm.h"
#include <random>
// Implement your plan method here, similar to HW2:

double MyGDAlgorithm::dist (Eigen::Vector2d q, Eigen::Vector2d q_heading) const{
    double x_interval = q.x() - q_heading.x();
    double y_interval = q.y() - q_heading.y();
    double distance;
    distance = sqrt(x_interval * x_interval + y_interval * y_interval);
    return distance;
}
double MyGDAlgorithm::dist2 (Eigen::Vector2d q, Eigen::Vector2d q_heading) const{
    double x_interval = q.x() - q_heading.x();
    double y_interval = q.y() - q_heading.y();
    double distance = x_interval * x_interval + y_interval * y_interval;
    return distance;
}
double MyGDAlgorithm::U_att(Eigen::Vector2d q, Eigen::Vector2d q_heading) const{
    // Threshold for the goal.
    double U_att;
    double distqqheading = dist(q,q_heading);
    if (distqqheading <= d_star){
        U_att = 0.5 * zetta* dist2(q,q_heading);
    }
    else{
        U_att = d_star * zetta * dist(q,q_heading) - 0.5 * zetta * d_star * d_star;
    }
    //std::cout << "q value: " << q << std::endl;
    // std::cout << "q_heading value: " << q_heading << std::endl;
    // std::cout << "distance value: " << dist(q,q_heading) << std::endl;
    // std::cout << "U_att value: " << U_att << std::endl;
    return U_att;
}
double MyGDAlgorithm::pointToSegmentDistance(const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& P) const {
    // A distance between line segment AB and point P.
    Eigen::Vector2d AB = B - A; 
    Eigen::Vector2d AP = P - A;

    double t = AB.dot(AP) / AB.squaredNorm(); // interpolate parameter t.
    t = std::max(0.0, std::min(1.0, t)); // t should be the region from 0. to 1.

    Eigen::Vector2d closestPoint = A + t * AB; // this is the closest point.
    return (P - closestPoint).norm(); // return the distance
}
double MyGDAlgorithm::disti(const amp::Polygon obstacle, const Eigen::Vector2d& P) const {
    double minDistance = std::numeric_limits<double>::max();  // Initialize the minimum distance to a very large value

    // Iterate over all obstacles in the problem
    const auto& vertices = obstacle.verticesCCW();

    // Iterate over all edges of the polygon
    for (size_t i = 0; i < vertices.size(); ++i) {
        Eigen::Vector2d v1 = vertices[i];
        Eigen::Vector2d v2 = vertices[(i + 1) % vertices.size()];  // Wrap around to the first vertex

        // Calculate the distance from point P to the edge (v1, v2)
        double distance = pointToSegmentDistance(v1, v2, P);

        // Update the minimum distance if a closer edge is found
        minDistance = std::min(minDistance, distance);
    }

    return minDistance;  // Return the minimum distance to any obstacle edge
}
// Helper function to get max radii of an obstacle
double obstacleRadii(const amp::Obstacle2D& obs, const Eigen::Vector2d& centroid) {
    double maxDist = 0; // initialize distance counter
    for (Eigen::Vector2d vtx : obs.verticesCCW()) {
        if (maxDist < ((vtx - centroid).norm())) {
            maxDist = (vtx - centroid).norm(); // set circle radius to maximum vertex distance
        }
    }
    return maxDist;
}

Eigen::Vector2d MyGDAlgorithm::gradient(Eigen::Vector2d q, amp::Problem2D problem) const{
    int k = 8; 
    std::vector<int> nearestIndices = getNearestObstacleIndices(q, problem, k);

    Eigen::Vector2d gradient_U = Eigen::Vector2d::Zero();

    for (size_t i = 0; i < nearestIndices.size(); ++i) {
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (const auto& vertex : problem.obstacles[nearestIndices[i]].verticesCCW()) {
            centroid += vertex;
        }
        centroid /= problem.obstacles[nearestIndices[i]].verticesCCW().size();

        double distance = disti(problem.obstacles[nearestIndices[i]], q);

        if (distance <= Q_star) {
            Eigen::Vector2d grad_U_rep_i = -eta * (1 / distance - 1 / Q_star) * (q - centroid) / std::pow(distance, 3);
            gradient_U += grad_U_rep_i;
        }
    }

    Eigen::Vector2d q_goal = problem.q_goal;
    double distanceToGoal = dist(q, q_goal);
    Eigen::Vector2d gradient_U_att;
    if (distanceToGoal <= d_star) {
        gradient_U_att = zetta * (q - q_goal);
    } else {
        gradient_U_att = d_star * zetta * (q - q_goal) / distanceToGoal;
    }


    gradient_U += gradient_U_att;
    //std::cout << "gradient_U value: (" << gradient_U[0] << ", " << gradient_U[1] << ")" << std::endl;
    return gradient_U;
}
std::vector<int> MyGDAlgorithm::getNearestObstacleIndices(Eigen::Vector2d q, amp::Problem2D problem, int k) const {
    std::vector<std::pair<double, int>> distances;
    for (size_t i = 0; i < problem.obstacles.size(); ++i) {
        // all centroid from a point
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (const auto& vertex : problem.obstacles[i].verticesCCW()) {
            centroid += vertex;
        }
        centroid /= problem.obstacles[i].verticesCCW().size();
        double distance = dist(q, centroid);
        distances.push_back(std::make_pair(distance, i));
    }
    // sort on a distance
    std::sort(distances.begin(), distances.end());

    // nearest K 
    std::vector<int> nearest_indices;
    for (int i = 0; i < std::min(k, static_cast<int>(distances.size())); ++i) {
        nearest_indices.push_back(distances[i].second);
    }

    return nearest_indices;
}

double MyGDAlgorithm::U_rep1(Eigen::Vector2d q, amp::Problem2D problem) const {
    int k = 8;

    std::vector<int> nearestIndices = getNearestObstacleIndices(q, problem, k);

    double U_rep1 = 0;

    // Repulsive potential from that
    for (size_t i = 0; i < nearestIndices.size(); ++i) {
        // Centroid of obstacles
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (const auto& vertex : problem.obstacles[nearestIndices[i]].verticesCCW()) {
            centroid += vertex;
        }
        centroid /= problem.obstacles[nearestIndices[i]].verticesCCW().size();

        // the minimum distance
        double distance = disti(problem.obstacles[nearestIndices[i]], q);

        if (distance <= Q_star) {
            // adding repulsive potential if the distance is close enough to consider.
            double U_rep_i = 0.5 * eta * std::pow((1 / distance - 1 / Q_star), 2);
            U_rep1 += U_rep_i;
        }
    }

    //std::cout << "U_rep1 value: " << U_rep1 << std::endl;
    return U_rep1;
}

amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    // Initialization.
    double eps = 0.25;
    path.waypoints.push_back(problem.q_init);
    //path.waypoints.push_back(Eigen::Vector2d(0.0, 0.0));
    //path.waypoints.push_back(Eigen::Vector2d(10.0, 0.0));
    //path.waypoints.push_back(problem.q_goal);
    // Do Gradient descent algorithm
    double MaxIter = 80000; 
    double i = 0;
    Eigen::Vector2d q_init = problem.q_init;
    path.waypoints.push_back(q_init);
    Eigen::Vector2d q = q_init;
    double alpha = 0.001;
    double beta = 0.10;
    double gradient_norm = gradient(q,problem).norm();
    double noise_scale = 0.001;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, noise_scale);
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    double distance = 0;
    //while (gradient(q,problem).norm()> eps && i < MaxIter) {
    while (dist(q,problem.q_goal)> eps && i < MaxIter) {
        //std::cout << "q: (" << q[0] << ", " << q[1] << ")" << std::endl;
        Eigen::Vector2d noise(distribution(generator), distribution(generator));
        velocity = beta * velocity - alpha * gradient(q,problem);
        Eigen::Vector2d q_new = q + velocity + noise;
        distance += (q - q_new).norm();
        q = q_new;
        path.waypoints.push_back(q);
        i += 1;
        gradient_norm=gradient(q,problem).norm();
    }
    path.waypoints.push_back(problem.q_goal);
    double SIZE= path.waypoints.size() ;
    std::cout << "length of path" << ": "<<distance<< std::endl;
    return path;
}
