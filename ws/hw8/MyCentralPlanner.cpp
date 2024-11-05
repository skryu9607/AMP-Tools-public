#include "MyMultiAgentPlanners.h"
#include "../hw7/MySamplingBasedPlanners.h"


class CollisionChecker {
public:
    CollisionChecker(double r = 0.5) : radius(r) {}
    bool checkLineCollision(const Eigen::Vector2d& seg_p1, const Eigen::Vector2d& seg_p2, const std::vector<amp::Polygon>& obstacles) {
        for (const auto& obstacle : obstacles) {
            if (lineCollision(seg_p1, seg_p2, obstacle)) {
                return true;
            }
        }
        return false;
    }
    bool checkPointCollision(const Eigen::Vector2d& testPoint, const std::vector<amp::Polygon>& obstacles) const {
        for (const auto& obstacle : obstacles) {
            if (isDiskInsideConvexPolygon(testPoint, obstacle.verticesCW())) {
                return true;
            }
        }
        return false;
    }
    // 
    bool checkDiskCollision(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double radius, const std::vector<amp::Polygon>& obstacles) {
        // Making perpendicular
        Eigen::Vector2d direction = (end - start).normalized();
        Eigen::Vector2d perpendicular(-direction.y(), direction.x());

        // definition of all vertices
        double Radius = radius + 0.1;
        Eigen::Vector2d startoffset = start - Radius * direction;
        Eigen::Vector2d endoffset = end + Radius * direction;
        Eigen::Vector2d rect_p1 = startoffset + Radius * perpendicular;
        Eigen::Vector2d rect_p2 = startoffset - Radius * perpendicular;
        Eigen::Vector2d rect_p3 = endoffset + Radius * perpendicular;
        Eigen::Vector2d rect_p4 = endoffset - Radius * perpendicular;

        // collision check
        for (const auto& obstacle : obstacles) {
            if (checkPolygonCollision(rect_p1, rect_p2, rect_p3, rect_p4, obstacle)) {
                return true;
            }
        }
        return false; 
    }
private:
    double radius;
    double crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
        return a.x() * b.y() - a.y() * b.x();
    }
   bool doLineSegmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                             const Eigen::Vector2d& q1, const Eigen::Vector2d& q2) {
    Eigen::Vector2d r = p2 - p1;
    Eigen::Vector2d s = q2 - q1;
    double rxs = crossProduct(r, s);
    Eigen::Vector2d qp = q1 - p1;
    double qpxr = crossProduct(qp, r);
    const double EPSILON = 1e-3;

    // Case 1: Collinear case
    if (std::abs(rxs) < EPSILON && std::abs(qpxr) < EPSILON) {
        double r_dot_r = r.dot(r);
        double s_dot_r = s.dot(r);
        double t0 = qp.dot(r) / r_dot_r;
        double t1 = t0 + s_dot_r / r_dot_r;

        double t_min = std::min(t0, t1);
        double t_max = std::max(t0, t1);

        // Check if segments overlap
        return !(t_max < 0 || t_min > 1);
    }

    // Case 2: Parallel but not collinear
    if (std::abs(rxs) < EPSILON && std::abs(qpxr) >= EPSILON) {
        return false;
    }

    // Case 3: General intersection
    if (std::abs(rxs) >= EPSILON) {
        double t = crossProduct(qp, s) / rxs;
        double u = crossProduct(qp, r) / rxs;
        if (t >= -EPSILON && t <= 1 + EPSILON && u >= -EPSILON && u <= 1 + EPSILON) {
            return true;
        }
    }

    return false;
    }

    bool lineCollision(const Eigen::Vector2d& seg_p1, const Eigen::Vector2d& seg_p2,
                    const amp::Polygon& polygon) {
        const std::vector<Eigen::Vector2d>& vertices = polygon.verticesCW();
        std::size_t num_vertices = vertices.size();

        for (std::size_t i = 0; i < num_vertices; ++i) {
            const Eigen::Vector2d& poly_p1 = vertices[i];
            const Eigen::Vector2d& poly_p2 = vertices[(i + 1) % num_vertices];

            // Check for intersection between the segment and each polygon edge
            if (doLineSegmentsIntersect(seg_p1, seg_p2, poly_p1, poly_p2)) {
                return true;  // Collision detected
            }
        }

        return false;  // No collision with any edges
    }


    bool isDiskInsideConvexPolygon(const Eigen::Vector2d& testPoint, const std::vector<Eigen::Vector2d>& vertices) const {
        std::size_t num_vertices = vertices.size();
        int sign = 0; 
        // Calculating the crossProduct over all edges.
        for (std::size_t i = 0; i < num_vertices; ++i) {
            std::size_t next = (i + 1) % num_vertices;

            // Current edges and test point
            const Eigen::Vector2d edge = vertices[next] - vertices[i];
            const Eigen::Vector2d toTestPoint = testPoint - vertices[i];
            double buffer = 0.25;
            double crossProd = crossProduct(edge, toTestPoint);

        if (crossProd != 0) { 
            int currentSign = (crossProd > 0) ? 1 : -1;

            if (sign == 0) {
                sign = currentSign; 
            } else if (sign != currentSign) {
                double distance = distanceToSegment(testPoint, vertices[i], vertices[next]);
                if (distance > radius + buffer) {
                    if ((testPoint - vertices[i]).norm() > radius + buffer) {
                        if ((testPoint - vertices[next]).norm() > radius + buffer){
                            return false;
                    }
                    }
                }
            }
        }
        }
        return true;
    }    
    double distanceToSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& v, const Eigen::Vector2d& w) const {
        Eigen::Vector2d edge = w - v;
        double l2 = edge.squaredNorm(); 
        if (l2 == 0.0) return (p - v).norm();
        double t = std::clamp((p - v).dot(edge) / l2, 0.0, 1.0);
        Eigen::Vector2d projection = v + t * edge;
        return (p - projection).norm();
    }

    bool checkPolygonCollision(const Eigen::Vector2d& rect_p1, const Eigen::Vector2d& rect_p2,
                               const Eigen::Vector2d& rect_p3, const Eigen::Vector2d& rect_p4,
                               const amp::Polygon& polygon) {
        const std::vector<Eigen::Vector2d>& vertices = polygon.verticesCW();
        std::size_t num_vertices = vertices.size();

        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> rectEdges = {
            {rect_p1, rect_p2}, {rect_p2, rect_p4}, {rect_p4, rect_p3}, {rect_p3, rect_p1}
        };

        for (const auto& rectEdge : rectEdges) {
            for (std::size_t i = 0; i < num_vertices; ++i) {
                const Eigen::Vector2d& poly_p1 = vertices[i];
                const Eigen::Vector2d& poly_p2 = vertices[(i + 1) % num_vertices];
                
                if (doLineSegmentsIntersect(rectEdge.first, rectEdge.second, poly_p1, poly_p2)) {
                    return true; 
                }
            }
        }
        return false; 
    }

};
size_t MyCentralPlanner::getNodePositionSize() const {
    return nodePositions.size();
}

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;

    // Parameters
    double r = 0.5;              // Step size
    double p_goal = 0.05;        // Goal bias probability
    size_t maxIterations = 20000; // Maximum number of iterations
    double epsilon = 0.25;       // Termination distance

    //size_t numAgents = problem.agent_properties.size(); // Number of agents (m)
    size_t numAgents = problem.numAgents();
    int metaSpaceDim = 2 * numAgents; // R^2m
    std::cout<<"Number of agents : " <<problem.numAgents() <<std::endl;
    // Define start and goal positions in R^2m (meta-agent space)
    Eigen::VectorXd startPosition(metaSpaceDim);
    Eigen::VectorXd goalPosition(metaSpaceDim);

    for (size_t i = 0; i < numAgents; ++i) {
        startPosition[2 * i] = problem.agent_properties[i].q_init.x();
        startPosition[2 * i + 1] = problem.agent_properties[i].q_init.y();
        goalPosition[2 * i] = problem.agent_properties[i].q_goal.x();
        goalPosition[2 * i + 1] = problem.agent_properties[i].q_goal.y();
    }
    //std::unordered_map<int, Eigen::VectorXd> nodePositions; // Meta-agent positions in R^2m
    std::unordered_map<int, int> parentMap;                 // Parent-child relationships for path reconstruction

    auto graphPtr = std::make_shared<amp::Graph<double>>(); // RRT tree
    CollisionChecker checker;                               // Collision checker for obstacles and agent-agent collisions
    // Initialize start node
    int startNode = 0;
    nodePositions[startNode] = startPosition;
    graphPtr->connect(startNode, startNode, 0.0); // Connect start node to itself

    // Random number generators for sampling in R^2m
    std::random_device rd;
    std::mt19937 gen(rd());
    //std::mt19937 gen(1);
    std::uniform_real_distribution<> disX(problem.x_min, problem.x_max);
    std::uniform_real_distribution<> disY(problem.y_min, problem.y_max);
    std::uniform_real_distribution<> disProb(0.0, 1.0);

    // Main RRT loop
    int finalNode = -1;
    for (size_t i = 0; i < maxIterations * numAgents ; ++i) {
        Eigen::VectorXd randomPoint(metaSpaceDim);
        //std::cout<<i<<std::endl;
        // Goal bias sampling
        if (disProb(gen) < p_goal) {
            for (size_t j = 0; j < numAgents; ++j) {
                randomPoint[2 * j] = problem.agent_properties[j].q_goal.x();
                randomPoint[2 * j + 1] = problem.agent_properties[j].q_goal.y();
            }
        } else {
            for (size_t j = 0; j < numAgents; ++j) {
                randomPoint[2 * j] = disX(gen);
                randomPoint[2 * j + 1] = disY(gen);
            }
            // std::cout << "Random sampling - randomPoint for agent 0: ("
            //           << randomPoint[0] << ", " << randomPoint[1] << ")" << std::endl;
        }

        // Find the nearest node in the meta-agent space
        int nearestNodeId = -1;
        double minDistance = std::numeric_limits<double>::infinity();
        for (const auto& [nodeId, position] : nodePositions) {
            double distance = (position - randomPoint).norm();
            if (distance < minDistance) {
                minDistance = distance;
                nearestNodeId = nodeId;
            }
        }

        // Generate new node by moving `r` distance towards the random point
        Eigen::VectorXd nearestPosition = nodePositions[nearestNodeId];
        Eigen::VectorXd direction = (randomPoint - nearestPosition).normalized();
        Eigen::VectorXd newPoint = nearestPosition + r * direction;

        // Check for collisions for each agent
        bool collisionFree = true;
        double disk_R = 0.5;
        double buffer = 1.0;

        for (size_t j = 0; j < numAgents; ++j) {
            Eigen::Vector2d agentPos(newPoint[2 * j], newPoint[2 * j + 1]);
            
            if (checker.checkPointCollision(agentPos, problem.obstacles)) {
                collisionFree = false;
                // std::cout << "Agent (" << j << ") collision with obstacle at position: ("
                //           << agentPos.x() << ", " << agentPos.y() << ")" << std::endl;
                break;
            }

            for (size_t k = j + 1; k < numAgents; ++k) {
                Eigen::Vector2d otherAgentPos(newPoint[2 * k], newPoint[2 * k + 1]);
                double minDist = disk_R * 2 + buffer;

                if ((agentPos - otherAgentPos).norm() <= minDist) {
                    collisionFree = false;
                    //std::cout << "Collision between agent " << j << " and agent " << k << std::endl;
                    break;
                }
            }



            // Check for collision along path from nearestPosition to newPoint
            Eigen::Vector2d nearestAgentPos(nearestPosition[2 * j], nearestPosition[2 * j + 1]);
            if (checker.checkDiskCollision(nearestAgentPos, agentPos, disk_R, problem.obstacles)) {
                collisionFree = false;
                // std::cout << "Path collision for agent " << j << " between positions: ("
                //           << nearestAgentPos.x() << ", " << nearestAgentPos.y() << ") and ("
                //           << agentPos.x() << ", " << agentPos.y() << ")" << std::endl;
                break;
            }

            if (!collisionFree) break;
        }

        if (collisionFree) {
            int newNodeId = nodePositions.size();
            nodePositions[newNodeId] = newPoint;
            double edgeWeight = (newPoint - nearestPosition).norm();
            graphPtr->connect(nearestNodeId, newNodeId, edgeWeight);
            parentMap[newNodeId] = nearestNodeId;

            bool allAgentsReachedGoal = true;
            for (size_t j = 0; j < numAgents; ++j) {
                Eigen::Vector2d agentNewPoint(newPoint[2 * j], newPoint[2 * j + 1]);
                Eigen::Vector2d agentGoalPosition(goalPosition[2 * j], goalPosition[2 * j + 1]);

                if ((agentNewPoint - agentGoalPosition).norm() >= epsilon) {
                    allAgentsReachedGoal = false;
                    break;
                }
            }

            if (allAgentsReachedGoal) {
                finalNode = newNodeId;
                std::cout << "Goal reached in R^2m space." << std::endl;
                break;
            }
        }
    }

    //size_t numAgents = problem.agent_properties.size();
    std::cout << "Number of agents (from problem): " << numAgents << std::endl;
    std::cout << "Total nodes in the RRT tree: " << nodePositions.size() << std::endl;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init}; 
        path.agent_paths.push_back(agent_path);
    }

    std::vector<int> reversePath;
    int currentNode = finalNode;
    bool pathComplete = true;
    std::cout << "Final Node : " << finalNode <<std::endl;
    if (currentNode == -1 || nodePositions.find(currentNode) == nodePositions.end()) {
        path.agent_paths.resize(numAgents);
        std::cerr << "Error: finalNode is invalid or not found in nodePositions." << std::endl;
        //std::cout << "1" << std::endl;
        // for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        //     amp::Path2D agent_path;
        //     agent_path.waypoints = {agent.q_init}; 
        //     path.agent_paths.push_back(agent_path);
        // }
        //std::cout << "2" << std::endl;
        for (size_t i = 0; i < numAgents; ++i) {
            path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_goal);  
        }
        //std::cout << "Number of agents : " << numAgents << std::endl;
        //std::cout << "Size of path.agent_paths: " << path.agent_paths.size() << std::endl;
        //std::cout << "3" << std::endl;
        return path;
    }
    while (currentNode != startNode) {
        if (parentMap.find(currentNode) == parentMap.end()) {
            std::cerr << "Error in path reconstruction. Missing parent for node " << currentNode << std::endl;
            pathComplete = false;
            break;
        }
        reversePath.push_back(currentNode);
        currentNode = parentMap[currentNode];
    }


    if (pathComplete) {
        reversePath.push_back(startNode); 

        for (auto it = reversePath.rbegin(); it != reversePath.rend(); ++it) {
            Eigen::VectorXd metaPosition = nodePositions[*it];
            for (size_t i = 0; i < numAgents; ++i) {
                Eigen::Vector2d agentPosition(metaPosition[2 * i], metaPosition[2 * i + 1]);
                path.agent_paths[i].waypoints.push_back(agentPosition); // 중간 경로 추가
            }
        }

        for (size_t i = 0; i < numAgents; ++i) {
            path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_goal); // 목표 위치 추가
        }
    } else {
        std::cerr << "Warning: Path reconstruction could not complete fully due to missing node connections." << std::endl;
        path.agent_paths.clear(); 
    }

    return path;
}

size_t MyDecentralPlanner::getNodePositionSize() const {
    return nodePositions.size();
}


amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    std::vector<amp::Path2D> agent_paths;
    double agent_radius = 0.5;
    double buffer = 0.5 * agent_radius;
    double minDist = agent_radius * 2 + buffer;
    int maxRetries = 3; // 재계획 최대 시도 횟수
    size_t maxiterations = 10000;
    // 각 agent의 경로를 순차적으로 계획
    std::cout << "Number of agents : " << problem.agent_properties.size() << std::endl;
    for (size_t i = 0; i < problem.agent_properties.size(); ++i) {
        const auto& agent = problem.agent_properties[i];
        // 개별 agent의 문제 설정
        amp::Problem2D single_agent_problem;
        single_agent_problem.q_init = agent.q_init;
        single_agent_problem.q_goal = agent.q_goal;
        single_agent_problem.obstacles = problem.obstacles;  // Static obstacles

        amp::Path2D agent_path;
        bool path_found = false;

        // 충돌 검사 및 재계획 반복 수행
        for (int retry = 0; retry < maxRetries; ++retry) {
            agent_path = rrtplan(single_agent_problem, {}, minDist, maxiterations);
            //std::cout << "agent " << i << " path found" << std::endl;
            bool collision_free = true;
            std::cout << "Collision checking"<< std::endl;
            // 이전 agent들과의 충돌 검사
            for (size_t j = 0; j < i; ++j) { // only previous agents
                if (i == j){
                    continue;
                }
                const amp::Path2D& prev_path = agent_paths[j];
                size_t max_time = std::min(agent_path.waypoints.size(), prev_path.waypoints.size());
                for (size_t t = 0; t < max_time; ++t) {
                    if ((agent_path.waypoints[t] - prev_path.waypoints[t]).norm() < minDist) {
                        // 충돌 발생 - 재계획을 위한 동적 장애물 설정
                        std::vector<std::vector<Eigen::Vector2d>> dynamic_obstacles;
                        for (size_t tt = t; tt < max_time; ++tt) {
                            dynamic_obstacles.push_back({prev_path.waypoints[tt]});
                        }
                        agent_path = rrtplan(single_agent_problem, dynamic_obstacles, minDist,maxiterations);
                        collision_free = false;
                        break;
                    }
                }
                if (!collision_free) break;
            }

            if (collision_free) {
                path_found = true;
                break;
            }
        }

        // 빈 경로인지 확인하고 기본 경로 설정 (계획 실패 시)
        if (!path_found || agent_path.waypoints.empty()) {
            std::cerr << "Warning: Path planning failed for agent " << i << ", falling back to straight-line path." << std::endl;
            agent_path.waypoints.push_back(agent.q_init);
            agent_path.waypoints.push_back(agent.q_goal);
        }
        agent_paths.push_back(agent_path); // 최종 경로 저장
    }

    // MultiAgentPath2D 형식으로 변환
    for (const auto& agent_path : agent_paths) {
        path.agent_paths.push_back(agent_path);
    }
    std::cout << "Every agent has valid path." << std::endl;
    return path;
}

amp::Path2D MyDecentralPlanner::rrtplan(
    const amp::Problem2D& problem, 
    const std::vector<std::vector<Eigen::Vector2d>>& dynamicObstacles,
    double minDist,size_t maxiterations) {
    amp::Path2D rrtpath;
    double agent_radius = 0.5;
    double buffer = 0.05;
    // Parameters
    double r = 0.5;              // Step size
    double p_goal = 0.05;        // Goal bias probability
    //size_t maxIterations = 100000; // Maximum number of iterations
    double epsilon = 0.25;       // Termination distance
    size_t maxRetries = 2;
    Eigen::Vector2d startPosition(problem.q_init.x(), problem.q_init.y());
    Eigen::Vector2d goalPosition(problem.q_goal.x(), problem.q_goal.y());

    std::unordered_map<int, Eigen::Vector2d> nodePositions;
    std::unordered_map<int, int> parentMap;

    auto graphPtr = std::make_shared<amp::Graph<double>>();
    CollisionChecker checker;

    int startNode = 0;
    nodePositions[startNode] = startPosition;
    graphPtr->connect(startNode, startNode, 0.0);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> disX(problem.x_min, problem.x_max);
    std::uniform_real_distribution<> disY(problem.y_min, problem.y_max);
    std::uniform_real_distribution<> disProb(0.0, 1.0);

    int finalNode = -1;
    for (size_t i = 0; i < maxiterations; ++i) {
        Eigen::Vector2d randomPoint = (disProb(gen) < p_goal) ? goalPosition : Eigen::Vector2d(disX(gen), disY(gen));

        int nearestNodeId = -1;
        double minDistance = std::numeric_limits<double>::infinity();
        for (const auto& [nodeId, position] : nodePositions) {
            double distance = (position - randomPoint).norm();
            if (distance < minDistance) {
                minDistance = distance;
                nearestNodeId = nodeId;
            }
        }

        Eigen::Vector2d nearestPosition = nodePositions[nearestNodeId];
        Eigen::Vector2d direction = (randomPoint - nearestPosition).normalized();
        Eigen::Vector2d newPoint = nearestPosition + r * direction;

        double disk_R = agent_radius + buffer;
        bool staticCollisionFree = true;
        bool dynamicCollisionFree = true;

        // 정적 장애물 충돌 검사
        if (checker.checkPointCollision(newPoint, problem.obstacles) ||
            checker.checkDiskCollision(nearestPosition, newPoint, agent_radius, problem.obstacles)) {
            staticCollisionFree = false;
            continue; // Static 장애물과 충돌 시 다음 반복으로
        }

        // 동적 장애물 충돌 검사
        for (const auto& dynamicObstacle : dynamicObstacles) {
            for (const Eigen::Vector2d& vertex : dynamicObstacle) {
                if ((newPoint - vertex).norm() < 2 * agent_radius + buffer) {
                    dynamicCollisionFree = false;
                    break;
                }
            }
            if (!dynamicCollisionFree) break;
        }

        // 동적 장애물과 충돌이 없는 경우 RRT 트리에 새로운 노드 추가
        if (dynamicCollisionFree) {
            int newNodeId = nodePositions.size();
            nodePositions[newNodeId] = newPoint;
            double edgeWeight = (newPoint - nearestPosition).norm();
            graphPtr->connect(nearestNodeId, newNodeId, edgeWeight);
            parentMap[newNodeId] = nearestNodeId;
            //std::cout << "tree is growing. " << std::endl;
            // 목표 도달 검사
            if ((newPoint - goalPosition).norm() < epsilon) {
                finalNode = newNodeId;
                std::cout << "Path found" << std::endl;
                break;
            }
        }
    }
    std::cout << "NodePositions' size : " << nodePositions.size() << std::endl;
    // 경로 재구성
    std::vector<int> reversePath;
    int currentNode = finalNode;
    bool pathComplete = true;

    if (currentNode == -1 || nodePositions.find(currentNode) == nodePositions.end()) {
        std::cerr << "Error: finalNode is invalid or not found in nodePositions. Retrying RRT..." << std::endl;
        if (maxRetries > 0) {
            maxRetries = maxRetries - 1;
            return rrtplan(problem, dynamicObstacles, minDist,5 * maxiterations);
        } else {
            rrtpath.waypoints = {problem.q_init, problem.q_goal};  // 기본 경로 (시작점과 목표점만 포함)
            return rrtpath;
        }
    }

    while (currentNode != startNode) {
        if (parentMap.find(currentNode) == parentMap.end()) {
            std::cerr << "Error in path reconstruction. Missing parent for node " << currentNode << std::endl;
            pathComplete = false;
            break;
        }
        reversePath.push_back(currentNode);
        currentNode = parentMap[currentNode];
    }

    if (pathComplete) {
        reversePath.push_back(startNode);
        for (auto it = reversePath.rbegin(); it != reversePath.rend(); ++it) {
            rrtpath.waypoints.push_back(nodePositions[*it]);
        }
        std::cout << "a single path found " << std::endl;
        rrtpath.waypoints.push_back(problem.q_goal);
    } else {
        std::cerr << "Warning: Path reconstruction could not complete fully due to missing node connections." << std::endl;
        rrtpath.waypoints.clear();
    }

    return rrtpath;
}
