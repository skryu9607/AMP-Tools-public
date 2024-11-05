
# include "../hw7/MySamplingBasedPlanners.h"
# include <random>
#include <queue>
#include <limits>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
struct Node{
    int id;
    Eigen::Vector2d position;
    Node() : id(-1), position(Eigen::Vector2d::Zero()) {}
    Node(int id, const Eigen::Vector2d & pos) : id(id), position(pos) {};
};
class Graph{
    private:
        // definition of vertices, and edges
        std::unordered_map<int,Node> V;
        std::vector<std::pair<int, int>> E;
        std::unordered_map<int,std::vector<int>> AdjList;
    public:
        void addNode(int id, const Eigen::Vector2d position){
            // if V.find(id) == V.end() : Something new.
            if (V.find(id) == V.end()) {
            V[id] = Node(id, position);
        }
        }
        void addEdge(int id1, int id2) {
        // Ensure edges are unique
        auto edge = std::make_pair(std::min(id1, id2), std::max(id1, id2));
        if (std::find(E.begin(), E.end(), edge) == E.end()) {
            E.push_back(edge);
            AdjList[id1].push_back(id2);
            AdjList[id2].push_back(id1);
            }
        }

        Node getNode(int id) const {
            return V.at(id);
        }

        const std::unordered_map<int, Node>& getNodes() const {
            return V;
        }

        const std::vector<std::pair<int, int>>& getEdges() const {
            return E;
        }
        void printGraph() const {
            std::cout << "Nodes: " <<std::endl;
            for(const auto& [id,node] : V){
                std::cout << "Node ID : " << id << " Position: (" << node.position.x() << ", " << node.position.y() << ")" << std::endl;
            }
            std::cout << "Edges: " << std::endl;
            for (const auto& edge : E) {
                std::cout << "Edge between Node" <<edge.first << ", Node " <<edge.second <<std::endl;
            }
        }
};

double MyPRM::Random(double min_val, double max_val) {
    std::random_device rd;  // seed generate
    std::mt19937 gen(rd()); // Mersenne Twister 
    std::uniform_real_distribution<> dis(min_val, max_val); // 균등 실수 분포 설정

    return dis(gen);
}



class CollisionChecker {
public:
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
            if (isPointInsideConvexPolygon(testPoint, obstacle.verticesCW())) {
                return true;
            }
        }
        return false;
    }

private:
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
        const double EPSILON = 1e-6;

        if (std::abs(rxs) < EPSILON && std::abs(qpxr) < EPSILON) {
            double r_dot_r = r.dot(r);
            double s_dot_r = s.dot(r);
            double t0 = qp.dot(r) / r_dot_r;
            double t1 = t0 + s_dot_r / r_dot_r;

            double t_min = std::min(t0, t1);
            double t_max = std::max(t0, t1);

            if (t_max < 0 || t_min > 1) {
                return false;

            } else {

                return true;
            }
        }

        if (std::abs(rxs) < EPSILON && std::abs(qpxr) >= EPSILON) {

            return false;
        }

        if (std::abs(rxs) >= EPSILON) {
            double t = crossProduct(qp, s) / rxs;
            double u = crossProduct(qp, r) / rxs;
            if (t >= 0 - EPSILON && t <= 1 + EPSILON && u >= 0 - EPSILON && u <= 1 + EPSILON) {

                return true;
            }
        }

        return false;
    };

    bool lineCollision(const Eigen::Vector2d& seg_p1, const Eigen::Vector2d& seg_p2,
                const amp::Polygon& polygon) {
        const std::vector<Eigen::Vector2d>& vertices = polygon.verticesCW();
        std::size_t num_vertices = vertices.size();

        for (std::size_t i = 0; i < num_vertices; ++i) {
            const Eigen::Vector2d& poly_p1 = vertices[i];
            const Eigen::Vector2d& poly_p2 = vertices[(i + 1) % num_vertices];

            if (doLineSegmentsIntersect(seg_p1, seg_p2, poly_p1, poly_p2)) {
                return true;  
            }
        }

        return false;  
    }

    bool isPointInsideConvexPolygon(const Eigen::Vector2d& testPoint, const std::vector<Eigen::Vector2d>& vertices) const {
        std::size_t num_vertices = vertices.size();

        // Calculating the crossProduct over all edges.
        for (std::size_t i = 0; i < num_vertices; ++i) {
            std::size_t next = (i + 1) % num_vertices;

            // Current edges and test point
            const Eigen::Vector2d edge = vertices[next] - vertices[i];
            const Eigen::Vector2d toTestPoint = testPoint - vertices[i];

            // Crodss prodcut
            double crossProd = crossProduct(edge, toTestPoint);
            if (crossProd < 0) {
                return false;
            }
        };
        return true;
    }    
};
// Heuristic function for A* (using Euclidean distance as an example)
double MyPRM::heuristic(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return (a - b).norm(); // Euclidean distance
}

// Helper function to get the edge weight between two nodes
double MyPRM::getEdgeWeight(std::shared_ptr<amp::Graph<double>>& graphPtr, amp::Node src, amp::Node dst) {
    const auto& children = graphPtr->children(src);
    auto it = std::find(children.begin(), children.end(), dst);
    if (it != children.end()) {
        size_t index = std::distance(children.begin(), it);
        const auto& weights = graphPtr->outgoingEdges(src);
        if (index < weights.size()) {
            return weights[index];
        }
    }
    return std::numeric_limits<double>::infinity(); // Return a large value if no direct edge exists
}

// A* implementation function
std::vector<amp::Node> MyPRM::aStar(std::shared_ptr<amp::Graph<double>>& graphPtr, amp::Node startNode, amp::Node goalNode, const std::unordered_map<amp::Node, Eigen::Vector2d>& nodePositions) {
    using NodeCostPair = std::pair<double, amp::Node>;
    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, std::greater<>> openSet;
    // save the gScore 
    std::unordered_map<amp::Node, double> gScore;
    // save the history of parents.
    std::unordered_map<amp::Node, amp::Node> cameFrom;
    // Set the all gScore is infinity.
    for (const auto& [node, pos] : nodePositions) {
        gScore[node] = std::numeric_limits<double>::infinity();
    }
    // Except the startNode
    gScore[startNode] = 0.0;

    if (nodePositions.find(startNode) == nodePositions.end() || nodePositions.find(goalNode) == nodePositions.end()) {
        std::cerr << "Error: Start or Goal node not found in nodePositions." << std::endl;
        return {};
    }
    // emplace : adding something in using priority_queue.
    // kind of initialization.
    openSet.emplace(heuristic(nodePositions.at(startNode), nodePositions.at(goalNode)), startNode);

    size_t i = 0;
    //while (!openSet.empty()) {
    // Until exploration.
    while (!openSet.empty()) {   
        // openSet.top().second() : the most priority's second term.
        amp::Node current = openSet.top().second;
        // current node is visited node. so, to do no exploration, just erase.
        openSet.pop();
        if (i > 1000000){
            std::cout << "Infinite loop" << std::endl;
            return {};
        }
        if (current == goalNode) {
            //std::cout << "Almost goal" << std::endl;
            std::vector<amp::Node> path;
            std::unordered_set<amp::Node> visited; 
            // current is not startNode <- go backward.
            while (current != startNode) {
                if (cameFrom.find(current) == cameFrom.end()) {
                    std::cerr << "Error: Cannot trace back the path. Node " << current << " not found in cameFrom." << std::endl;
                    // Debugging
                    std::cerr << "Disconnected nodes detected." << std::endl;
                    std::cerr << "Currently traced path: ";
                    for (const auto& node : path) {
                        std::cerr << node << " ";
                    }
                    std::cerr << std::endl;
                    // Debugging
                    for (const auto& [node, pos] : nodePositions) {
                        if (visited.find(node) == visited.end()) {
                            std::cout << "Unconnected node: " << node << " Position: (" 
                                    << pos.x() << ", " << pos.y() << ")" << std::endl;
                        }
                    }
                    return {};
                }
                // addding current node
                path.push_back(current);
                // adding current node to visited group
                visited.insert(current); 
                // next current is a parent of current node.
                current = cameFrom[current];
            }
            path.push_back(startNode);
            std::reverse(path.begin(), path.end());
            std::cout << "End A star" << std::endl;
            return path;
        }
        //current node's nearest node save into graphPtr. 
        const auto& neighbors = graphPtr->children(current);
        //std::cout << "Number of neighbors for node " << current << ": " << neighbors.size() << std::endl;
        i +=1;
        // Do an exploration for the children(neighbors) of current node.
        for (const auto& neighbor : neighbors) {
            // check if the neighbor node is in nodePosition.
            if (nodePositions.find(neighbor) == nodePositions.end()) {
                std::cerr << "Error: Neighbor " << neighbor << " not found in nodePositions." << std::endl;
                continue; // Skip this neighbor
            }
            double edgeWeight = getEdgeWeight(graphPtr, current, neighbor);
            //std::cout << "GetEdgeWeight: " << edgeWeight << std::endl;

            double tentativeGScore = gScore[current] + edgeWeight;
            //std::cout << "Tentative GScore: " << tentativeGScore << ", Current GScore for neighbor: " << gScore[neighbor] << std::endl;
            
            // if current way (tentativegScore and edge) is better
            if (tentativeGScore < gScore[neighbor]) {
                // Set the relationship parent and children. 
                // it is different from the nearesst node relationship.
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeGScore;
                //std::cout << "Updated GScore for Neighbor " << neighbor << ": " << gScore[neighbor] << std::endl;
                double fScore = tentativeGScore + heuristic(nodePositions.at(neighbor), nodePositions.at(goalNode));
                openSet.emplace(fScore, neighbor);
            } else {
                //std::cout << "No Update: Tentative GScore >= Current GScore for Neighbor " << neighbor << std::endl;
            }
        }
    }
}
std::shared_ptr<amp::Graph<double>> MyPRM::generateRoadmap(const amp::Problem2D& problem, std::unordered_map<amp::Node, Eigen::Vector2d>& nodePositions){ 
    auto graphPtr = std::make_shared<amp::Graph<double>>(); // Create a shared pointer for the graph

    CollisionChecker checker;
    size_t n = numSamples; 
    double r = connectionRadius; 

    // Node IDs for start and goal
    amp::Node startNode = 0;
    amp::Node goalNode = n;

    nodePositions[startNode] = Eigen::Vector2d(problem.q_init.x(), problem.q_init.y());
    //std::cout << "Node " << goalNode << " added at: (" << nodePositions[goalNode].x() << ", " << nodePositions[goalNode].y() << ")" << std::endl;
    
    std::random_device rd; // Random device to generate a seed
    std::mt19937 gen(rd()); // Use a fixed seed for the random generator
    std::uniform_real_distribution<> disX(problem.x_min, problem.x_max);
    std::uniform_real_distribution<> disY(problem.y_min, problem.y_max);

    for (size_t i = 1; i < n; ++i) {
        Eigen::Vector2d q(disX(gen), disY(gen));
        if (!checker.checkPointCollision(q, problem.obstacles)) {
            nodePositions[i] = q;
            //std::cout << "Node " << i << " added at: (" << q.x() << ", " << q.y() << ")" << std::endl;
        }
    }
    nodePositions[goalNode] = Eigen::Vector2d(problem.q_goal.x(), problem.q_goal.y());
    std::cout << "Sampling is finished." << std::endl;

    // Connecting nodes based on distance and collision checks
    for (const auto& [src, src_position] : nodePositions) {
        for (const auto& [dst, dst_position] : nodePositions) {
            if (src != dst) {
                double distance = (src_position - dst_position).norm();
                if (distance < r && !checker.checkLineCollision(src_position, dst_position, problem.obstacles)) {
                    graphPtr->connect(src, dst, distance);
                    //std::cout << "Connected nodes: " << src << " <--> " << dst << std::endl;
                }
            }
        }
    }
    std::cout << "Roadmap construction is completed." << std::endl;
    return graphPtr;
}

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    int N = numSamples;
    std::unordered_map<amp::Node, Eigen::Vector2d> nodePositions;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto graphPtr = generateRoadmap(problem, nodePositions); //
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> computation_duration = end_time - start_time;
    double computation_time = computation_duration.count(); //
    // Check if goal node (500) is in nodePositions
    if (nodePositions.find(N) == nodePositions.end()) {
        std::cerr << "Error: Goal node (500) not found in nodePositions." << std::endl;
        return path; // Return an empty path if the goal is not properly added
    }
    // After making graph, a finding a solution is by Astar. 
    std::vector<amp::Node> pathNodes = aStar(graphPtr, 0, N, nodePositions);
    double path_length = 0.0;
    path.waypoints.push_back(problem.q_init);
    for (size_t i = 1; i < pathNodes.size(); ++i) {
        amp::Node currentNode = pathNodes[i - 1];
        amp::Node nextNode = pathNodes[i];
        double segmentLength = (nodePositions[nextNode] - nodePositions[currentNode]).norm();
        path_length += segmentLength;
        path.waypoints.push_back(nodePositions[nextNode]);
    }
    path.waypoints.push_back(problem.q_goal);

    // Computation time과 Path length 출력
    std::cout << "PRM Computation Time: " << computation_time << " seconds" << std::endl;
    std::cout << "Total Path Length: " << path_length << std::endl;

    return path;
}



void MyRRT::addNode(int nodeId, const Eigen::Vector2d& position) {
    nodes[nodeId] = position;
}

// Retrieve all stored nodes as a map
const std::unordered_map<int, Eigen::Vector2d>& MyRRT::getNodes() const {
    return nodes;
}

// Retrieve a node's position by its ID
const Eigen::Vector2d& MyRRT::getNode(int nodeId) const {
    return nodes.at(nodeId);
}

// Add an edge between two nodes
void MyRRT::addEdge(int srcId, int dstId) {
    edges.emplace_back(srcId, dstId);
}

// Retrieve all stored edges
std::shared_ptr<amp::Graph<double>> MyRRT::generateGoalBiasedRRTTree(
    const amp::Problem2D& problem, 
    double r, double p_goal, size_t maxIterations, double epsilon, 
    std::unordered_map<amp::Node, Eigen::Vector2d>& nodePositions) {
    
    auto graphPtr = std::make_shared<amp::Graph<double>>(); // Create a shared pointer for the RRT tree

    CollisionChecker checker;

    // Define start and goal nodes
    amp::Node startNode = 0;
    amp::Node goalNode = 1;

    // Add the start position as the root of the RRT
    Eigen::Vector2d startPosition(problem.q_init.x(), problem.q_init.y());
    graphPtr->connect(startNode, startNode, 0.0); // Connect start node to itself

    nodePositions[startNode] = startPosition;

    // Random number generation setup

    std::random_device rd;  // seed generate
    std::mt19937 gen(rd()); // Use a fixed seed for reproducibility
    std::uniform_real_distribution<> disX(problem.x_min, problem.x_max);
    std::uniform_real_distribution<> disY(problem.y_min, problem.y_max);
    std::uniform_real_distribution<> disProb(0.0, 1.0); // For goal biasing

    // Build the RRT
    for (size_t i = 0; i < maxIterations; ++i) {
        // Step 3: Generate a random sample
        Eigen::Vector2d randomPoint;
        if (disProb(gen) < p_goal) {
            randomPoint = Eigen::Vector2d(problem.q_goal.x(), problem.q_goal.y()); // Bias towards the goal
        } else {
            randomPoint = Eigen::Vector2d(disX(gen), disY(gen)); // Sample randomly
        }

        // Step 4: Find the nearest configuration in the tree to the random point
        int nearestNodeId = -1;
        double minDistance = std::numeric_limits<double>::infinity();

        for (const auto& [nodeId, position] : nodePositions) {
            double distance = (position - randomPoint).norm();
            if (distance < minDistance) {
                minDistance = distance;
                nearestNodeId = nodeId;
            }
        }

        // Step 5: Generate a path and move a step size towards the random point
        Eigen::Vector2d nearestPosition = nodePositions[nearestNodeId];
        Eigen::Vector2d direction = (randomPoint - nearestPosition).normalized();
        Eigen::Vector2d newPoint = nearestPosition + r * direction;

        // Step 6: Check if the new path is collision-free
        if (!checker.checkPointCollision(newPoint, problem.obstacles) &&
            !checker.checkLineCollision(nearestPosition, newPoint, problem.obstacles)) {

            // Step 7: Add the new configuration and edge to the tree
            amp::Node newNodeId = nodePositions.size(); // Assign a new ID to the node
            nodePositions[newNodeId] = newPoint;
            graphPtr->connect(nearestNodeId, newNodeId, (newPoint - nearestPosition).norm());

            // Step 9: Check if we are within epsilon of the goal
            if ((newPoint - Eigen::Vector2d(problem.q_goal.x(), problem.q_goal.y())).norm() < epsilon) {
                graphPtr->connect(newNodeId, goalNode, epsilon); // Connect to goal
                nodePositions[goalNode] = Eigen::Vector2d(problem.q_goal.x(), problem.q_goal.y());
                std::cout << "Goal Biased RRT reached the goal." << std::endl;
                break;
            }
        }
    }

    return graphPtr; // Return the graphPtr containing the corresponding tree
}

std::vector<std::pair<amp::Node, amp::Node>> MyRRT::getEdges(const std::shared_ptr<amp::Graph<double>>& graphPtr) const {
    std::vector<std::pair<amp::Node, amp::Node>> edges;

    // Iterate over all nodes to collect edges
    for (const auto& node : graphPtr->nodes()) {
        const auto& children = graphPtr->children(node); // Get all connected children
        for (const auto& child : children) {
            edges.emplace_back(node, child); // Add the parent-child pair as an edge
        }
    }

    return edges;
}

amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;

    // Parameters based on Exercise 2 requirements
    double r = 0.5; // Step size
    double p_goal = 0.05; // Goal bias probability
    size_t maxIterations = 7500; // Maximum number of iterations
    double epsilon = 0.25; // Termination radius

    std::unordered_map<amp::Node, Eigen::Vector2d> nodePositions;
    std::unordered_map<amp::Node, amp::Node> parentMap; // To track parent-child relationships
    auto graphPtr = std::make_shared<amp::Graph<double>>(); // Create a shared pointer for the RRT tree

    CollisionChecker checker;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Define start and goal nodes
    amp::Node startNode = 0;

    // Add the start position as the root of the RRT
    Eigen::Vector2d startPosition(problem.q_init.x(), problem.q_init.y());
    graphPtr->connect(startNode, startNode, 0.0); // Connect start node to itself
    nodePositions[startNode] = startPosition;

    // Random number generation setup
    std::random_device rd;  // seed generate
    std::mt19937 gen(rd()); // Use a fixed seed for reproducibility
    std::uniform_real_distribution<> disX(problem.x_min, problem.x_max);
    std::uniform_real_distribution<> disY(problem.y_min, problem.y_max);
    std::uniform_real_distribution<> disProb(0.0, 1.0); // For goal biasing

    // Build the RRT
    amp::Node finalNode = -1; // To store the final node that connects close to the goal

    for (size_t i = 0; i < maxIterations; ++i) {
        // Step 3: Generate a random sample
        Eigen::Vector2d randomPoint;
        if (disProb(gen) < p_goal) {
            randomPoint = Eigen::Vector2d(problem.q_goal.x(), problem.q_goal.y()); // Bias towards the goal
        } else {
            randomPoint = Eigen::Vector2d(disX(gen), disY(gen)); // Sample randomly
        }

        // Step 4: Find the nearest configuration in the tree to the random point
        int nearestNodeId = -1;
        double minDistance = std::numeric_limits<double>::infinity();

        for (const auto& [nodeId, position] : nodePositions) {
            double distance = (position - randomPoint).norm();
            if (distance < minDistance) {
                minDistance = distance;
                nearestNodeId = nodeId;
            }
        }

        // Step 5: Generate a path and move a step size towards the random point
        Eigen::Vector2d nearestPosition = nodePositions[nearestNodeId];
        Eigen::Vector2d direction = (randomPoint - nearestPosition).normalized();
        Eigen::Vector2d newPoint = nearestPosition + r * direction;

        // Step 6: Check if the new path is collision-free
        if (!checker.checkPointCollision(newPoint, problem.obstacles) &&
            !checker.checkLineCollision(nearestPosition, newPoint, problem.obstacles)) {

            // Step 7: Add the new configuration and edge to the tree
            amp::Node newNodeId = nodePositions.size(); // Assign a new ID to the node
            nodePositions[newNodeId] = newPoint;
            graphPtr->connect(nearestNodeId, newNodeId, (newPoint - nearestPosition).norm());

            // Track the parent-child relationship
            parentMap[newNodeId] = nearestNodeId;

            // Step 9: Check if we are within epsilon of the goal
            if ((newPoint - Eigen::Vector2d(problem.q_goal.x(), problem.q_goal.y())).norm() < epsilon) {
                amp::Node goalNode = newNodeId + 1;
                graphPtr->connect(newNodeId, goalNode, epsilon); // Connect to goal
                nodePositions[goalNode] = Eigen::Vector2d(problem.q_goal.x(), problem.q_goal.y());
                parentMap[goalNode] = newNodeId;
                finalNode = newNodeId; // Save the final node to return the path
                std::cout << "Goal Biased RRT reached the goal." << std::endl;
                break;
            }
        }
    }
    // Reconstruct the path by tracing back from the goal to the start using `parentMap`

    std::vector<amp::Node> reversePath;
    amp::Node currentNode = finalNode;
    // std::cout <<finalNode <<std::endl;
    // std::cout <<currentNode <<std::endl;
    while (currentNode != startNode ) {
        // std::cout <<startNode <<std::endl;
        // std::cout <<finalNode <<std::endl;
        // std::cout <<currentNode <<std::endl;
        reversePath.push_back(currentNode);
        //std::cout <<"Calculating" <<std::endl;
        // Ensure we're not creating a loop or accessing an undefined parent
        if (parentMap.find(currentNode) == parentMap.end()) {
            std::cerr << "Failed to trace back path. Node " << currentNode << " is missing parent." << std::endl;
            return amp::Path2D(); // Return empty path
        }

        currentNode = parentMap[currentNode];
    }
    //reversePath.push_back(problem.q_init);
    // reversePath.push_back(startNode);

    // Reverse the path to get it from start to goal
    std::reverse(reversePath.begin(), reversePath.end());
    path.waypoints.push_back(problem.q_init);
    // Convert to waypoints: Ensure all waypoints are from the sampled nodes
    for (const auto& nodeId : reversePath) {
        path.waypoints.push_back(nodePositions[nodeId]);
    }

    std::cout << "Generated RRT Tree and traced path successfully through sampled points." << std::endl;
    double path_length = 0.0;
    for (size_t i = 1; i < path.waypoints.size(); ++i) {
        path_length += (path.waypoints[i - 1] - path.waypoints[i]).norm();
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> computation_duration = end_time - start_time;
    double computation_time = computation_duration.count(); 
    std::cout << "RRT Computation Time: " << computation_time << " seconds" << std::endl;
    std::cout << "Total Path Length: " << path_length << std::endl;
    path.waypoints.push_back(problem.q_goal);
    return path;

}
