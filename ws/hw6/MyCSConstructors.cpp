#include "MyCSConstructors.h"
#include <queue>
#include <fstream>
#include <random>
////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/
std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // get boundary values, and resolution.
    double x_min, x_max, y_min, y_max;
    std::size_t x_cells, y_cells;

    std::tie(x_min, x_max) = x0Bounds();  
    std::tie(y_min, y_max) = x1Bounds();  
    std::tie(x_cells, y_cells) = size(); 
    std::size_t grid_resolution = x_cells;

    std::size_t cell_x = static_cast<std::size_t>((x0 - x_min) / (x_max - x_min) * grid_resolution);
    std::size_t cell_y = static_cast<std::size_t>((x1 - y_min) / (y_max - y_min) * grid_resolution);

    // Boundary correction 
    if (cell_x == grid_resolution) cell_x = grid_resolution - 1;
    if (cell_y == grid_resolution) cell_y = grid_resolution - 1;

    return {cell_x, cell_y};
}
// cspace - > rspace
std::pair<double, double> MyGridCSpace2D::getPointFromCell(int i, int j) const {
    double x_min, x_max, y_min, y_max;
    std::size_t x_cells, y_cells;
    
    std::tie(x_min, x_max) = x0Bounds();  // bound space
    std::tie(y_min, y_max) = x1Bounds();
    std::tie(x_cells, y_cells) = size();    // get number of cells in each dim

    double step_x = (x_max - x_min) / x_cells;
    double step_y = (y_max - y_min) / y_cells;
    double x = i * step_x + x_min; 
    double y = j * step_y + y_min;
    return {x, y};
}
double MyManipulatorCSConstructor::crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() * b.y() - a.y() * b.x();
}
bool MyManipulatorCSConstructor::doLineSegmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                             const Eigen::Vector2d& q1, const Eigen::Vector2d& q2) {
    Eigen::Vector2d r = p2 - p1;
    Eigen::Vector2d s = q2 - q1;
    double rxs = crossProduct(r, s);
    Eigen::Vector2d qp = q1 - p1;
    double qpxr = crossProduct(qp, r);
    const double EPSILON = 1e-9;

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
}

bool MyManipulatorCSConstructor::checkCollision(const Eigen::Vector2d& seg_p1, const Eigen::Vector2d& seg_p2,
            const amp::Polygon& polygon) {
    const std::vector<Eigen::Vector2d>& vertices = polygon.verticesCW();
    std::size_t num_vertices = vertices.size();
    for (std::size_t i = 0; i < num_vertices; ++i) {
        const Eigen::Vector2d& poly_p1 = vertices[i];
        const Eigen::Vector2d& poly_p2 = vertices[(i + 1) % num_vertices];
        //std::cout << "Vertex " << i << ": (" << poly_p1.x() << ", " << poly_p1.y() << ")" << std::endl;
        if (doLineSegmentsIntersect(seg_p1, seg_p2, poly_p1, poly_p2)) {
            return true;  
        }
    }
    return false;  
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0.0,2 * M_PI,0.0,2*M_PI);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    
    // Bound space setup using std::tie
    double x_min, x_max, y_min, y_max;
    std::size_t x_cells, y_cells;

    // Use std::tie to extract values from x0Bounds, x1Bounds, and size
    std::tie(x_min, x_max) = cspace.x0Bounds();  
    std::tie(y_min, y_max) = cspace.x1Bounds();  
    std::tie(x_cells, y_cells) = cspace.size(); 
    std::cout << x_min << ", " << x_max <<std::endl;
    std::cout << x_cells << ", " << y_cells <<std::endl;
    // Link Lengths and manipulator setup
    std::vector<Eigen::Vector2d> joint_locations;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> manipulator_segments;
    std::size_t num_joints = manipulator.nLinks(); 
    amp::ManipulatorState state(2);

    // Loop over all cells in the grid
    double step_size = (x_max - x_min) / x_cells;
    int ii = 0;
    std::cout << "step_size : " << step_size <<std::endl; 
    for (double x0 = x_min; x0 <= x_max ; x0 += step_size) {  
        for (double x1 = y_min; x1 <= y_max ; x1 += step_size) {
            joint_locations.clear();
            manipulator_segments.clear();
            state << x0, x1;
            //std::cout << x0 << "  " << x1 << std::endl;
            for (std::size_t joint_index = 0; joint_index <= num_joints; joint_index += 1) {
                Eigen::Vector2d joint_location = manipulator.getJointLocation(state, static_cast<uint32_t>(joint_index));
                joint_locations.push_back(joint_location);
            }
           
            for (std::size_t k = 0; k < joint_locations.size() - 1; ++k) {
                manipulator_segments.emplace_back(joint_locations[k], joint_locations[k + 1]);
            }
        bool collision = false;
        for (const auto& segment : manipulator_segments) {
            const Eigen::Vector2d& seg_p1 = segment.first;
            const Eigen::Vector2d& seg_p2 = segment.second;

            for (const auto& obstacle : env.obstacles) {
                if (checkCollision(seg_p1, seg_p2, obstacle)) {
                    collision = true;
                }
            }
            if (seg_p1[0]<= env.x_min or seg_p1[0] >= env.x_max or seg_p2[0]<= env.x_min or seg_p2[0] >= env.x_max){
                collision = true;
            }
            if (seg_p1[1]<= env.y_min or seg_p1[1] >= env.y_max or seg_p2[1]<= env.y_min or seg_p2[1] >= env.y_max){
                collision = true;
            }
            if (collision) {
                break;
            }
        }
        std::pair<std::size_t, std::size_t> cellIndex = cspace.getCellFromPoint(x0, x1);
        cspace(cellIndex.first,cellIndex.second) = collision;
    }
    }

    // Return the unique pointer of the custom grid space object
    return cspace_ptr;
}
// Helper function to check if a point lies on an edge of the polygon
bool MyPointAgentCSConstructor::isPointOnEdge(const Eigen::Vector2d& point, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, double epsilon = 1e-9) {
    // Compute cross product to determine if point is collinear with edge
    Eigen::Vector2d edge = v2 - v1;
    Eigen::Vector2d to_point = point - v1;
    double cross_product = edge.x() * to_point.y() - edge.y() * to_point.x();

    // Check if point is collinear with edge (within a small epsilon)
    if (std::abs(cross_product) > epsilon) {
        return false;
    }

    // Check if the point is within the bounds of the edge segment
    double dot_product = (point - v1).dot(v2 - v1);
    double edge_length_squared = edge.squaredNorm();

    return dot_product >= 0 && dot_product <= edge_length_squared;
}

// Ray-casting algorithm to determine if a point is inside a polygon
bool MyPointAgentCSConstructor::isPointInsidePolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& vertices) {
    bool inside = false;
    std::size_t num_vertices = vertices.size();

    for (std::size_t i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
        const Eigen::Vector2d& v1 = vertices[i];
        const Eigen::Vector2d& v2 = vertices[j];

        // Check if the ray crosses the edge
        if ((v1.y() > point.y()) != (v2.y() > point.y()) &&
            (point.x() < (v2.x() - v1.x()) * (point.y() - v1.y()) / (v2.y() - v1.y()) + v1.x())) {
            inside = !inside;
        }
    }

    return inside;
}

// Combined function for edge inclusion and ray-casting test
bool MyPointAgentCSConstructor::checkPointInPolygon(const Eigen::Vector2d& point, const amp::Polygon& polygon) {
    const std::vector<Eigen::Vector2d>& vertices = polygon.verticesCW(); // Assuming verticesCW() returns vertices in clockwise order
    std::size_t num_vertices = vertices.size();

    // First, check if the point lies on any of the polygon's edges
    for (std::size_t i = 0; i < num_vertices; ++i) {
        const Eigen::Vector2d& v1 = vertices[i];
        const Eigen::Vector2d& v2 = vertices[(i + 1) % num_vertices];

        if (isPointOnEdge(point, v1, v2)) {
            return true;  // The point is on the boundary (edge)
        }
    }

    // If the point is not on an edge, use ray-casting to determine if it's inside the polygon
    return isPointInsidePolygon(point, vertices);
}

//////////////////////////////////////////////////////////////

std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    Eigen::Vector2d state;
    // Loop over all cells in the grid
    // c-space of point is x,y
    const std::vector<std::pair<int, int>> directions = {{-1,0},{0,-1},{1,0},{0,1}};//{{0,1},{1,0},{0,-1},{-1,0}};
    
    double x_step_size = (env.x_max - env.x_min) / (m_cells_per_dim);
    double y_step_size = (env.y_max - env.y_min) / (m_cells_per_dim);
    std::cout << "step size(point) : " << x_step_size << std::endl;
    for (double x = env.x_min; x < env.x_max ; x += x_step_size) {
        for (double y = env.y_min; y < env.y_max ; y += y_step_size) {
            bool collision = false;
            state << x,y;
            for (const auto& obstacle : env.obstacles) {
                if (checkPointInPolygon(state, obstacle)) {
                    collision = true;
                }
                std::pair<std::size_t, std::size_t> cellIndex = cspace.getCellFromPoint(x,y);
                if (cellIndex.first >= m_cells_per_dim || cellIndex.second >= m_cells_per_dim) {
                    std::cerr << "Error: Cell index out of bounds! (" << cellIndex.first << ", " << cellIndex.second << ")" << std::endl;
                    continue;
                }
                //std::cout << m_cells_per_dim <<std::endl;
                cspace(cellIndex.first,cellIndex.second) = collision;
                // if (collision){
                // for (const auto& dir : directions) {
                //     int neighborX = cellIndex.first + dir.first;
                //     int neighborY = cellIndex.second + dir.second;
                //     std::cout << "Current : "<<cellIndex.first << ", "<< cellIndex.second <<std::endl;
                //     std::cout << neighborX << ", " << neighborY <<std::endl;
                //     if (neighborX >= 0 && neighborX < m_cells_per_dim && neighborY >= 0 && neighborY < m_cells_per_dim) {
                //         cspace(neighborX,neighborY) = collision;
                //     }
                // }
                // }
                
                //std::cout << "Point: (" << x << ", " << y << "), Cell Index: (" << cellIndex.first << ", " << cellIndex.second << ")" << std::endl;
                //std::cout << "Collision: " << collision << std::endl;
                if(collision){
                    break;
                }
                
            }
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}


void MyWaveFrontAlgorithm::savetocsv(const std::vector<std::vector<int>>& grid_wv, const std::string& filename) {
    std::ofstream file(filename);
    
    if (file.is_open()) {
        for (const auto& row : grid_wv) {
            for (size_t i = 0; i < row.size(); ++i) {
                file << row[i];
                if (i != row.size() - 1) {
                    file << ","; // Add comma between values
                }
            }
            file << "\n"; // Newline after each row
        }
        file.close();
        std::cout << "Grid saved to " << filename << " successfully." << std::endl;
    } else {
        std::cerr << "Unable to open file " << filename << std::endl;
    }
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator){
    double x_min, x_max, y_min, y_max;
    amp::Path2D path;
    
    const MyGridCSpace2D& my_grid_cspace = static_cast<const MyGridCSpace2D&>(grid_cspace);
    std::tie(x_min, x_max) = my_grid_cspace.x0Bounds();  
    std::tie(y_min, y_max) = my_grid_cspace.x1Bounds(); 
    Eigen::Vector2d new_q_goal = q_goal; 
    path.waypoints.push_back(q_init);
    
    if(isManipulator) {
        std::cout << "Manipulator search" << std::endl;
        std::cout << "q_init: " << q_init.x() << ", " << q_init.y() << std::endl;
        if (new_q_goal == Eigen::Vector2d(0.0, 0.0)) {
            new_q_goal = Eigen::Vector2d(2 * M_PI, 0.0);  // (0,0) -> (2*pi, 0)로 변경
        }
        std::cout << "q_goal: " << new_q_goal.x() << ", " << new_q_goal.y() << std::endl;
    }

    size_t Numcells = my_grid_cspace.size().first;
    std::vector<std::vector<int>> grid_wv(Numcells, std::vector<int>(Numcells, 0));
    double step_size = (x_max - x_min) / Numcells;

    std::pair<std::size_t, std::size_t> cellGoal = my_grid_cspace.getCellFromPoint(new_q_goal.x(), new_q_goal.y());
    grid_wv[cellGoal.first][cellGoal.second] = 2;

    std::queue<std::pair<std::size_t, std::size_t>> q;
    q.push(cellGoal);
    const std::vector<std::pair<int, int>> directions = {{-1,0},{0,-1},{1,0},{0,1}};
    
    for (double x = x_min; x < x_max; x += step_size) {
        for (double y = y_min; y < y_max; y += step_size) {
            std::pair<std::size_t, std::size_t> cellIndex = my_grid_cspace.getCellFromPoint(x, y);
            if (my_grid_cspace(cellIndex.first, cellIndex.second) == true) {
                grid_wv[cellIndex.first][cellIndex.second] = 1;
                for (const auto& dir : directions) {
                    int neighborX = cellIndex.first + dir.first;
                    int neighborY = cellIndex.second + dir.second;
                    if (neighborX >= 0 && neighborX < Numcells && neighborY >= 0 && neighborY < Numcells) {
                        grid_wv[neighborX][neighborY] = 1;
                    }
                }
            }
        }
    }

    while (!q.empty()) {
        auto current = q.front();
        q.pop();
        int currentValue = grid_wv[current.first][current.second];
        for (const auto& dir : directions) {
            int neighborX = current.first + dir.first;
            int neighborY = current.second + dir.second;
            if (neighborX >= 0 && neighborX < Numcells && neighborY >= 0 && neighborY < Numcells) {
                if (grid_wv[neighborX][neighborY] == 0) {
                    grid_wv[neighborX][neighborY] = currentValue + 1;
                    q.push({neighborX, neighborY});
                }
            }
        }
    }

    std::pair<std::size_t, std::size_t> cellStart = my_grid_cspace.getCellFromPoint(q_init.x(), q_init.y());
    auto current = cellStart;
    size_t i = 0;
    bool solution_found = false;
    
    while (i < 1000) {
        if (current == cellGoal) {
            solution_found = true;
            break;
        }

        std::pair<double, double> waypoint = my_grid_cspace.getPointFromCell(current.first, current.second);
        path.waypoints.push_back(Eigen::Vector2d(waypoint.first, waypoint.second));

        int minValue = grid_wv[current.first][current.second];
        std::pair<std::size_t, std::size_t> nextCell = current;

        for (const auto& dir : directions) {
            int neighborX = current.first + dir.first;
            int neighborY = current.second + dir.second;
            if (neighborX >= 0 && neighborX < Numcells && neighborY >= 0 && neighborY < Numcells) {
                if (grid_wv[neighborX][neighborY] < minValue && grid_wv[neighborX][neighborY] > 1) {
                    minValue = grid_wv[neighborX][neighborY];
                    nextCell = {neighborX, neighborY};
                }
            }
        }
        current = nextCell;
        i++;
    }

    // If no solution is found after 1000 iterations, retry with adjusted goal
    if (!solution_found) {
        std::cout << "Solution not found in 1000 iterations, adjusting goal and retrying..." << std::endl;
        if (new_q_goal == Eigen::Vector2d(2 * M_PI, 0.0)) {
            new_q_goal = Eigen::Vector2d(0.0, 0.0);  // Adjust goal to (2*pi, 0)
        }

        // Restart the wavefront planning process
        //return planInCSpace(q_init, new_q_goal, grid_cspace, isManipulator);  // Recursive call with new goal
    }

    return path;  // Return the final path if solution is found
}
