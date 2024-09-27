#include "CSpaceSkeleton.h"

// Override this method for returning whether or not a point is in collision

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    const double min_value = 0.0;
    const double max_value = 2.0 * M_PI;
    std::size_t grid_resolution = m_x0_cells;
    // Ensure x0 and x1 are within the range [0, 2*pi]
    x0 = std::fmod(x0, max_value);  // Wrap around if x0 > 2*pi
    x1 = std::fmod(x1, max_value);  // Wrap around if x1 > 2*pi

    // Discretize x0 and x1 into grid cells
    std::size_t cell_x = static_cast<std::size_t>((x0 - min_value) / (max_value - min_value) * grid_resolution);
    std::size_t cell_y = static_cast<std::size_t>((x1 - min_value) / (max_value - min_value) * grid_resolution);

    // Boundary correction (if the point exactly hits 2*pi, assign it to the last cell)
    if (cell_x == grid_resolution) cell_x = grid_resolution - 1;
    if (cell_y == grid_resolution) cell_y = grid_resolution - 1;

    return {cell_x, cell_y};
}
// 2D 벡터의 외적(cross product)을 계산하는 함수
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
        // 선분이 공선상에 있음
        double r_dot_r = r.dot(r);
        double s_dot_r = s.dot(r);
        double t0 = qp.dot(r) / r_dot_r;
        double t1 = t0 + s_dot_r / r_dot_r;

        // t0와 t1이 범위 [0,1] 내에 있는지 확인하여 겹치는지 판단
        double t_min = std::min(t0, t1);
        double t_max = std::max(t0, t1);

        if (t_max < 0 || t_min > 1) {
            // 선분이 겹치지 않음
            return false;
        } else {
            // 선분이 겹침
            return true;
        }
    }

    if (std::abs(rxs) < EPSILON && std::abs(qpxr) >= EPSILON) {
        // 선분이 평행하며 교차하지 않음
        return false;
    }

    if (std::abs(rxs) >= EPSILON) {
        double t = crossProduct(qp, s) / rxs;
        double u = crossProduct(qp, r) / rxs;
        if (t >= 0 - EPSILON && t <= 1 + EPSILON && u >= 0 - EPSILON && u <= 1 + EPSILON) {
            // 선분이 교차함
            return true;
        }
    }

    // 교차하지 않음
    return false;
}
// 선분과 다각형의 충돌 여부를 확인하는 함수 (crossProduct 함수 사용)
bool MyManipulatorCSConstructor::checkCollision(const Eigen::Vector2d& seg_p1, const Eigen::Vector2d& seg_p2,
            const amp::Polygon& polygon) {
    const std::vector<Eigen::Vector2d>& vertices = polygon.verticesCCW();
    std::size_t num_vertices = vertices.size();

    // 다각형의 각 변과 선분의 교차 여부 확인
    for (std::size_t i = 0; i < num_vertices; ++i) {
        const Eigen::Vector2d& poly_p1 = vertices[i];
        const Eigen::Vector2d& poly_p2 = vertices[(i + 1) % num_vertices];

        if (doLineSegmentsIntersect(seg_p1, seg_p2, poly_p1, poly_p2)) {
            return true;  // 충돌 발생
        }
    }

    return false;  // 충돌하지 않음
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Link Lengths. 
    std::vector<Eigen::Vector2d> joint_locations;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> manipulator_segments;
    std::size_t num_joints = manipulator.nLinks(); 
    amp::ManipulatorState state(2);

    // Loop over all cells in the grid
    for (double x0 = 0; x0 < 2 * M_PI; x0 +=0.1) {
        for (double  x1 = 0; x1 < 2 * M_PI; x1 +=0.1) {
            joint_locations.clear();
            manipulator_segments.clear();
            state << x0, x1;
            std::cout << x0 << "  " << x1 << std::endl;
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
            if (collision) {
                break;
            }
        }
        std::pair<std::size_t, std::size_t> cellIndex = cspace.getCellFromPoint(x0, x1);
        cspace(cellIndex.first,cellIndex.second) = collision;
    }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    
    return cspace_ptr;

}
