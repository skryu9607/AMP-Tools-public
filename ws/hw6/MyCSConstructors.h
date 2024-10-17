#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW4.h"
#include "hw/HW6.h"

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

// Derive the amp::GridCSpace2D class and override the missing method
class MyGridCSpace2D : public amp::GridCSpace2D {
    public:
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max),
            m_x0_cells(x0_cells),x0_min(x0_min),x0_max(x0_max),x1_min(x1_min),x1_max(x1_max)// Call base class constructor
        {}

        // Override this method for determining which cell a continuous point belongs to
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;
        virtual std::pair<double, double>getPointFromCell(int i, int j) const ;
        // Getter methods for accessing the bounds and the number of cells
        double getXMin() const { return x0_min; }
        double getXMax() const { return x0_max; }
        double getYMin() const { return x1_min; }
        double getYMax() const { return x1_max; }

        // Getter methods for number of cells in each dimension
        std::size_t getXCells() const { return m_x0_cells; }

    private:
    std::size_t m_x0_cells;
    double x0_min;
    double x0_max;
    double x1_min;
    double x1_max;
};

class MyManipulatorCSConstructor : public amp::ManipulatorCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyManipulatorCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;
        double crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b);
        // doLineSegmentsIntersect 함수 선언
        bool doLineSegmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                             const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
        bool checkCollision(const Eigen::Vector2d& seg_p1, const Eigen::Vector2d& seg_p2,
            const amp::Polygon& polygon);

    private:
        std::size_t m_cells_per_dim;
};

//////////////////////////////////////////////////////////////

// Derive the PointAgentCSConstructor class and override the missing method
class MyPointAgentCSConstructor : public amp::PointAgentCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyPointAgentCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}
        bool isPointOnEdge(const Eigen::Vector2d& point, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, double epsilon);
        bool isPointInsidePolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& vertices);
        bool checkPointInPolygon(const Eigen::Vector2d& point, const amp::Polygon& polygon);
        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::Environment2D& env) override;
        //bool isPointOnEdge(const Eigen::Vector2d& point, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, double epsilon = 1e-9);
    private:
        std::size_t m_cells_per_dim;
};

class MyWaveFrontAlgorithm : public amp::WaveFrontAlgorithm {
    public:
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) override;
        void savetocsv(const std::vector<std::vector<int>>& grid_wv, const std::string& filename);
};
