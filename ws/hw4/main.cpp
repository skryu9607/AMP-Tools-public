// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;
// void problem2b(){
//     Vector2d endEffector(2,0)
//     MyManipulator2D manipulator({1.0,0.5,1.0}})
//     amp::ManiuplatorState manipulator.
// }
int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());
    std::vector<double> link_lengths1 = {0.5, 1.0, 0.5};
    MyManipulator2D manipulator(link_lengths1);

    // You can visualize your manipulator given an angle state like so:
    amp::ManipulatorState test_state(3);  // 
    test_state << M_PI / 3, M_PI / 6, 7.0 / 4.0 * M_PI; 
    // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    Visualizer::makeFigure(manipulator, test_state); 


    std::vector<double> link_lengths2 = {1.0, 0.5, 1.0};
    MyManipulator2D manipulator_2b(link_lengths2);

    // Print link lengths to verify
    std::cout << "Link lengths before IK: ";
    for (const auto& length : manipulator_2b.getLinkLengths()) {
        std::cout << length << " ";
    }
    std::cout << std::endl;
    Eigen::Vector2d end_effector_location = {2,0};
    // Call the inverse kinematics function
    amp::ManipulatorState test_IK_state = manipulator_2b.getConfigurationFromIK(end_effector_location);

    // Print link lengths again to ensure they haven't changed
    std::cout << "Link lengths after IK: ";
    for (const auto& length : manipulator_2b.getLinkLengths()) {
        std::cout << length << " ";
    }
    std::cout << std::endl;

    Visualizer::makeFigure(manipulator_2b, test_IK_state); 

    // Create the collision space constructor
    std::size_t n_cells = 1000;
    MyManipulatorCSConstructor cspace_constructor(n_cells);
    std::vector<double> link_lengths3 = {1.0, 1.0};
    MyManipulator2D manipulator_2c(link_lengths3);
    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator_2c, HW4::getEx3Workspace2());

    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace);

    Visualizer::showFigures();

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "seungkeol.ryu@colorado.edu", argc, argv);
    return 0;
}
