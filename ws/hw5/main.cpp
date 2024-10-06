// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"
// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());
    
    // Test your gradient descent algorithm on a random problem.
    // d_star,zetta, Q_star, eta
    //MyGDAlgorithm algo(8.0, .8, 0.5, 1.); // k = 12;
    MyGDAlgorithm algo(3.0, 5.0, 1.0, 4.0);
    amp::Path2D path;
    amp::Problem2D prob = HW2::getWorkspace1();

    MyPotentialFunction potential_function(algo,prob);
   //bool success = HW5::generateAndCheck(algo, path, prob);
    path = algo.plan(prob);
    Visualizer::makeFigure(prob, path);
    // 초기화된 q_init과 q_goal 값을 확인
    std::cout << "Initial point: (" << prob.q_init[0] << ", " << prob.q_init[1] << ")" << std::endl;
    std::cout << "Goal point: (" << prob.q_goal[0] << ", " << prob.q_goal[1] << ")" << std::endl;
    // Visualize your potential function
    //amp::Visualizer::makeFigure(potential_function, prob, 50,true);
   // amp::Visualizer::makeFigure(potential_function, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 50,true);
    amp::Visualizer::makeFigure(potential_function, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 200);
    Visualizer::showFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("seungkeol.ryu@colorado.edu", argc, argv, 1.0, 1.0, 1.0, 1.0);
    return 0;

}
