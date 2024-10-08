#include "ManipulatorSkeleton.h"
#include <cmath>

MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0,1.0}) // Default to a 2-link with all links of 1.0 length
    //: LinkManipulator2D({0.5,1.0,0.5})
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)
    // Access and print values using parentheses
    if (state.size() != nLinks()) {
        std::cerr << "State vector size (" << state.size() << ") does not match the number of links (" << nLinks() << ")." << std::endl;
        return Eigen::Vector2d::Zero();
    }
    // Check if joint_index is valid
    if (joint_index > nLinks()) {
        //std::cerr << "Joint index " << joint_index << " out of range." << std::endl;
        return Eigen::Vector2d::Zero();
    }
    std::vector<Eigen::Vector2d> joint_positions;
    // Joint positions initialization (base position)
    Eigen::Vector2d current_position(0.0, 0.0);  // Base of the manipulator

    for (int i = 0; i < state.size(); i++) {
        //std::cout << "state(" << i << ") = " << state(i) << std::endl;
    }
    // Initialize angles and positions for the joints
    double cumulative_angle = 0.0;

    // Compute the position of each joint up to joint_index
    for (uint32_t i = 0; i <= joint_index; i++) {
        cumulative_angle += state(i);  // Accumulate joint angles up to joint i
        joint_positions.push_back(current_position);
        // Print debug info for each joint
        //std::cout << "Joint " << i << " position: (" << current_position.x() << ", " << current_position.y() << ")" << std::endl;
        // Link length for the current link
        double link_length = m_link_lengths[i];

        // Compute the new position of the joint
        current_position.x() += link_length * cos(cumulative_angle);
        current_position.y() += link_length * sin(cumulative_angle);


    }
    return joint_positions[joint_index];
}


// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here
    double end_x = end_effector_location.x();
    double end_y = end_effector_location.y();
    const std::vector<double>& link_lengths = getLinkLengths();
    for (int i = 0; i < link_lengths.size(); i++) {
        std::cout << "LinkLengths(" << i << ") = " << link_lengths[i] << std::endl;
    }
    amp::ManipulatorState joint_angles;
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
    double a1 = link_lengths[0];
    double a2 = link_lengths[1];
    // For angle 2
    double cos_value2_1 = 1/(2*a1*a2) * ((end_x * end_x + end_y * end_y) - (a1 * a1 + a2 * a2)) ;
    double sin_value2_1 = sqrt(1 - cos_value2_1 * cos_value2_1); 
    double angle2_1 = atan2(sin_value2_1,cos_value2_1);
    double angle2_2 = 2 * M_PI - angle2_1;
    // For angle 1
    double cos_value1_1 = (end_x * (a1 + a2 * cos_value2_1) + end_y * a2 * sqrt(1 - cos_value2_1 * cos_value2_1))/(end_x * end_x + end_y * end_y);
    double sin_value1_1 = (end_y * (a1 + a2 * cos_value2_1) - end_x * a2 * sqrt(1 - cos_value2_1 * cos_value2_1))/(end_x * end_x + end_y * end_y);
    double angle1_1 = atan2(sin_value1_1,cos_value1_1);

    double cos_value1_2 = (end_x * (a1 + a2 * cos_value2_1) - end_y * a2 * sqrt(1 - cos_value2_1 * cos_value2_1))/(end_x * end_x + end_y * end_y);
    double sin_value1_2 = (end_y * (a1 + a2 * cos_value2_1) + end_x * a2 * sqrt(1 - cos_value2_1 * cos_value2_1))/(end_x * end_x + end_y * end_y);
    double angle1_2 = atan2(sin_value1_2,cos_value1_2);

    amp::ManipulatorState joint_angles_1(2);
    joint_angles_1 << angle1_1, angle2_1;

    amp::ManipulatorState joint_angles_2(2);
    joint_angles_2 << angle1_2, angle2_2;
    return joint_angles_1;

    } 
    else if (nLinks() == 3) {

    double a1 = link_lengths[0];
    double a2 = link_lengths[1];
    double a3 = link_lengths[2];
    double total_angle = M_PI/6;
    double X = end_x - a3 * cos(total_angle);
    double Y = end_y - a3 * sin(total_angle);
    // For angle 2
    double cos_value2_1 = 1/(2*a1*a2) * ((X * X+ Y * Y) - (a1 * a1 + a2 * a2)) ;
    double sin_value2_1 = sqrt(1 - cos_value2_1 * cos_value2_1); 
    // std::cout << "cos_value2_1 = " << cos_value2_1 << std::endl;
    // std::cout << "sin_value2_1 = " << sin_value2_1 << std::endl;
    double angle2_1 = atan2(sin_value2_1,cos_value2_1);
    // std::cout << "angle2_1 = " << angle2_1 << std::endl;
    double angle2_2 = 2 * M_PI - angle2_1;
    // For angle 1
    double cos_value1_1 = (X * (a1 + a2 * cos_value2_1) + Y* a2 * sqrt(1 - cos_value2_1 * cos_value2_1))/(X * X + Y * Y);
    double sin_value1_1 = (Y * (a1 + a2 * cos_value2_1) - X * a2 * sqrt(1 - cos_value2_1 * cos_value2_1))/(X * X + Y * Y);
    double angle1_1 = atan2(sin_value1_1,cos_value1_1);
    // std::cout << "angle1_1 = " << angle1_1 << std::endl;
    double cos_value1_2 = (X* (a1 + a2 * cos_value2_1) - Y * a2 * sqrt(1 - cos_value2_1 * cos_value2_1))/(X * X + Y * Y);
    double sin_value1_2 = (Y * (a1 + a2 * cos_value2_1) + X * a2 * sqrt(1 - cos_value2_1 * cos_value2_1))/(X * X + Y * Y);
    double angle1_2 = atan2(sin_value1_2,cos_value1_2);

    double angle3_1 = total_angle - (angle1_1 + angle2_1);
    double angle3_2 = total_angle - (angle1_2 + angle2_2);

    amp::ManipulatorState joint_angles(3);
    joint_angles << angle1_1, angle2_1, angle3_1;

        return joint_angles;
    } else {

        return joint_angles;
    }
    
    return joint_angles;
}
