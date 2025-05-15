#include "least_squares_fitter/calculator_node.h"

DistanceEstimatorNode::DistanceEstimatorNode(const rclcpp::NodeOptions & options)
: Node("least_squares_fitter", options)
{
    double relative_dist = 234;  // depth median
    double real_x = 23.11; // cm
    double abs_dist = quadraticFunction(relative_dist);
    double angle = calculateClampedAngle(real_x, abs_dist, max_x);

    RCLCPP_INFO(this->get_logger(), "Estimated Distance: %.2f cm, Servo Angle: %.2f deg", abs_dist, angle);

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("hw_px3", 10);
    std_msgs::msg::Float64 msg;
    msg.data = angle;
    publisher_->publish(msg);
}

    double DistanceEstimatorNode::quadraticFunction(double x) 
    {
        double a = 0.00143026;
        double b = 0.32379224;
        double c = -56.7942361;
        return a * std::pow(x, 2) + b * x + c;
    }

    double DistanceEstimatorNode::calculateClampedAngle(double x, double z, double max_x) {
        // Clamp x to the range [0, max_x]
        x = std::max(0.0, std::min(x, max_x));

        // Prevent division by zero
        if (z == 0.0) return 90.0;

        // Compute angle in radians and convert to degrees
        double angle_rad = atan(x / z);
        return angle_rad * (180.0 / M_PI);
    }

// main function similar to inference.cpp
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceEstimatorNode>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

