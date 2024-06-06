#include "Node.hpp"

#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DonatelloNode>());
    rclcpp::shutdown();
    return 0;
}
