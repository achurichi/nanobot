#include "rclcpp/rclcpp.hpp"

#include <memory>

#include "nanobot_imu/ImuNode.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto imu_node = std::make_shared<nanobot_imu::ImuNode>();
  rclcpp::spin(imu_node);

  rclcpp::shutdown();

  return 0;
}
