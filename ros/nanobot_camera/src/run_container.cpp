#include "rclcpp/rclcpp.hpp"

#include <memory>

#include "nanobot_camera/DockerRunnerNode.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto docker_runner_node = std::make_shared<nanobot_camera::DockerRunnerNode>();
  rclcpp::spin(docker_runner_node);

  rclcpp::shutdown();

  return 0;
}
