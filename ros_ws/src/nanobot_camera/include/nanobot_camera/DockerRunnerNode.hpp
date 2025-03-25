#ifndef NANOBOT_CAMERA__DOCKER_RUNNER_NODE_HPP_
#define NANOBOT_CAMERA__DOCKER_RUNNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace nanobot_camera
{
  class DockerRunnerNode : public rclcpp::Node
  {
  public:
    DockerRunnerNode();
    ~DockerRunnerNode();

  private:
    std::string exec(const char *cmd);

    std::string container_id_;
  };
} // namespace nanobot_camera

#endif // NANOBOT_CAMERA__DOCKER_RUNNER_NODE_HPP_