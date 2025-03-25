#include "nanobot_camera/DockerRunnerNode.hpp"

#include <array>
#include <cstdio>
#include <iostream>
#include <memory>

namespace nanobot_camera
{
  DockerRunnerNode::DockerRunnerNode()
      : Node("docker_runner_node"), container_id_("")
  {
    RCLCPP_INFO(this->get_logger(), "Starting RealSense camera Docker container...");

    // TODO : show container logs
    std::string command = "sudo docker run \
      --privileged \
      --user 'nano' \
      --network host \
      --pid host \
      --ipc host \
      --runtime nvidia \
      --rm \
      --detach \
      realsense_camera";
    container_id_ = exec(command.c_str());

    if (container_id_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Error starting container");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Container started with id: %s", container_id_.c_str());
  }

  DockerRunnerNode::~DockerRunnerNode()
  {
    if (!container_id_.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Stopping container...");

      // TODO: Stop gracefully
      std::string command = "sudo docker stop " + container_id_;
      exec(command.c_str());

      RCLCPP_INFO(this->get_logger(), "Container stopped");
    }
  }

  std::string DockerRunnerNode::exec(const char *cmd)
  {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);

    if (!pipe)
    {
      RCLCPP_ERROR(this->get_logger(), "Error executing command: %s", cmd);
      return "";
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
      result += buffer.data();
    }

    return result;
  }
}
