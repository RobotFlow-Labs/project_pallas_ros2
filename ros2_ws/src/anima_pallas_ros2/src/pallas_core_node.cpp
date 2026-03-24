#include <anima_pallas_ros2/core_runtime.hpp>
#include <anima_pallas_ros2/runtime_node.hpp>

#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<anima::pallas::PallasRuntimeNode<anima::pallas::PallasCoreRuntime>>(
      "anima_pallas_core_node",
      "/pallas/core"));
  rclcpp::shutdown();
  return 0;
}
