#include <anima_pallas_ros2/ct_runtime.hpp>
#include <anima_pallas_ros2/runtime_node.hpp>

#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<anima::pallas::PallasRuntimeNode<anima::pallas::PallasCtRuntime>>(
      "anima_pallas_ct_node",
      "/pallas/ct"));
  rclcpp::shutdown();
  return 0;
}
