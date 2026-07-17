#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_srvs/srv/set_bool.hpp>

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using SetBool = std_srvs::srv::SetBool;

class MultiServiceNode : public rclcpp::Node
{
public:
  MultiServiceNode()
  : Node("multi_service_node")
  {
    // 创建第一个服务：AddTwoInts
    add_service_ = this->create_service<AddTwoInts>(
      "add_two_ints",
      std::bind(&MultiServiceNode::handle_add, this,
                std::placeholders::_1, std::placeholders::_2));

    // 创建第二个服务：SetBool
    set_bool_service_ = this->create_service<SetBool>(
      "set_flag",
      [this](const std::shared_ptr<SetBool::Request> request,
             std::shared_ptr<SetBool::Response> response) {
        // 简单示例：将请求中的 data 作为 internal_flag 并回传成功
        this->internal_flag_ = request->data;
        response->success = true;
        response->message = this->internal_flag_ ? "flag set to true" : "flag set to false";
        RCLCPP_INFO(this->get_logger(), "set_flag called: %s", response->message.c_str());
      });

    RCLCPP_INFO(this->get_logger(), "MultiServiceNode ready: offers 'add_two_ints' and 'set_flag'");
  }

private:
  // AddTwoInts 回调
  void handle_add(const std::shared_ptr<AddTwoInts::Request> request,
                  std::shared_ptr<AddTwoInts::Response> response)
  {
    // 简单相加
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "add_two_ints: %ld + %ld = %ld",
                request->a, request->b, response->sum);
  }

  rclcpp::Service<AddTwoInts>::SharedPtr add_service_;
  rclcpp::Service<SetBool>::SharedPtr set_bool_service_;
  bool internal_flag_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 可选：如果服务处理可能耗时，使用多线程执行器允许并发处理请求
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<MultiServiceNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
