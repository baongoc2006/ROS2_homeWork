#include <rclcpp/rclcpp.hpp>
#include "distance_warning/srv/set_threshold.hpp"

using SetThreshold = distance_warning::srv::SetThreshold;

class SetThresholdService : public rclcpp::Node
{
public:
  SetThresholdService() : Node("set_threshold_service")
  {
    this->declare_parameter<double>("threshold", 0.5);

    // Tạo service server
    service_ = this->create_service<SetThreshold>(
      "set_threshold",
      std::bind(&SetThresholdService::handleSetThreshold, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "Service '/set_threshold' is ready. "
      "Call with increase=true to INCREASE, increase=false to DECREASE.");
  }

private:
  void handleSetThreshold(
    const SetThreshold::Request::SharedPtr request,
    SetThreshold::Response::SharedPtr response)
  {
    constexpr double MIN_THRESHOLD = 0.1;
    constexpr double MAX_THRESHOLD = 1.5;
    constexpr double STEP          = 0.1;

    double current       = this->get_parameter("threshold").as_double();
    double new_threshold = current;

    // Tăng hoặc giảm theo request->increase
    if (request->increase) {
      new_threshold = current + STEP;
    } else {
      new_threshold = current - STEP;
    }

    // Clamp trong khoảng [MIN_THRESHOLD, MAX_THRESHOLD]
    new_threshold = std::max(MIN_THRESHOLD, std::min(MAX_THRESHOLD, new_threshold));

    // Làm tròn 1 chữ số thập phân tránh floating-point drift
    new_threshold = std::round(new_threshold * 10.0) / 10.0;

    // Cập nhật parameter
    this->set_parameter(rclcpp::Parameter("threshold", new_threshold));

    // Set response
    response->success       = true;
    response->new_threshold = new_threshold;
    response->message       = (request->increase ? "Threshold increased to " : "Threshold decreased to ")
                              + std::to_string(new_threshold).substr(0, 4) + " m";

    RCLCPP_INFO(this->get_logger(), "Threshold updated: %.2f -> %.2f m",
                current, new_threshold);
  }

  rclcpp::Service<SetThreshold>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetThresholdService>());
  rclcpp::shutdown();
  return 0;
}
