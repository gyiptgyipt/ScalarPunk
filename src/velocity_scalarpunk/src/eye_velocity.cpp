#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scalarpunk_interfaces/srv/emotion_command.hpp"

class EyeVelocity : public rclcpp::Node
{
public:
  EyeVelocity()
  : Node("eye_velocity")
  {
    emotion_client_ =
      this->create_client<scalarpunk_interfaces::srv::EmotionCommand>("/robot_emotion_command");

    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&EyeVelocity::cmdVelCallback, this, std::placeholders::_1));
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::string emotion = "wakeup";

    if (msg->angular.z < 0.0) {
      emotion = "look_left";
    } else if (msg->angular.z > 0.0) {
      emotion = "look_right";
    }

    if (emotion == last_emotion_) {
      return;
    }

    if (sendEmotion(emotion)) {
      last_emotion_ = emotion;
    }
  }

  bool sendEmotion(const std::string & emotion)
  {
    if (!emotion_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for /robot_emotion_command service");
      return false;
    }

    auto request = std::make_shared<scalarpunk_interfaces::srv::EmotionCommand::Request>();
    request->emotion = emotion;

    emotion_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Eye emotion: %s", emotion.c_str());
    return true;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Client<scalarpunk_interfaces::srv::EmotionCommand>::SharedPtr emotion_client_;
  std::string last_emotion_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EyeVelocity>());
  rclcpp::shutdown();
  return 0;
}
