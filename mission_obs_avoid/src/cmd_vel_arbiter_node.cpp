#include <algorithm>
#include <chrono>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class CmdVelArbiterNode : public rclcpp::Node {
 public:
  CmdVelArbiterNode() : Node("cmd_vel_arbiter") {
    offboard_cmd_topic_ =
        declare_parameter<std::string>("offboard_cmd_topic", "/offboard_stack/cmd_vel");
    interceptor_cmd_topic_ = declare_parameter<std::string>(
        "interceptor_cmd_topic", "/mission_interceptor/cmd_vel");
    override_topic_ = declare_parameter<std::string>("override_topic",
                                                      "/mission_interceptor/override_active");
    output_topic_ = declare_parameter<std::string>("output_topic",
                                                   "/mavros/setpoint_velocity/cmd_vel");

    arbiter_rate_hz_ = declare_parameter<double>("arbiter_rate_hz", 30.0);
    offboard_cmd_timeout_sec_ =
        declare_parameter<double>("offboard_cmd_timeout_sec", 0.50);
    interceptor_cmd_timeout_sec_ =
        declare_parameter<double>("interceptor_cmd_timeout_sec", 0.25);
    publish_zero_on_stale_ = declare_parameter<bool>("publish_zero_on_stale", true);
    cmd_frame_id_ = declare_parameter<std::string>("cmd_frame_id", "map");

    auto qos_cmd = rclcpp::QoS(10).reliable();
    auto qos_flag = rclcpp::QoS(10).reliable();

    offboard_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        offboard_cmd_topic_, qos_cmd,
        [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          offboard_cmd_ = *msg;
          offboard_cmd_seen_ = true;
          last_offboard_cmd_time_ = now();
        });

    interceptor_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        interceptor_cmd_topic_, qos_cmd,
        [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          interceptor_cmd_ = *msg;
          interceptor_cmd_seen_ = true;
          last_interceptor_cmd_time_ = now();
        });

    override_sub_ = create_subscription<std_msgs::msg::Bool>(
        override_topic_, qos_flag,
        [this](std_msgs::msg::Bool::SharedPtr msg) { override_active_ = msg->data; });

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, qos_cmd);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, arbiter_rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&CmdVelArbiterNode::on_timer, this));

    RCLCPP_INFO(get_logger(),
                "cmd_vel_arbiter started. offboard='%s' interceptor='%s' output='%s'",
                offboard_cmd_topic_.c_str(), interceptor_cmd_topic_.c_str(), output_topic_.c_str());
  }

 private:
  enum class Source {
    kNone,
    kOffboard,
    kInterceptor,
    kZero,
  };

  void on_timer() {
    const auto tnow = now();

    const bool offboard_fresh = offboard_cmd_seen_ &&
                                (tnow - last_offboard_cmd_time_).seconds() <=
                                    offboard_cmd_timeout_sec_;
    const bool interceptor_fresh = interceptor_cmd_seen_ &&
                                   (tnow - last_interceptor_cmd_time_).seconds() <=
                                       interceptor_cmd_timeout_sec_;

    geometry_msgs::msg::TwistStamped out;
    Source source = Source::kNone;

    if (override_active_) {
      if (interceptor_fresh) {
        out = interceptor_cmd_;
        source = Source::kInterceptor;
      } else if (publish_zero_on_stale_) {
        out = zero_cmd();
        source = Source::kZero;
      }
    } else {
      if (offboard_fresh) {
        out = offboard_cmd_;
        source = Source::kOffboard;
      } else if (publish_zero_on_stale_) {
        out = zero_cmd();
        source = Source::kZero;
      }
    }

    if (source == Source::kNone) {
      report_source(source);
      return;
    }

    out.header.stamp = tnow;
    if (out.header.frame_id.empty()) {
      out.header.frame_id = cmd_frame_id_;
    }

    cmd_pub_->publish(out);
    report_source(source);
  }

  geometry_msgs::msg::TwistStamped zero_cmd() const {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = cmd_frame_id_;
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.linear.z = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  void report_source(Source current) {
    if (current == last_source_) {
      return;
    }
    last_source_ = current;

    const char *name = "none";
    switch (current) {
      case Source::kOffboard:
        name = "offboard_stack";
        break;
      case Source::kInterceptor:
        name = "mission_interceptor";
        break;
      case Source::kZero:
        name = "zero";
        break;
      case Source::kNone:
      default:
        name = "none";
        break;
    }

    RCLCPP_INFO(get_logger(), "Arbiter source -> %s (override=%s)", name,
                override_active_ ? "true" : "false");
  }

  std::string offboard_cmd_topic_;
  std::string interceptor_cmd_topic_;
  std::string override_topic_;
  std::string output_topic_;
  std::string cmd_frame_id_;

  double arbiter_rate_hz_{30.0};
  double offboard_cmd_timeout_sec_{0.5};
  double interceptor_cmd_timeout_sec_{0.25};
  bool publish_zero_on_stale_{true};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr offboard_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr interceptor_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr override_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::TwistStamped offboard_cmd_;
  geometry_msgs::msg::TwistStamped interceptor_cmd_;
  bool offboard_cmd_seen_{false};
  bool interceptor_cmd_seen_{false};
  rclcpp::Time last_offboard_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_interceptor_cmd_time_{0, 0, RCL_ROS_TIME};
  bool override_active_{false};

  Source last_source_{Source::kNone};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelArbiterNode>());
  rclcpp::shutdown();
  return 0;
}
