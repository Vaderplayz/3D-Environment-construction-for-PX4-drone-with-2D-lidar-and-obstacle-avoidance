#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

class TiltBroadcaster : public rclcpp::Node {
public:
    TiltBroadcaster() : Node("tilt_broadcaster") {
        // 1. Setup Serial Communication
        serial_port_ = open("/dev/ttyACM0", O_RDWR);
        struct termios tty;
        if(tcgetattr(serial_port_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
        }
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tcsetattr(serial_port_, TCSANOW, &tty);

        // 2. Initialize TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // 3. Loop Timer (50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&TiltBroadcaster::update_tf, this));
    }

private:
    void update_tf() {
        char buf[32];
        int n = read(serial_port_, buf, sizeof(buf));
        if (n > 0) {
            try {
                float angle_deg = std::stof(std::string(buf, n));
                float angle_rad = angle_deg * (M_PI / 180.0); // Convert to radians

                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "base_link"; // The stationary base
                t.child_frame_id = "laser";     // The tilting LiDAR frame

                t.transform.translation.x = 0.0;
                t.transform.translation.y = 0.0;
                t.transform.translation.z = 0.1; // Offset height of 10cm

                tf2::Quaternion q;
                q.setRPY(0, angle_rad, 0); // Apply the Pitch (tilt)
                t.transform.rotation.x = q.x();
                t.transform.rotation.y = q.y();
                t.transform.rotation.z = q.z();
                t.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(t);
            } catch (...) {}
        }
    }

    int serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TiltBroadcaster>());
    rclcpp::shutdown();
    return 0;
}