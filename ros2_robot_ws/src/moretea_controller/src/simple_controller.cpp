#include "moretea_controller/simple_controller.hpp"
#include <Eigen/Geometry>

using std::placeholders::_1;

SimpleController::SimpleController(const std::string& name)
    : Node(name)
{
    // 1. Declare Parameters
    declare_parameter("wheel_radius", 0.029);
    declare_parameter("wheel_separation_width", 0.14);  // Distance Left-Right
    declare_parameter("wheel_separation_length", 0.121); // Distance Front-Back

    // 2. Get Parameters
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_width_ = get_parameter("wheel_separation_width").as_double();
    wheel_separation_length_ = get_parameter("wheel_separation_length").as_double();

    // 3. Calculate Geometry Factor (lx + ly)
    // Assuming separation is total distance, lx = length/2, ly = width/2
    geom_factor_ = (wheel_separation_width_ + wheel_separation_length_) / 2.0;

    RCLCPP_INFO_STREAM(get_logger(), "Mecanum Controller Started");
    RCLCPP_INFO_STREAM(get_logger(), "Radius: " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Geom Factor (lx+ly): " << geom_factor_);
    
    // 4. Init Pub/Sub
    // Note: Ensure topic matches your launch file remappings
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/moretea_controller/cmd_vel", 
        10, 
        std::bind(&SimpleController::velCallback, this, _1)
    );
}

void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    // 1. Extract Robot Velocities
    double vx = msg.twist.linear.x;
    double vy = msg.twist.linear.y; // Mecanum can move sideways
    double wz = msg.twist.angular.z;

    // 2. Calculate Wheel Velocities (Inverse Kinematics)
    // Formula: v_wheel = (vx +/- vy +/- (lx+ly)*wz) / radius
    
    // Front Left (FL)
    double fl_speed = (vx - vy - (geom_factor_ * wz)) / wheel_radius_;
    
    // Front Right (FR)
    double fr_speed = (vx + vy + (geom_factor_ * wz)) / wheel_radius_;
    
    // Rear Left (RL)
    double rl_speed = (vx + vy - (geom_factor_ * wz)) / wheel_radius_;
    
    // Rear Right (RR)
    double rr_speed = (vx - vy + (geom_factor_ * wz)) / wheel_radius_;

    // 3. Publish Command
    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    
    // IMPORTANT: Match this order with your controller.yaml 'joints' list!
    // Common order: [FL, FR, RL, RR] or [FL, RL, FR, RR]
    wheel_speed_msg.data.push_back(fl_speed);
    wheel_speed_msg.data.push_back(rl_speed);
    wheel_speed_msg.data.push_back(fr_speed);
    wheel_speed_msg.data.push_back(rr_speed);
    
    wheel_cmd_pub_->publish(wheel_speed_msg);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleController>("simple_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}