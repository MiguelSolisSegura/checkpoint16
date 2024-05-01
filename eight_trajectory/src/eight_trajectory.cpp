#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include <string>
#include <cmath>

using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Odometry = nav_msgs::msg::Odometry;
using namespace std::chrono_literals;
using namespace std::placeholders;

class EightTrajectory : public rclcpp::Node
{
public:
    EightTrajectory() : Node("eight_trajectory") {
        RCLCPP_INFO(this->get_logger(), "Initialized eight_trajectory node.");
        CommandVels << 0.0, 0.0, 0.0;
        publisher_ = this->create_publisher<Float32MultiArray>("/wheel_speed", 10);
        subscription_ = this->create_subscription<Odometry>("/rosbot_xl_base_controller/odom", 10, std::bind(&EightTrajectory::topic_callback, this, _1));
        waypoint_timer_ = this->create_wall_timer(3s, std::bind(&EightTrajectory::select_waypoint, this));
        //velocities_timer_ = this->create_wall_timer(100ms, std::bind(&EightTrajectory::compute_velocities, this));
    }

private:
    // General attributes
    rclcpp::Publisher<Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr waypoint_timer_;
    rclcpp::TimerBase::SharedPtr velocities_timer_;

    // Robot physical properties
    float w = 0.134845;
    float r = 0.05;
    float l = 0.085;

    // Orietation and waypoints
    float phi;
    int current_waypoint = 0;
    //Eigen::MatrixXd CommandVels;
    Eigen::Vector3d CommandVels;

    void select_waypoint() {
        RCLCPP_INFO(this->get_logger(), "Changing waypoint.");
        current_waypoint += 1;
        switch (current_waypoint) {
            case 1:
                CommandVels << 0.0, 1.0, -1.0;
                break;
            case 2:
                CommandVels << 0.0, 1.0, 1.0;
                break;
            case 3:
                CommandVels << 0.0, 1.0, 1.0;
                break;
            case 4:
                CommandVels << 1.5708, 1.0, -1.0;
                break;
            case 5:
                CommandVels << -3.1415, -1.0, -1.0;
                break;
            case 6:
                CommandVels << 0.0, -1.0, 1.0;
                break;
            case 7:
                CommandVels << 0.0, -1.0, 1.0;
                break;
            case 8:
                CommandVels << 0.0, -1.0, -1.0;
                break;
            default:
                rclcpp::shutdown();
        }
    }

    void compute_velocities() {
        Eigen::MatrixXd RotationMatrix(3, 3);
        Eigen::MatrixXd TwistVels(3, 1);
        RotationMatrix << 1,    0,             0,
                          0,    std::cos(phi), std::sin(phi),
                          0,   -std::sin(phi), std::cos(phi);
        TwistVels = RotationMatrix * CommandVels;
        send_message(TwistVels, "Update velocities");
    }

    void send_message(Eigen::MatrixXd TwistVels, std::string Message) {
        Eigen::MatrixXd HolonomicMatrix(4, 3);
        HolonomicMatrix << -l-w, 1, -1,
                            l+w, 1,  1,
                            l+w, 1, -1,
                           -l-w, 1,  1;

        Eigen::MatrixXd WheelsVel(4, 1);
        WheelsVel = (1/r) * HolonomicMatrix * TwistVels;

        Float32MultiArray vel_message;
        vel_message.data = {float(WheelsVel(0, 0)),
                            float(WheelsVel(1, 0)), 
                            float(WheelsVel(2, 0)), 
                            float(WheelsVel(3, 0))};

        RCLCPP_DEBUG(this->get_logger(), "%s.", Message.c_str());
        publisher_->publish(vel_message);
    }

    void topic_callback(const Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Odom callback.");
        float z = msg->pose.pose.orientation.z;
        float w = msg->pose.pose.orientation.w;
        phi = 2 * std::atan2(z, w);
        compute_velocities();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EightTrajectory>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}