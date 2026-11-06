#include <rclcpp/rclcpp.hpp>
#include <numbers>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std::chrono_literals;
using nav_msgs::msg::Odometry;

class Odom2Chassis : public rclcpp::Node {
public:
    Odom2Chassis (void):
    Node("odom_to_chassis_node") {
        odom_sub_ptr = this->create_subscription<Odometry>("/Odometry", 1, std::bind(&Odom2Chassis::odom_cbk, this, std::placeholders::_1));
        odom_pub_ptr = this->create_publisher<Odometry>("/Odometry_chassis", 1);
        this->declare_parameter<double>("yaw", 0.0);
        this->declare_parameter<double>("pitch", 0.0);
        this->declare_parameter<double>("roll", 0.0);
        this->declare_parameter<double>("x", 0.2);
        this->declare_parameter<double>("y", 0.0);
        this->declare_parameter<double>("z", 0.05);
        this->declare_parameter<bool>("debug", false);
        timer_ = this->create_wall_timer(500ms, std::bind(&Odom2Chassis::timer_cbk, this));
    }

private:
    void odom_cbk(const Odometry & msg) {
        Eigen::Quaterniond q_lidar_world(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
        Eigen::Vector3d translate (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
        Eigen::Matrix4d transform_l_w = Eigen::Matrix4d::Identity();
        transform_l_w.block(0, 0, 3, 3) = q_lidar_world.toRotationMatrix();
        transform_l_w.block(0, 3, 3, 1) = translate;
        update_extrinsic();
        Eigen::Matrix4d transform_chassisego_chassisworld;
        transform_chassisego_chassisworld = transform_l_chassis *transform_l_w * transform_l_chassis.inverse();
        Odometry msg_pub;
        msg_pub.header = msg.header;
        Eigen::Matrix3d m_tmp = transform_chassisego_chassisworld.block(0, 0, 3, 3);
        Eigen::Quaterniond q_out (m_tmp);
        geometry_msgs::msg::Quaternion q_pose;
        q_pose.x = q_out.x();
        q_pose.y = q_out.y();
        q_pose.z = q_out.z();
        q_pose.w = q_out.w();
        // RCLCPP_INFO(this->get_logger(), "%f\n", q_lidar_world.norm());
        msg_pub.pose.pose.orientation = q_pose;
        msg_pub.pose.pose.position.x = transform_chassisego_chassisworld(0, 3);
        msg_pub.pose.pose.position.y = transform_chassisego_chassisworld(1, 3);
        msg_pub.pose.pose.position.z = transform_chassisego_chassisworld(2, 3);

        odom_pub_ptr->publish(msg_pub);
        latest_ori = q_out;
        // latest_pos = transform_chassisego_chassisworld.block(0, 3, 3, 1);
        latest_pos = Eigen::Vector3d(
             transform_chassisego_chassisworld(0, 3),
             transform_chassisego_chassisworld(1, 3),
             transform_chassisego_chassisworld(2, 3) 
        );
    }

    // 2, 1, 0, yaw, pitch, roll in rad
    Eigen::Vector3d q2euler(Eigen::Quaterniond &q) {
        auto mat = q.toRotationMatrix();
        double a, b, r;
        b = std::asin(mat(0, 2));
        if (std::cos(b) > 0.001) {
            // a = std::atan2(- mat(1, 2) / std::cos(b), mat(2, 2) / std::cos(b));
            // r = std::atan2(- mat(0, 1) / std::cos(b), mat(0, 0) / std::cos(b));
            a = std::atan2(- mat(1, 2), mat(2, 2));
            r = std::atan2(- mat(0, 1), mat(0, 0));  // Damn the cos is always positive here.
        }
        else {
            a = 0;
            r = std::atan2(mat(1, 0), mat(1, 2));
        }
        return Eigen::Vector3d (a, b, r);
    }

    void timer_cbk(void) {
        Eigen::Vector3d euler = q2euler(latest_ori);
        if (this->get_parameter("debug").as_bool() == true) {
            RCLCPP_INFO(this->get_logger(), "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", euler(0) * 180 / 3.14159, euler(1) * 180 / 3.14159, euler(2) * 180 / 3.14159, latest_pos(0), latest_pos(1), latest_pos(2));
        }
    }

    void update_extrinsic(void) {
        double yaw = this->get_parameter("yaw").as_double();
        double pitch = this->get_parameter("pitch").as_double();
        double roll = this->get_parameter("roll").as_double();
        double x = this->get_parameter("x").as_double();
        double y = this->get_parameter("y").as_double();
        double z = this->get_parameter("z").as_double();

        transform_l_chassis.block(0, 0, 3, 3) = ypr2mat(yaw, pitch, roll);
        transform_l_chassis(0, 3) = x;
        transform_l_chassis(1, 3) = y;
        transform_l_chassis(2, 3) = z;
        transform_l_chassis(3, 3) = 1.0;       
    }

    // angle format, z, y, x
    Eigen::Matrix3d ypr2mat(double yaw, double pitch, double roll) {
        yaw = yaw / 180 * 3.14159;
        pitch = pitch / 180 * 3.14159;
        roll = roll / 180 * 3.14159;
        Eigen::Matrix3d ret;
        using std::cos, std::sin;
        ret(0, 0) = cos(pitch) * cos(yaw);
        ret(0, 1) = - cos(pitch) * sin(yaw);
        ret(0, 2) = sin(pitch);
        ret(1, 0) = cos(roll) * sin(yaw) + sin(roll) * sin(pitch) * cos(yaw);
        ret(1, 1) = cos(roll) * cos(yaw) - sin(roll) * sin(pitch) * sin(yaw);
        ret(1, 2) = - sin(roll) * cos(pitch);
        ret(2, 0) = sin(roll) * sin(yaw) - cos(roll) * sin(pitch) * cos(yaw);
        ret(2, 1) = sin(roll) * cos(yaw) + cos(roll) * sin(pitch) * sin(yaw);
        ret(2, 2) = cos(roll) * cos(pitch);
        return ret;
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_ptr;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ptr;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::Matrix4d transform_l_chassis = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond latest_ori;
    Eigen::Vector3d latest_pos;
};
int main(int argc, char** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odom2Chassis>());
    return 0;
}
