#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "mcl_positioning/mcl.hpp"
#include "mcl_positioning/landmark_manager.hpp"

#include <memory>
#include <cmath>
#include <vector>

class MclPositioningNode : public rclcpp::Node {
public:
    MclPositioningNode() : Node("mcl_positioning_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing MCL Positioning Node");
    
        this->declare_parameter<std::string>("landmarks_csv_path", "");
        this->declare_parameter<int>("num_particles", 500);

        this->declare_parameter<double>("motion_noise_xy", 0.01);
        this->declare_parameter<double>("motion_noise_theta", 0.01);
        this->declare_parameter<double>("measurement_noise", 0.01);

        motion_noise_xy_ = this->get_parameter("motion_noise_xy").as_double();
        motion_noise_theta_ = this->get_parameter("motion_noise_theta").as_double();
        measurement_noise_ = this->get_parameter("measurement_noise").as_double();

        std::string csv_path = this->get_parameter("landmarks_csv_path").as_string();
        int num_particles = this->get_parameter("num_particles").as_int();
       
        double x_min = -10.0;
        double x_max = 10.0;
        double y_min = -10.0;
        double y_max = 10.0;

        // Initialize Landmark Manager and Load CSV
        if (!landmarksManager_.loadFromCSV(csv_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load CSV: %s", csv_path.c_str());
        }

        mcl_ = std::make_unique<MCL>(num_particles, x_min, x_max, y_min, y_max, landmarksManager_);

        // Create subscribers
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_noisy",
            rclcpp::QoS(10),
            std::bind(&MclPositioningNode::odometryCallback, this, std::placeholders::_1)
        );
        
        landmarks_obs_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/landmarks_observed",
            rclcpp::QoS(10),
            std::bind(&MclPositioningNode::landmarksObservedCallback, this, std::placeholders::_1)
        );
        
        estimated_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/robot_estimated_odometry_mcl", rclcpp::QoS(10)
        );
        particle_cloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
                    "/mcl_particle_cloud", rclcpp::QoS(10));

        last_time_ = rclcpp::Time();

        RCLCPP_INFO(this->get_logger(), "MCL Positioning Node initialized with %d particles", num_particles);

        
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_obs_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_cloud_pub_;

    std::unique_ptr<MCL> mcl_;
    LandmarkManager landmarksManager_;

    rclcpp::Time last_time_;
    double last_x_ = 0.0, last_y_ = 0.0, last_theta_ = 0.0;
    double motion_noise_xy_;
    double motion_noise_theta_;
    double measurement_noise_;

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        //odometry msg in global frame
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = quaternionToYaw(msg->pose.pose.orientation);

        if (last_time_.nanoseconds() == 0) {
            last_time_ = msg->header.stamp;
            last_x_ = x; last_y_ = y; last_theta_ = theta;
            return;
        }

        // calculate deltas for motion model
        double dx = x - last_x_;
        double dy = y - last_y_;
        double dtheta = normalizeAngle(theta - last_theta_);

        // update Particles
        mcl_->motionUpdate(dx, dy, dtheta, motion_noise_xy_, motion_noise_theta_);

        // saving for next loop
        last_x_ = x; 
        last_y_ = y; 
        last_theta_ = theta;
        last_time_ = msg->header.stamp;

    }
    
    void landmarksObservedCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        
        std::vector<std::tuple<int, double, double>> observations;//id, x, y, noise

        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_id(*msg, "id");
            
            int count = 0;
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_id) {
                int landmark_id = static_cast<int>(*iter_id);
                float obs_x = static_cast<float>(*iter_x);
                float obs_y = static_cast<float>(*iter_y);
                observations.push_back(std::make_tuple(landmark_id, obs_x, obs_y));//constant not used

                RCLCPP_DEBUG(this->get_logger(),
                    "Landmark %d observed at (%.3f, %.3f)",
                    landmark_id, obs_x, obs_y);
                
                count++;
            }
            
            RCLCPP_DEBUG(this->get_logger(), 
                "Processed %d landmark observations", count);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), 
                "Failed to parse landmark observations: %s", e.what());
        }

        if (!observations.empty()) {
            mcl_->measurementUpdate(observations, measurement_noise_);

            mcl_->resample(); 
        }

        else {
        RCLCPP_WARN(this->get_logger(), "No observations to process!");
        }

        publishParticleCloud(msg->header.stamp);
        publishEstimatedOdometry(msg->header.stamp);
    }

    void publishEstimatedOdometry(const rclcpp::Time& timestamp) {
        auto particles = mcl_->getParticles();
        if (particles.empty()) return;

        double sum_x = 0.0, sum_y = 0.0, sum_sin = 0.0, sum_cos = 0.0;
        double total_weight = 0.0;

        //A5: compute weighted mean
        for (const auto& p : particles) {
            sum_x += p.state[0] * p.weight;
            sum_y += p.state[1] * p.weight;
            sum_sin += std::sin(p.state[2]) * p.weight;
            sum_cos += std::cos(p.state[2]) * p.weight;
            total_weight += p.weight;
        }
        if (total_weight < 1e-12) {
            RCLCPP_WARN(this->get_logger(), "Total particle weight too small, skipping publish");
            return;
        }

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "robot_estimated_mcl";

        odom_msg.pose.pose.position.x = sum_x / total_weight;
        odom_msg.pose.pose.position.y = sum_y / total_weight;
        double avg_theta = std::atan2(sum_sin, sum_cos);//need tan2 for weighted angle
        odom_msg.pose.pose.orientation = yawToQuaternion(avg_theta);

        estimated_odom_pub_->publish(odom_msg);
    }

    void publishParticleCloud(const rclcpp::Time& timestamp) {
    geometry_msgs::msg::PoseArray cloud_msg;
    cloud_msg.header.stamp = timestamp;
    cloud_msg.header.frame_id = "map"; // Particles exist in map coordinates

    for (const auto& p : mcl_->getParticles()) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.state[0];
        pose.position.y = p.state[1];
        pose.position.z = 0.0; // 2D robot

        // Convert particle theta to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, p.state[2]);
        pose.orientation = tf2::toMsg(q);

        cloud_msg.poses.push_back(pose);
    }

    particle_cloud_pub_->publish(cloud_msg);
    }

    // Helpers
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double r, p, y;
        tf2::Matrix3x3(tf_q).getRPY(r, p, y);
        return y;
    }

    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MclPositioningNode>());
    rclcpp::shutdown();
    return 0;
}