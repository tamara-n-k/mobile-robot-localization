#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "kalman_positioning/ukf.hpp"
#include "kalman_positioning/landmark_manager.hpp"


#include <memory>
#include <cmath>

/**
 * @brief Positioning node for UKF-based robot localization (Student Assignment)
 * 
 * This node subscribes to:
 *   - /robot_noisy: Noisy odometry (dead-reckoning)
 *   - /landmarks_observed: Noisy landmark observations
 * 
 * And publishes to:
 *   - /robot_estimated_odometry: Estimated pose and velocity from filter
 * 
 * STUDENT ASSIGNMENT:
 * Implement the Kalman filter logic to fuse odometry and landmark observations
 * to estimate the robot's true position.
 */
class PositioningNode : public rclcpp::Node {
public:
    PositioningNode() : Node("kalman_positioning_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Kalman Positioning Node");

    this->declare_parameter<std::string>("landmarks_csv_path", "");
    this->declare_parameter<double>("process_noise_xy", 0.01);
    this->declare_parameter<double>("process_noise_theta", 0.1);
    this->declare_parameter<double>("measurement_noise_xy", 0.01);

    std::string csv_path = this->get_parameter("landmarks_csv_path").as_string();
    double process_noise_xy = this->get_parameter("process_noise_xy").as_double();
    double process_noise_theta = this->get_parameter("process_noise_theta").as_double();
    double measurement_noise_xy = this->get_parameter("measurement_noise_xy").as_double();



    ukf_ = std::make_unique<UKF>(
        process_noise_xy,
        process_noise_theta,
        measurement_noise_xy
        );

        if (!landmarksManager_.loadFromCSV(csv_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load CSV: %s", csv_path.c_str());
        } 
        
        ukf_->setLandmarks(landmarksManager_.getLandmarks());

        // Create subscribers
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_noisy",
            rclcpp::QoS(10),
            std::bind(&PositioningNode::odometryCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot_noisy");
        
        landmarks_obs_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/landmarks_observed",
            rclcpp::QoS(10),
            std::bind(&PositioningNode::landmarksObservedCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /landmarks_observed");
        
        // Create publisher
        estimated_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/robot_estimated_odometry", rclcpp::QoS(10)
        );
        RCLCPP_INFO(this->get_logger(), "Publishing to /robot_estimated_odometry");
        
        RCLCPP_INFO(this->get_logger(), "Kalman Positioning Node initialized successfully");
    }

private:
    // ============================================================================
    // SUBSCRIBERS AND PUBLISHERS
    // ============================================================================
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_obs_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odom_pub_;
    std::unique_ptr<UKF> ukf_;
    rclcpp::Time last_time_;
    double last_x_;
    double last_y_;
    double last_theta_;
    LandmarkManager landmarksManager_;

    // ============================================================================
    // PLACEHOLDER: KALMAN FILTER STATE
    // ============================================================================
    // Students should implement a proper Kalman filter (e.g., UKF, EKF) 
    // with the following state:
    //   - Position: x, y (m)
    //   - Orientation: theta (rad)
    //   - Velocity: vx, vy (m/s)
    // And maintain:
    //   - State covariance matrix
    //   - Process noise covariance
    //   - Measurement noise covariance
    
    // ============================================================================
    // CALLBACK FUNCTIONS
    // ============================================================================
    
    /**
     * @brief Callback for noisy odometry measurements
     * 
     * STUDENT TODO:
     * 1. Extract position (x, y) and orientation (theta) from the message
     * 2. Update the Kalman filter's prediction step with this odometry
     * 3. Publish the estimated odometry
     */
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), 
            "Odometry received: x=%.3f, y=%.3f", 
            msg->pose.pose.position.x, msg->pose.pose.position.y);

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = quaternionToYaw(msg->pose.pose.orientation);

        if (last_time_.nanoseconds() == 0) {
            last_time_ = msg->header.stamp;
            last_x_ = x;
            last_y_ = y;
            last_theta_ = normalizeAngle(theta);
            return;
        }

        rclcpp::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Increments
        double dx = x - last_x_;
        double dy = y - last_y_;
        double dtheta = normalizeAngle(theta - last_theta_);

        last_x_ = x;
        last_y_ = y;
        last_theta_ = normalizeAngle(theta);


        ukf_->predict(dt, dx, dy, dtheta);
    }
    
    /**
     * @brief Callback for landmark observations
     * 
     * STUDENT TODO:
     * 1. Parse the PointCloud2 data to extract landmark observations
     * 2. Update the Kalman filter's measurement update step with these observations
     * 3. Optionally publish the updated estimated odometry
     */
    void landmarksObservedCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        RCLCPP_DEBUG(this->get_logger(), 
            "Landmark observation received with %u points", msg->width);
        
        std::vector<std::tuple<int, double, double>> observations;//id, x, y, noise
        //get landmark info from sensor msg and push back in tuple
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
            ukf_->update(observations);
        }
        publishEstimatedOdometry(msg->header.stamp);
    }
    
    // ============================================================================
    // HELPER FUNCTIONS
    // ============================================================================
    
    /**
     * @brief Convert quaternion to yaw angle
     * @param q Quaternion from orientation
     * @return Yaw angle in radians [-pi, pi]
     */


    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);
        return q_msg;
    }
    
    /**
     * @brief Normalize angle to [-pi, pi]
     * @param angle Input angle in radians
     * @return Normalized angle in [-pi, pi]
     */
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    /**
     * @brief Publish estimated odometry message
     * @param timestamp Message timestamp
     * @param odom_msg Odometry message to publish
     */
    void publishEstimatedOdometry(const rclcpp::Time& timestamp)
                                //   ,const nav_msgs::msg::Odometry& odom_msg) 
    {
        // nav_msgs::msg::Odometry estimated_odom = odom_msg;
        // nav_msgs::msg::Odometry odom_msg;
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "robot_estimated";
        
        // STUDENT TODO: Replace this with actual filter output
        // Set position, orientation, velocity, and covariance from your Kalman filter
        // Currently using placeholder values (noisy odometry)
        auto state = ukf_->getState();
        odom_msg.pose.pose.position.x = state(0);
        odom_msg.pose.pose.position.y = state(1);
        double theta = state(2);

        odom_msg.pose.pose.orientation = yawToQuaternion(normalizeAngle(theta));

        odom_msg.twist.twist.linear.x = state(3);
        odom_msg.twist.twist.linear.y = state(4);

        // Add covariance if you want:
        // fillCovarianceFromUkf(odom_msg);


        estimated_odom_pub_ ->publish(odom_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositioningNode>());
    rclcpp::shutdown();
    return 0;
}
