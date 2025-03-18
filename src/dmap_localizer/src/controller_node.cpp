#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
class RobotController {
private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber initial_pose_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    double systematic_error_linear = 0.05; 
    double systematic_error_angular = 0.05; 
    double noise_stddev_linear = 0.05; 
    double noise_stddev_angular = 0.05; 
    
    geometry_msgs::Twist current_velocity_;
    double x_, y_, theta_;
    double true_x_, true_y_, true_theta_;
    bool has_initial_pose_=false;
    ros::Time current_time_, last_time_;
public:
    RobotController() : nh_("~") {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
        
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &RobotController::cmdVelCallback, this);
        initial_pose_sub_ = nh_.subscribe("/initialpose", 10, &RobotController::initialPoseCallback, this);
        
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        current_time_ = ros::Time::now();
        last_time_ = current_time_;
    }
    

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        current_velocity_ = *msg;
    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        
        true_x_ = msg->pose.pose.position.x;
        true_y_ = msg->pose.pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        true_theta_ = yaw;
        
        x_ = 0;
        y_ = 0;
        theta_ = 0;
        
        has_initial_pose_ = true;
        ROS_INFO("Initial pose received.");
    }

    void updateOdometry() {
        if (!has_initial_pose_) return;
        current_time_ = ros::Time::now();
        double dt = (current_time_ - last_time_).toSec();
        last_time_ = current_time_;

        double delta_x = current_velocity_.linear.x * cos(true_theta_) * dt;
        double delta_y = current_velocity_.linear.x * sin(true_theta_) * dt;
        double delta_theta = current_velocity_.angular.z * dt;
        
        true_x_ += delta_x;
        true_y_ += delta_y;
        true_theta_ += delta_theta;


        geometry_msgs::TransformStamped gt_map_trans;
        gt_map_trans.header.stamp = current_time_;
        gt_map_trans.header.frame_id = "map";
        gt_map_trans.child_frame_id = "gt_base_link";
        gt_map_trans.transform.translation.x = true_x_;
        gt_map_trans.transform.translation.y = true_y_;
        tf2::Quaternion true_q;
        true_q.setRPY(0, 0, true_theta_);
        gt_map_trans.transform.rotation = tf2::toMsg(true_q);
        tf_broadcaster_.sendTransform(gt_map_trans);


        double drift_delta_x = delta_x*(1.0 + systematic_error_linear);
        double drift_delta_y =delta_y* (1.0 + systematic_error_linear);
        double drift_delta_theta =delta_theta *(1.0 - systematic_error_angular);

        std::random_device rd;
        std::mt19937 gen(rd());
        drift_delta_x += std::normal_distribution<double>(0.0, noise_stddev_linear)(gen);
        drift_delta_y += std::normal_distribution<double>(0.0, noise_stddev_linear)(gen);
        drift_delta_theta += std::normal_distribution<double>(0.0, noise_stddev_angular)(gen);

        x_ += drift_delta_x;
        y_ += drift_delta_y;
        theta_ += drift_delta_theta;

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time_;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist = current_velocity_;
        odom_pub_.publish(odom);


        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time_;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();

        tf_broadcaster_.sendTransform(odom_trans);


    }


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");
    RobotController robot;

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        robot.updateOdometry();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}