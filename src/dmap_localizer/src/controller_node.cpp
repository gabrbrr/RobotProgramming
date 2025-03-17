#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
class RobotController {
private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber initial_pose_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    geometry_msgs::Twist current_velocity_;
    double true_x_, true_y_, true_theta_;
    bool has_initial_pose_=false;
    ros::Time current_time_, last_time_;
public:
    RobotController() : nh_("~") {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
        
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &RobotController::cmdVelCallback, this);
        initial_pose_sub_ = nh_.subscribe("/initialpose", 10, &RobotController::initialPoseCallback, this);
        
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