#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

class ScanGenerator {
public:
    ScanGenerator() : tf_listener_(tf_buffer_) {
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 10);
        map_sub_ = nh_.subscribe("/map", 1, &ScanGenerator::mapCallback, this);
        
        // Initialize scan parameters
        scan_.header.frame_id = "base_link";
        scan_.angle_min = -M_PI/2;
        scan_.angle_max = M_PI/2;
        scan_.angle_increment = M_PI/180; 
        scan_.range_min = 0.1;
        scan_.range_max = 30.0;
        scan_.ranges.resize(181, scan_.range_max); 
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        current_map_ = *msg;
        has_map_ = true;
    }

    void generateScan() {
        if (!has_map_) return;

        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform("map", "gt_base_link", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        double yaw = tf2::getYaw(transform.transform.rotation);

       
        for (size_t i = 0; i < scan_.ranges.size(); ++i) {
            double angle = scan_.angle_min + i * scan_.angle_increment;
            scan_.ranges[i] = raycast(x, y, yaw + angle);
        }

        scan_.header.stamp = ros::Time::now();
        scan_pub_.publish(scan_);
    }

    float raycast(double x_robot, double y_robot, double angle) {
        const float max_range = scan_.range_max;

        for (float r = 0; r < max_range; r += current_map_.info.resolution) {
            float wx = x_robot + r * cos(angle);
            float wy = y_robot + r * sin(angle);

            int mx =  wx / current_map_.info.resolution;
            int my = wy / current_map_.info.resolution;

            if (mx < 0 || mx >= current_map_.info.width || my < 0 || my >= current_map_.info.height)
                continue;

            int index = my * current_map_.info.width + mx;
            if (current_map_.data[index] > 50) { 
                return r;
            }
        }
        return max_range;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher scan_pub_;
    ros::Subscriber map_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    nav_msgs::OccupancyGrid current_map_;
    sensor_msgs::LaserScan scan_;
    bool has_map_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_generator");
    ScanGenerator generator;

    ros::Rate loop_rate(2);
    while (ros::ok()) {
        generator.generateScan();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}