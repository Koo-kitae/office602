#include "ros/ros.h"
// #include "std_msgs/Int32.h"  // CHANGED: No longer using Int32 for ticks
#include "std_msgs/Float32.h" // ADDED: Using Float32 for speed
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

// Create odometry data publishers
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

// Robot physical constants
// const double TICKS_PER_REVOLUTION = 4096*4; // REMOVED: No longer needed
const double WHEEL_RADIUS = 0.107/2; // Wheel radius in meters - This might still be useful if speed is in rad/s, but assuming m/s for now.
const double WHEEL_BASE = 0.360; // Center of left tire to center of right tire
// const double TICKS_PER_METER = TICKS_PER_REVOLUTION/(2*PI*WHEEL_RADIUS); // REMOVED: No longer needed

// ADDED: Variables to store current wheel speeds and time
double left_wheel_speed = 0.0;  // m/s
double right_wheel_speed = 0.0; // m/s
ros::Time last_time;

// REMOVED: Distance variables are no longer needed
// double distanceLeft = 0;
// double distanceRight = 0;

// Flag to see if initial pose has been received
bool initialPoseRecieved = false;

using namespace std;

// Get initial_2d message from either Rviz clicks or a manual pose publisher
// This function remains the same.
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {
    odomOld.pose.pose.position.x = rvizClick.pose.position.x;
    odomOld.pose.pose.position.y = rvizClick.pose.position.y;
    odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
    initialPoseRecieved = true;
}

// ADDED: Callback for left motor speed
void leftSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
    left_wheel_speed = msg->data;
}

// ADDED: Callback for right motor speed
void rightSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
    right_wheel_speed = msg->data;
}

// REMOVED: The old tick-based calculation functions are no longer needed.
// void Calc_Left(const std_msgs::Int32& leftCount) { ... }
// void Calc_Right(const std_msgs::Int32& rightCount) { ... }


// Publish a nav_msgs::Odometry message in quaternion format
// This function remains mostly the same.
void publish_quat() {
    tf2::Quaternion q;
    // The yaw angle is in odomNew.pose.pose.orientation.z
    q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

    nav_msgs::Odometry quatOdom;
    quatOdom.header.stamp = odomNew.header.stamp;
    quatOdom.header.frame_id = "odom";
    quatOdom.child_frame_id = "base_link"; // As per your launch file, robot_pose_ekf will handle odom->base_footprint. This should be base_footprint.
    quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
    quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
    quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
    quatOdom.pose.pose.orientation.x = q.x();
    quatOdom.pose.pose.orientation.y = q.y();
    quatOdom.pose.pose.orientation.z = q.z();
    quatOdom.pose.pose.orientation.w = q.w();
    quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
    quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
    quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
    quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
    quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
    quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

    for(int i = 0; i<36; i++) {
        if(i == 0 || i == 7 || i == 14) {
            quatOdom.pose.covariance[i] = .01;
        }
        else if (i == 21 || i == 28 || i== 35) {
            quatOdom.pose.covariance[i] += 0.1;
        }
        else {
            quatOdom.pose.covariance[i] = 0;
        }
    }
    odom_data_pub_quat.publish(quatOdom);
}

// Update odometry information
void update_odom() {
    // ---- CHANGED: This function is completely rewritten for speed-based calculation ----

    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec(); // FIXED: Changed to_sec() to toSec()
    last_time = current_time;

    // Robot's linear and angular velocities
    double v_x = (right_wheel_speed + left_wheel_speed) / 2.0;
    double v_th = (right_wheel_speed - left_wheel_speed) / WHEEL_BASE;

    // Compute change in pose
    double delta_x = (v_x * cos(odomOld.pose.pose.orientation.z)) * dt;
    double delta_y = (v_x * sin(odomOld.pose.pose.orientation.z)) * dt;
    double delta_th = v_th * dt;

    // Update new pose
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + delta_x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + delta_y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z + delta_th;

    // Prevent lockup from a single bad cycle
    if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
        || isnan(odomNew.pose.pose.position.z)) {
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
        odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
    }

    // Make sure theta stays in the correct range (-PI to PI)
    if (odomNew.pose.pose.orientation.z > PI) {
        odomNew.pose.pose.orientation.z -= 2 * PI;
    }
    else if (odomNew.pose.pose.orientation.z < -PI) {
        odomNew.pose.pose.orientation.z += 2 * PI;
    }

    // Fill in the velocity information
    odomNew.header.stamp = current_time;
    odomNew.twist.twist.linear.x = v_x;
    odomNew.twist.twist.angular.z = v_th;

    // Save the pose data for the next cycle
    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
    odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
    odomOld.header.stamp = odomNew.header.stamp; // This was missing in the original code, but good practice

    // Publish the odometry message (for debugging or other nodes)
    odom_data_pub.publish(odomNew);
}

int main(int argc, char **argv) {

    // Set the data fields of the odometry message
    odomNew.header.frame_id = "odom";
    odomNew.child_frame_id = "base_footprint"; // CHANGED: child_frame_id should be consistent
    odomNew.pose.pose.position.z = 0;
    odomNew.pose.pose.orientation.x = 0;
    odomNew.pose.pose.orientation.y = 0;
    odomNew.twist.twist.linear.x = 0;
    odomNew.twist.twist.linear.y = 0;
    odomNew.twist.twist.linear.z = 0;
    odomNew.twist.twist.angular.x = 0;
    odomNew.twist.twist.angular.y = 0;
    odomNew.twist.twist.angular.z = 0;
    odomOld.pose.pose.position.x = initialX;
    odomOld.pose.pose.position.y = initialY;
    odomOld.pose.pose.orientation.z = initialTheta;

    // Launch ROS and create a node
    ros::init(argc, argv, "ekf_odom_pub");
    ros::NodeHandle node;

    // ---- CHANGED: Subscribe to speed topics instead of tick topics ----
    ros::Subscriber subForRightSpeed = node.subscribe("right_motor_speed", 100, rightSpeedCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForLeftSpeed = node.subscribe("left_motor_speed", 100, leftSpeedCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);

    // Publisher of simple odom message where orientation.z is an euler angle
    odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_data_euler", 100);

    // Publisher of full odom message where orientation is quaternion
    odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_data_quat", 100); // Increased queue size slightly

    ros::Rate loop_rate(30); // Loop rate in Hz

    // ADDED: Initialize last_time before the loop starts
    last_time = ros::Time::now();

    while(ros::ok()) {
        if(initialPoseRecieved) {
            update_odom();
            publish_quat();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
