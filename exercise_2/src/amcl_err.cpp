#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>
#include <cmath>
#include <vector>

// List to store the particles
std::vector<geometry_msgs::Pose> particles;

// Variable to store the true pose
geometry_msgs::Pose true_pose;

// Callback function to update particles
void particlecloudCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    particles = msg->poses; // The PoseArray message contains a list of poses (particles)
}

// Callback function to get the true pose from gazebo/model_states
void truePoseCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    if (!msg->pose.empty()) {
        true_pose = msg->pose[0]; // Assuming the robot is the first model in model_states
    }
}

// Function to calculate Euclidean distance between two poses
double euclideanDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int main(int argc, char** argv) {
    // Initialize the node
    ros::init(argc, argv, "amcl_error_calculator");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber particlecloud_sub = nh.subscribe("/particlecloud", 10, particlecloudCallback); // Subscribe to particlecloud
    ros::Subscriber true_pose_sub = nh.subscribe("/gazebo/model_states", 10, truePoseCallback);   // Subscribe to model_states for true pose

    // Publisher for error
    ros::Publisher error_pub = nh.advertise<std_msgs::Float64>("/amcl_error", 10);

    // Loop rate (e.g., 10Hz)
    ros::Rate rate(10);

    while (ros::ok()) {
        if (!particles.empty()) {
            double total_error = 0.0;
            size_t num_particles = particles.size();

            for (const auto& particle : particles) {
                // Calculate the Euclidean distance between the true pose and each particle
                double error = euclideanDistance(particle, true_pose);
                total_error += error;
            }

            // Calculate the mean error
            double mean_error = total_error / num_particles;

            // Publish the mean error
            std_msgs::Float64 error_msg;
            error_msg.data = mean_error;
            error_pub.publish(error_msg);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
