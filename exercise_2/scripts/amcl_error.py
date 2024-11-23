#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates

# List to store the particles
particles = []

# Callback function to update particles
def particlecloud_callback(msg):
    global particles
    particles = msg.poses  # The PoseArray message contains a list of poses (particles)

# Callback function to get the true pose from gazebo/model_states
def true_pose_callback(msg):
    global true_pose
    true_pose = msg.pose[0]  # Assuming the robot is the first model in model_states

# Function to calculate Euclidean distance between two poses
def euclidean_distance(pose1, pose2):
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    dz = pose1.position.z - pose2.position.z
    return np.sqrt(dx**2 + dy**2 + dz**2)

# Main function
def main():
    # Initialize the node
    rospy.init_node('amcl_error_calculator', anonymous=True)
    
    # Subscribers
    rospy.Subscriber("/particlecloud", PoseArray, particlecloud_callback)  # Subscribe to particlecloud
    rospy.Subscriber("/gazebo/model_states", ModelStates, true_pose_callback)  # Subscribe to model_states for true pose
    
    # Publisher for error
    error_pub = rospy.Publisher('/amcl_error', Float64, queue_size=10)

    # Loop rate (e.g., 10Hz)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if len(particles) > 0 and true_pose:
            total_error = 0
            num_particles = len(particles)
            for particle in particles:
                # Calculate the Euclidean distance between the true pose and each particle
                particle_pose = particle  # Each particle is a Pose
                error = euclidean_distance(particle_pose, true_pose)
                total_error += error

            # Calculate the mean error
            mean_error = total_error / num_particles

            # Publish the mean error
            error_pub.publish(mean_error)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
