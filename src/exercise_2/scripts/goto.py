#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from math import atan2, sqrt, pow, pi
from geometry_msgs.msg import Point
import math
class StationNavigator:
    def __init__(self):
        rospy.init_node("station_navigator")

        # Publishers
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_result_callback)

        # Publisher
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

        # Parameters
        self.linear_velocity = rospy.get_param("~linear_velocity", 0.5)
        self.angular_velocity = rospy.get_param("~angular_velocity", 1.5)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.5)
        self.buffer_distance = rospy.get_param("~buffer_distance", 0.5)  # Additional buffer zone

        # Variables
        self.robot_pose = None
        self.station_poses = {}
        self.stations = ["station1", "station2", "station3", "station4", "station5", "station6", "station7", "station8"]
        self.stations_to_cover = ["station1", "station2", "station3", "station4"]

        self.obstacle_detected = False
        self.poses_received = False
        self.goals_covered = 0
        self.current_goal = 0
        self.goal_sent = False 
        self.stop_pose = False
        self.callback_count = 0
        

    def model_states_callback(self, msg):
        """
        Callback to store the positions of all models in the simulation.
        """
        if all(station in self.station_poses for station in self.stations_to_cover) and not self.stop_pose:
            rospy.loginfo("All station positions received. Starting navigation.")
            self.poses_received = True
            self.send_next()
            return 
        for name, pose in zip(msg.name, msg.pose):
            if name == "mir":
                self.robot_pose = pose.position
            if name in self.stations:
                self.station_poses[name] = pose.position

    def calculate_waypoint(self, station_pos):
        """
        Calculate a waypoint in the robot's frame of reference, with a buffer zone.
        """
        if not self.robot_pose:
            rospy.logwarn("Robot pose not yet available. Cannot calculate waypoint.")
            return None

        # Transform station position from world frame to robot's local frame
        dx = station_pos.x - self.robot_pose.x
        dy = station_pos.y - self.robot_pose.y

        # Calculate the distance and direction
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)

        # Apply buffer distance
        buffer_distance = self.buffer_distance
        adjusted_distance = max(0, distance - buffer_distance)

        # Compute the waypoint in the robot's local frame
        waypoint = Point()
        waypoint.x = adjusted_distance * math.cos(angle)
        waypoint.y = adjusted_distance * math.sin(angle)
        waypoint.z = station_pos.z  # Z remains the same (2D navigation)
        return waypoint

        

    def send_next(self):
        if self.poses_received:
            self.stop_pose = True
            if self.current_goal < len(self.stations_to_cover):
                station_name = self.stations_to_cover[self.current_goal]
                station_pos = self.station_poses[station_name]
                waypoint = self.calculate_waypoint(station_pos)

                # Publish goal
                goal = MoveBaseActionGoal()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "map"

                goal.goal.target_pose.header.frame_id = "map"
                goal.goal.target_pose.pose.position = waypoint
                goal.goal.target_pose.pose.orientation.w = 1.0  # Face forward
                self.goal_pub.publish(goal)

                self.goal_sent = True
                rospy.loginfo(f"Goal sent to {station_name}: Waypoint {waypoint}")
            else:
                rospy.loginfo("All stations covered. Exiting.")
                rospy.signal_shutdown("Navigation complete.")
                

    def move_base_result_callback(self, msg):
        """
        Callback to handle move_base results and proceed to the next station.
        """
        if self.goal_sent and self.callback_count != 0:
            if msg.status.status == 3:  # Goal reached successfully
                # Proceed to the next station
                rospy.loginfo(f"Reached goal for station {self.stations_to_cover[self.current_goal]}.")
                self.current_goal += 1
                self.goal_sent = False
                self.send_next()
                
            elif msg.status.status in [4, 5]:  # Aborted or rejected
                rospy.logwarn(f"Failed to reach goal for station {self.stations_to_cover[self.current_goal]}.")
                self.send_next()
        self.callback_count = 1
        rospy.logwarn(f"Callback {self.callback_count}.")


            





if __name__ == "__main__":
    try:
        navigator = StationNavigator()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        rospy.logerr("Navigation interrupted.")