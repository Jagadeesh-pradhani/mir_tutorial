#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseActionResult
import numpy as np

class WaypointNavigator:

    def __init__(self):
        rospy.init_node('mir_waypoint_navigator')

        self.stations_count = rospy.get_param('~stations_count', 4)  

        # List of stations to visit
        self.stations = [f"station{i}" for i in range(1, 9)]
        self.stations_to_cover = [f"station{i}" for i in range(1, self.stations_count+1)]
        self.station_positions = {}
        self.current_goal_reached = False
        self.robot_pose = None

        # Subscribers and publishers
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # TF listener
        self.tf_listener = tf.TransformListener()

        rospy.loginfo("Waiting for station positions...")
        rospy.sleep(2)  # Allow time to populate station positions
        self.navigate_to_stations()
        

    def model_states_callback(self, data):
        try:
            # Extract positions of stations
            for name, pose in zip(data.name, data.pose):
                if name == "mir":
                    self.robot_pose = pose
                if name in self.stations:
                    self.station_positions[name] = pose
        except IndexError:
            rospy.logwarn("Error processing model states data!")

    def result_callback(self, result):
        # Check if the goal was successfully reached
        if result.status.status == 3:  # 3 indicates "Goal reached successfully"
            rospy.loginfo("Goal reached!")
            self.current_goal_reached = True
        # else:
        #     self.navigate_to_stations()

    def convert(self, m, n):
        pos = "position"
        ori = "orientation"
        n.position.x = m[pos][0]
        n.position.y = m[pos][1]
        n.position.z = m[pos][2]

        n.orientation.x = m[ori][0]
        n.orientation.y = m[ori][1]
        n.orientation.z = m[ori][2]
        n.orientation.w = m[ori][3]

        return n

    def navigate_to_stations(self):
        for station in self.stations_to_cover:
            if station not in self.station_positions:
                rospy.logwarn(f"Position for {station} not found!")
                continue

            rospy.loginfo(f"Navigating to {station}...")
            station_pose = self.station_positions[station]

            # Transform station pose from world to map frame
            goal_data = self.get_station_dock_pose(station)
            goal_pose = self.convert(goal_data, station_pose)
            if goal_pose is not None:
                self.current_goal_reached = False
                self.send_goal(goal_pose)

                # Wait until the goal is reached
                while not self.current_goal_reached and not rospy.is_shutdown():
                    # rospy.loginfo(f"Waiting to reach {station}...")
                    rospy.sleep(1)
            else:
                rospy.logwarn(f"Failed to transform {station} position!")

    def get_station_dock_pose(self, name):
        dock_positions_data = {
            "station1": {
                "position": [-3.4334793090820312, -3.0196692943573, 0.0],
                "orientation": [0.0, 0.0, -0.9999649318479376, 0.008374668611330374]
            },
            "station2": {
                "position": [-3.7818751335144043, -0.44609326124191284, 0.0],
                "orientation": [0.0, 0.0, -0.999964925857652, 0.008375383841982838]
            },
            "station3": {
                "position": [-2.6103591918945312, -0.12560981512069702, 0.0],
                "orientation": [0.0, 0.0, 0.7130041495358447, 0.7011598125567856]
            },
            "station4": {
                "position": [0.4878208637237549, 0.8642598390579224, 0.0],
                "orientation": [0.0, 0.0, 0.7191272648311571, 0.6948783900629367]
            },
            "station5": {
                "position": [-4.965003967285156, 4.024663925170898, 0.0],
                "orientation": [0.0, 0.0, -0.7441068939415117, 0.6680605738918559]
            },
            "station6": {
                "position": [5.37608528137207, 2.3630504608154297, 0.0],
                "orientation": [0.0, 0.0, -0.029158487701653014, 0.9995748009003391]
            },
            "station7": {
                "position": [5.833176136016846, 4.74932861328125, 0.0],
                "orientation": [0.0, 0.0, 0.7309561824619074, 0.6824243982454867]
            },
            "station8": {
                "position": [2.495434522628784, 4.751615524291992, 0.0],
                "orientation": [0.0, 0.0, 0.7388400727311167, 0.6738808106235687]
            }
        }

        return dock_positions_data[name]

    def send_goal(self, pose):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
    
        rospy.loginfo(f"Goal {pose}")

        goal.pose.position.x = pose.position.x
        goal.pose.position.y = pose.position.y
        goal.pose.position.z = pose.position.z

        goal.pose.orientation.x = pose.orientation.x
        goal.pose.orientation.y = pose.orientation.y
        goal.pose.orientation.z = pose.orientation.z
        goal.pose.orientation.w = pose.orientation.w

        self.goal_pub.publish(goal)
        rospy.loginfo("Goal published!")

if __name__ == '__main__':
    try:
        WaypointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint Navigator node terminated.")
