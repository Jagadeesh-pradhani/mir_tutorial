#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Float32
from std_srvs.srv import Empty
import logging
class WaypointNavigator:

    def __init__(self):
        rospy.init_node('mir_waypoint_navigator')

        self.stations_count = rospy.get_param('~stations_count', 4)

        self.stations = [f"station{i}" for i in range(1, 9)]
        self.stations_to_cover = [f"station{i}" for i in range(1, self.stations_count + 1)]
        self.station_positions = {}
        self.current_goal_reached = False
        self.int_goal_reached = False
        self.robot_pose = None
        self.battery_level = 100  # Initial battery level
        self.interrupted = False
        self.goal_index = 0
        self.mission_complete = False
        self.current_goal = None
        self.returning = False
        self.returned = False

        # Battery topic and reset service
        rospy.Subscriber('/battery_status', Float32, self.battery_callback)
        self.reset_battery_service = rospy.ServiceProxy('/reset_battery', Empty)

        # Subscribers and publishers
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # TF listener
        self.tf_listener = tf.TransformListener()

        rospy.loginfo("Waiting for station positions...")
        rospy.sleep(2)  # Allow time to populate station positions

        self.start_navigation()

    def battery_callback(self, data):
        self.battery_level = data.data
        rospy.logwarn(f"returning : {self.returning}, returned : {self.returned}, mission : {self.mission_complete}")
        rospy.logwarn(f"Battery : {self.battery_level},")
        # if self.battery_level < 85.0 and not self.interrupted:
        #     if not self.returning and not self.returned:
        #         rospy.logwarn("Battery low! Returning to starting position.")
        #         self.interrupted = True
        #         self.return_to_start()
        #     if not self.mission_complete and self.returned:
        #         self.returned = False
        #         self.send_goal(self.current_goal)

    def model_states_callback(self, data):
        try:
            for name, pose in zip(data.name, data.pose):
                if name == "mir":
                    self.robot_pose = pose
                if name in self.stations:
                    self.station_positions[name] = pose
        except IndexError:
            rospy.logwarn("Error processing model states data!")

    def result_callback(self, result):
        if result.status.status == 3:  # 3 indicates "Goal reached successfully"
            
            if self.waiting_r:
                rospy.loginfo("Returned home!")
                self.int_goal_reached = True
            else:
                rospy.loginfo("Goal reached!")
                self.current_goal_reached = True

    def start_navigation(self):
        for station in self.stations_to_cover:
            if self.interrupted:
                rospy.loginfo("Task interrupted. Waiting to resume.")
                while self.interrupted and not rospy.is_shutdown():
                    rospy.sleep(1)

            if station not in self.station_positions:
                rospy.logwarn(f"Position for {station} not found!")
                continue

            rospy.loginfo(f"Navigating to {station}...")
            station_pose = self.get_station_dock_pose(station)
            if station_pose:
                self.current_goal_reached = False
                self.send_goal(station_pose)
                self.current_goal = station_pose
                self.waiting_r = False

                while not self.current_goal_reached and not self.returning and not rospy.is_shutdown():
                    if self.battery_level < 85.0 and not self.interrupted:
                        if not self.returning and not self.returned:
                            rospy.logwarn("Battery low! Returning to starting position.")
                            self.interrupted = True
                            self.return_to_start()
                        if not self.mission_complete and self.returned:
                            rospy.logwarn(f"Continue mission")
                            self.returned = False
                            self.waiting_r = False
                            self.send_goal(self.current_goal)
                    rospy.logwarn(f"Waiting C: {self.current_goal_reached},")
                    rospy.sleep(1)
            else:
                rospy.logwarn(f"Failed to transform {station} position!")
            rospy.logwarn(f"One waypoint-completed")
        self.mission_complete = True
        self.return_to_start()
        rospy.loginfo("All stations covered... Mission completed")

    def return_to_start(self):
        starting_station = "station8"
        self.returning = True
        if starting_station not in self.station_positions:
            rospy.logwarn(f"Starting position ({starting_station}) not found!")
            return

        rospy.loginfo("Navigating to starting position...")
        start_pose = self.get_station_dock_pose(starting_station)
        if start_pose:
            self.int_goal_reached = False
            self.waiting_r = True
            self.send_goal(start_pose)

            while not self.int_goal_reached and not rospy.is_shutdown():
                rospy.logwarn(f"Waiting R: {self.int_goal_reached},")
                rospy.sleep(1)

            rospy.loginfo("Reached starting position. Resetting battery.")
            self.reset_battery()

    def reset_battery(self):
        try:
            
            self.reset_battery_service()
            self.returned = True
            rospy.loginfo("Battery Charging.")
            rospy.sleep(5)
            rospy.loginfo("Battery reset successfully.")
            self.battery_level = 100
            self.interrupted = False
            self.returning = False
            
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to reset battery: {e}")

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
                "position": [0.023822665214538574, 3.5399768352508545, 0.0],
                "orientation": [0.0, 0.0, -0.6710064089191482, 0.7414515487807878]
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
                "position": [2.4377570152282715, 4.870716571807861, 0.0],
                "orientation": [0.0, 0.0, -0.7071067966408575, 0.7071067657322372]
            }
        }

        return dock_positions_data.get(name)

    def send_goal(self, pose_data):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()

        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = pose_data["position"]
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = pose_data["orientation"]

        self.goal_pub.publish(goal)
        rospy.loginfo("Goal published!")

if __name__ == '__main__':
    try:
        WaypointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint Navigator node terminated.")