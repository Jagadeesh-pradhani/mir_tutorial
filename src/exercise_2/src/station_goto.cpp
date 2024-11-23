#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <unordered_map>
#include <string>
#include <mutex>
std::mutex data_mutex_;

class WaypointNavigator
{
public:
    //! Constructor
    WaypointNavigator();

    void run();

    void populateStationDockPositions();


    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

    void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& result);

    void sendGoal(const geometry_msgs::Pose& pose);

    void startNavigation();

    geometry_msgs::Pose createPose(double px, double py, double pz, double ox, double oy, double oz, double ow);


private:

    ros::NodeHandle nh_;
    ros::Subscriber battery_sub_, model_states_sub_, result_sub_;
    ros::Publisher goal_pub_;
    ros::ServiceClient reset_battery_client_;

    int stations_count_;
    float battery_level_;
    bool interrupted_, mission_complete_, current_goal_reached_, int_goal_reached_, returning_, returned_, waiting_r_;

    geometry_msgs::Pose robot_pose_, current_goal_;
    std::vector<std::string> stations_;
    std::vector<std::string> stations_to_cover_;
    std::unordered_map<std::string, geometry_msgs::Pose> station_positions_, station_dock_positions_;


};


WaypointNavigator::WaypointNavigator()
    : nh_("~"),
    stations_count_(nh_.param<int>("stations_count", 4)),
    battery_level_(100.0),
    interrupted_(false),
    current_goal_reached_(false),
    int_goal_reached_(false),
    returning_(false),
    returned_(false),
    mission_complete_(false),
    waiting_r_(false) 
{
    for (int i = 1; i <= 8; ++i) {
        stations_.push_back("station" + std::to_string(i));
    }
    for (int i = 1; i <= stations_count_; ++i) {
        stations_to_cover_.push_back("station" + std::to_string(i));
    }

    ROS_INFO("Station count : %d",stations_count_);
    populateStationDockPositions();

    // Subscribers
    model_states_sub_ = nh_.subscribe("/gazebo/model_states", 10, &WaypointNavigator::modelStatesCallback, this);
    result_sub_ = nh_.subscribe("/move_base/result", 10, &WaypointNavigator::resultCallback, this);

    // Publishers
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);


    ROS_INFO("Waiting for station positions...");
    ros::Duration(4.0).sleep();


    
}

void WaypointNavigator::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    try {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "mir") {
                robot_pose_ = msg->pose[i];
                // ROS_INFO("Robot pose collecrted");
            } else if (station_positions_.find(msg->name[i]) != station_positions_.end()) {
                // ROS_INFO("Station pose collecrted");
                station_positions_[msg->name[i]] = msg->pose[i];
            }
        }
    } catch (const std::exception& e) {
        ROS_WARN("Error processing model states data: %s", e.what());
    }
}

void WaypointNavigator::resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& result) {
    if (result->status.status == 3) {
        ROS_INFO("Goal reached!");
        current_goal_reached_ = true;        
    }
}

void WaypointNavigator::sendGoal(const geometry_msgs::Pose& pose) {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose = pose;

    goal_pub_.publish(goal);
    ROS_INFO("Goal published!");
}

void WaypointNavigator::startNavigation() {
    for (const auto& station : stations_to_cover_) {
        if (station_positions_.find(station) == station_positions_.end()) {
            ROS_WARN("Position for %s not found!", station.c_str());
            while (station_positions_.find(station) != station_positions_.end()) {
                ros::Duration(1.0).sleep();
            }
        }
        ROS_INFO("Position for %s found!", station.c_str());
        ROS_INFO("Navigating to %s...", station.c_str());
        const auto& station_pose = station_dock_positions_[station];
        current_goal_reached_ = false;
        sendGoal(station_pose);

        while (!current_goal_reached_ && !returning_ && ros::ok()) {
            ros::Duration(1.0).sleep();
        }
        ROS_INFO("One way point completed");
    }
    mission_complete_ = true;
    ROS_INFO("All stations covered. Mission complete.");
}

void WaypointNavigator::populateStationDockPositions() {
    station_dock_positions_["station1"] = createPose(-3.4334793090820312, -3.0196692943573, 0.0, 0.0, 0.0, -0.9999649318479376, 0.008374668611330374);
    station_dock_positions_["station2"] = createPose(-3.7818751335144043, -0.44609326124191284, 0.0, 0.0, 0.0, -0.999964925857652, 0.008375383841982838);
    station_dock_positions_["station3"] = createPose(-2.6103591918945312, -0.12560981512069702, 0.0, 0.0, 0.0, 0.7130041495358447, 0.7011598125567856);
    station_dock_positions_["station4"] = createPose(0.023822665214538574, 3.5399768352508545, 0.0, 0.0, 0.0, -0.6710064089191482, 0.7414515487807878);
    station_dock_positions_["station5"] = createPose(-4.965003967285156, 4.024663925170898, 0.0, 0.0, 0.0, -0.7441068939415117, 0.6680605738918559);
    station_dock_positions_["station6"] = createPose(5.37608528137207, 2.3630504608154297, 0.0, 0.0, 0.0, -0.029158487701653014, 0.9995748009003391);
    station_dock_positions_["station7"] = createPose(5.833176136016846, 4.74932861328125, 0.0, 0.0, 0.0, 0.7309561824619074, 0.6824243982454867);
    station_dock_positions_["station8"] = createPose(2.4377570152282715, 4.870716571807861, 0.0, 0.0, 0.0, -0.7071067966408575, 0.7071067657322372);
}

geometry_msgs::Pose WaypointNavigator::createPose(double px, double py, double pz, double ox, double oy, double oz, double ow) {
    geometry_msgs::Pose pose;
    pose.position.x = px;
    pose.position.y = py;
    pose.position.z = pz;
    pose.orientation.x = ox;
    pose.orientation.y = oy;
    pose.orientation.z = oz;
    pose.orientation.w = ow;
    return pose;
}


void WaypointNavigator::run() {
    ros::AsyncSpinner spinner(4);  // Use 4 threads for callbacks
    spinner.start();

    startNavigation();  // Main navigation logic
    ros::waitForShutdown();  // Wait for shutdown signal
}


//! Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mir_waypoint_navigator");
    auto navigator = std::make_shared<WaypointNavigator>();
    // WaypointNavigator navigator;
    navigator->run();
    // ros::spin();
    return 0;

}
