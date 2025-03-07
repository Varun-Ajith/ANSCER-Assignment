#include <chrono>
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "trajectory_visualization/srv/save_trajectory.hpp"

using namespace std::chrono_literals;

class TrajectoryPublisherSaverNode : public rclcpp::Node
{
public:
  TrajectoryPublisherSaverNode()
  : Node("trajectory_publisher_saver")
  {
    // Subscribe to the robot pose topic
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "robot_pose", 10,
      std::bind(&TrajectoryPublisherSaverNode::pose_callback, this, std::placeholders::_1));

    // Publisher for the trajectory marker array
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_marker", 10);

    // Service to save trajectory data to a file
    save_service_ = this->create_service<trajectory_visualization::srv::SaveTrajectory>(
      "save_trajectory",
      std::bind(&TrajectoryPublisherSaverNode::handle_save_request, this,
                std::placeholders::_1, std::placeholders::_2));

    // Timer to periodically publish markers (every 500ms)
    marker_timer_ = this->create_wall_timer(
      500ms, std::bind(&TrajectoryPublisherSaverNode::publish_markers, this));

    RCLCPP_INFO(this->get_logger(), "Trajectory Publisher and Saver Node started.");
  }

private:
  // Subscriber, publisher, and service
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<trajectory_visualization::srv::SaveTrajectory>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  // Container for trajectory data (timestamp paired with pose)
  std::vector<std::pair<rclcpp::Time, geometry_msgs::msg::PoseStamped>> trajectory_;

  // Callback to store incoming poses
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    trajectory_.push_back({this->now(), *msg});
  }

  // Publish the trajectory as a MarkerArray for visualization in RViz
  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";  // assuming input poses are in the "map" frame
    line_marker.header.stamp = this->now();
    line_marker.ns = "trajectory";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05;  // line width
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    for (auto & data : trajectory_) {
      line_marker.points.push_back(data.second.pose.position);
    }

    marker_array.markers.push_back(line_marker);
    marker_pub_->publish(marker_array);
  }

  // Service callback: filter trajectory data for the specified duration and save to file
  void handle_save_request(
    const std::shared_ptr<trajectory_visualization::srv::SaveTrajectory::Request> request,
    std::shared_ptr<trajectory_visualization::srv::SaveTrajectory::Response> response)
  {
    // Compute cutoff time (current time minus requested duration)
    rclcpp::Time current_time = this->now();
    rclcpp::Time cutoff_time = current_time - rclcpp::Duration::from_seconds(request->duration);

    // Filter the stored poses
    std::vector<geometry_msgs::msg::PoseStamped> filtered_poses;
    for (auto & data : trajectory_) {
      if (data.first >= cutoff_time) {
        filtered_poses.push_back(data.second);
      }
    }

    if (filtered_poses.empty()) {
      response->success = false;
      response->message = "No trajectory data available for the specified duration.";
      RCLCPP_WARN(this->get_logger(), "Save request failed: %s", response->message.c_str());
      return;
    }

    // Write the filtered trajectory data to a JSON file
    std::ofstream ofs(request->filename);
    if (!ofs.is_open()) {
      response->success = false;
      response->message = "Failed to open file: " + request->filename;
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", request->filename.c_str());
      return;
    }

    ofs << "{\n  \"trajectory\": [\n";
    for (size_t i = 0; i < filtered_poses.size(); ++i) {
      const auto & pose = filtered_poses[i];
      ofs << "    {\n";
      ofs << "      \"time\": " << pose.header.stamp.sec << "." << std::setw(9)
          << std::setfill('0') << pose.header.stamp.nanosec << ",\n";
      ofs << "      \"position\": { \"x\": " << pose.pose.position.x << ", \"y\": " << pose.pose.position.y
          << ", \"z\": " << pose.pose.position.z << " },\n";
      ofs << "      \"orientation\": { \"x\": " << pose.pose.orientation.x << ", \"y\": " << pose.pose.orientation.y 
          << ", \"z\": " << pose.pose.orientation.z << ", \"w\": " << pose.pose.orientation.w << " }\n";
      ofs << "    }";
      if (i != filtered_poses.size() - 1) {
        ofs << ",";
      }
      ofs << "\n";
    }
    ofs << "  ]\n}\n";
    ofs.close();

    response->success = true;
    response->message = "Trajectory data saved successfully to " + request->filename;
    RCLCPP_INFO(this->get_logger(), "Trajectory data saved to: %s", request->filename.c_str());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPublisherSaverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
