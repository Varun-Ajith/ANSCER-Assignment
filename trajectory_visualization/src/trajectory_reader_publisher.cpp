#include <chrono>
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Include nlohmann/json header (make sure this library is available in your workspace)
#include "nlohmann/json.hpp"
using json = nlohmann::json;

using namespace std::chrono_literals;

struct TrajectoryPoint {
  rclcpp::Time time;
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Quaternion orientation;
};

class TrajectoryReaderPublisherNode : public rclcpp::Node
{
public:
  TrajectoryReaderPublisherNode()
  : Node("trajectory_reader_publisher")
  {
    // Declare a parameter for the trajectory file name (default: trajectory.json)
    this->declare_parameter<std::string>("trajectory_file", "trajectory.json");
    this->get_parameter("trajectory_file", trajectory_file_);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_marker_transformed", 10);

    // Read the trajectory data from file
    if (!read_trajectory_file(trajectory_file_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read trajectory file: %s", trajectory_file_.c_str());
    }

    // Timer to publish markers periodically (every 1 second)
    marker_timer_ = this->create_wall_timer(
      1000ms, std::bind(&TrajectoryReaderPublisherNode::publish_markers, this));

    RCLCPP_INFO(this->get_logger(), "Trajectory Reader and Publisher Node started.");
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  std::vector<TrajectoryPoint> trajectory_points_;
  std::string trajectory_file_;

  // Function to read and parse the JSON trajectory file
  bool read_trajectory_file(const std::string & filename)
  {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
      return false;
    }
    json j;
    try {
      ifs >> j;
    } catch (json::parse_error & e) {
      RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
      return false;
    }
    if (!j.contains("trajectory") || !j["trajectory"].is_array()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid JSON format: missing 'trajectory' array");
      return false;
    }
    for (auto & item : j["trajectory"]) {
      TrajectoryPoint point;
      // Note: the time field is not used in the transformation here.
      point.position.x = item["position"]["x"].get<double>();
      point.position.y = item["position"]["y"].get<double>();
      point.position.z = item["position"]["z"].get<double>();
      point.orientation.x = item["orientation"]["x"].get<double>();
      point.orientation.y = item["orientation"]["y"].get<double>();
      point.orientation.z = item["orientation"]["z"].get<double>();
      point.orientation.w = item["orientation"]["w"].get<double>();

      trajectory_points_.push_back(point);
    }
    return true;
  }

  // Simulated transformation: add a fixed offset (for example, x+1.0, y+2.0)
  geometry_msgs::msg::Point transform_to_odom(const geometry_msgs::msg::Point & point)
  {
    geometry_msgs::msg::Point transformed;
    transformed.x = point.x + 1.0;
    transformed.y = point.y + 2.0;
    transformed.z = point.z;
    return transformed;
  }

  // Publish the transformed trajectory as a MarkerArray in the "odom" frame
  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "odom";
    line_marker.header.stamp = this->now();
    line_marker.ns = "trajectory_transformed";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05;
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    for (const auto & pt : trajectory_points_) {
      geometry_msgs::msg::Point p = transform_to_odom(pt.position);
      line_marker.points.push_back(p);
    }

    marker_array.markers.push_back(line_marker);
    marker_pub_->publish(marker_array);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryReaderPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
