# ANSCER-Assignment

This ROS 2 package implements a solution for automated trajectory visualization and storage for an Autonomous Mobile Robot (AMR) navigation task. It is designed as part of the Robotics Software Developer assignment and meets the following objectives:

- **Package Creation:** Encapsulate the nodes in a ROS 2 package.
- **Custom Service Definition:** Define a custom service (`SaveTrajectory.srv`) to request saving trajectory data (filename and duration).
- **Trajectory Publisher and Saver Node:**
  - Subscribes to the robot's pose on `/robot_pose`
  - Accumulates pose data and publishes a visualization MarkerArray on `/trajectory_marker`
  - Provides a ROS 2 service `/save_trajectory` to save the last N seconds of trajectory data to a file (in JSON format)
- **Trajectory Reader and Publisher Node:**
  - Reads the saved trajectory file (`trajectory.json`)
  - Applies a transformation to the trajectory (simulated offset, e.g. for frame conversion to `odom`)
  - Publishes the transformed trajectory as a MarkerArray on `/trajectory_marker_transformed`
- **Configuration and Testing:**  
  - CMakeLists.txt and package.xml are configured with all dependencies.  
  - A launch file is provided to start both nodes along with RViz.

---

## Table of Contents

- [Overview](#overview)
- [Problem Statement & Scenario](#problem-statement--scenario)
- [Package Structure](#package-structure)
- [Building the Package](#building-the-package)
- [Usage Instructions](#usage-instructions)
  - [Launching the Nodes](#launching-the-nodes)
  - [Publishing Robot Poses](#publishing-robot-poses)
  - [Saving the Trajectory](#saving-the-trajectory)
  - [Visualizing in RViz](#visualizing-in-rviz)
- [Detailed Workflow](#detailed-workflow)
- [Trajectory File](#trajectory-file)
- [Video Demo](#video-demo)
---

## Overview

In a dynamic manufacturing environment, an AMR moves materials between production stations. For monitoring and analysis, it is crucial to capture and visualize the robot's trajectory automatically. This package automates:
- Trajectory data collection and visualization in RViz.
- Saving of trajectory data to a file upon user request.
- Reading and transforming stored trajectory data for further visualization.

---

## Problem Statement & Scenario

**Challenge:**  
Existing open-source modules often require manual intervention for trajectory visualization and data handling. This solution simplifies the process by:
- Automating visualization of the robot's path using RViz.
- Saving the trajectory data in a flexible file format (JSON, with potential extensions to CSV/YAML).

**Scenario:**  
In a busy manufacturing facility, an AMR navigates along predefined paths. The package helps operators quickly visualize the robot's trajectory in real time and store this data for analysis with minimal manual setup.

---

## Package Structure

```
└── trajectory_visualization
    ├── CMakeLists.txt
    ├── include
    │   └── trajectory_visualization
    ├── launch
    │   └── trajectory_visualization.launch.py
    ├── package.xml
    ├── rviz
    │   └── trajectory_visualization.rviz
    ├── src
    │   ├── trajectory_publisher_saver.cpp
    │   └── trajectory_reader_publisher.cpp
    └── srv
        └── SaveTrajectory.srv

```

- **CMakeLists.txt / package.xml:** Define dependencies (e.g., rclcpp, geometry_msgs, visualization_msgs, tf2, nlohmann_json).
- **Launch File:** Starts both nodes and RViz with a preconfigured RViz file.
- **RViz Configuration:** Sets up MarkerArray displays for both the original and transformed trajectories.
- **Nodes:**
  - **trajectory_publisher_saver.cpp:** Collects pose data, publishes a live trajectory marker, and saves data via a service call.
  - **trajectory_reader_publisher.cpp:** Reads the saved trajectory file, transforms the data, and publishes it for visualization.
- **Service Definition:** `srv/SaveTrajectory.srv` allows users to specify a filename and duration.

---

## Building the Package

1. Clone the repository into your ROS 2 workspace:
   ```
   cd ~/workshop_ws/src
   git clone https://github.com/Varun-Ajith/ANSCER-Assignment.git trajectory_visualization
   ```

2. Build the package:
   ```
   cd ~/workshop_ws
   colcon build --packages-select trajectory_visualization
   source install/setup.bash
   ```

---

## Usage Instructions

### Launching the Nodes

Launch all nodes (publisher, reader, and RViz) with:
```
ros2 launch trajectory_visualization trajectory_visualization.launch.py
```
This command starts:
- **Trajectory Publisher and Saver Node** (subscribes to `/robot_pose` and publishes `/trajectory_marker`).
- **Trajectory Reader and Publisher Node** (reads `trajectory.json` and publishes `/trajectory_marker_transformed`).
- **RViz:** Displays the MarkerArray markers.

### Publishing Robot Poses

Open a new terminal to simulate the robot's movement by publishing `PoseStamped` messages on `/robot_pose`. Use different positions to form a visible trajectory:
```
ros2 topic pub /robot_pose geometry_msgs/msg/PoseStamped -r 1 \
'{"header": {"frame_id": "map"}, "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}'
```
After a few seconds, change the position:
```
ros2 topic pub /robot_pose geometry_msgs/msg/PoseStamped -r 1 \
'{"header": {"frame_id": "map"}, "pose": {"position": {"x": 2.0, "y": 3.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}'
```
This creates a path that the saver node will record.

### Saving the Trajectory

After enough data is accumulated, call the save service in a new terminal:
```
ros2 service call /save_trajectory trajectory_visualization/srv/SaveTrajectory \
'{"filename": "trajectory.json", "duration": 20.0}'
```
A successful response will look like:
```
success: true
message: "Trajectory data saved successfully to trajectory.json"
```

### Visualizing in RViz

RViz is launched automatically with a configuration that displays:
- **Original Trajectory:** Published on `/trajectory_marker` (Frame: `map`).
- **Transformed Trajectory:** Published on `/trajectory_marker_transformed` (Frame: `odom`).

**RViz Setup:**
- Set **Fixed Frame** to `map` for original data or `odom` for transformed data.
- Verify that MarkerArray displays are enabled and subscribed to the correct topics.

---

## Detailed Workflow

1. **Launch the Package:**  
   Start the nodes and RViz using the provided launch file.

2. **Publish Poses:**  
   Simulate robot movement by publishing varying `PoseStamped` messages on `/robot_pose`.

3. **Data Accumulation:**  
   The **Trajectory Publisher and Saver** node collects pose data and publishes a live trajectory in RViz as a green line (using a LINE_STRIP marker).

4. **Save the Trajectory:**  
   After data collection, call the `/save_trajectory` service to write the last N seconds of data into `trajectory.json`.

5. **Read and Transform:**  
   The **Trajectory Reader and Publisher** node reads the saved file (upon startup), applies a transformation (e.g., an offset to simulate conversion to the `odom` frame), and publishes the transformed trajectory in RViz as a second marker.

6. **Visualization:**  
   Use RViz to view both the original and transformed trajectories by setting the Fixed Frame and adding MarkerArray displays.

---

## Trajectory File

When you call the save service, a file named `trajectory.json` is generated. An example snippet of the file format is:
```json
{
  "trajectory": [
    {
      "time": 1678262707.123456789,
      "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    },
    {
      "time": 1678262712.987654321,
      "position": { "x": 2.0, "y": 3.0, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    }
  ]
}
```
This file demonstrates that the trajectory data is stored in a structured JSON format, making it easy to parse and analyze.

---

## Video Demo

A video demonstration is included to showcase:
- How the nodes are launched.
- The process of publishing poses and accumulating trajectory data.
- The saving of the trajectory using the custom service.
- Visualizing both the original and transformed trajectories in RViz.
- A brief overview of the generated `trajectory.json` file.

**Video File:**  
Please refer to the ![ANSCER_simulation.mp4](ANSCER_simulation.mp4) file included in the repository  for a complete walkthrough of the functionality.

---

This README file aligns with the assignment requirements and provides clear instructions on setting up, using, and demonstrating the functionality of the trajectory visualization and storage package.
