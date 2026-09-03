# Wasted Potential - ERC 2026 Phase 1

This repository contains the solution for the Emirates Robotics Competition 2026 Phase 1 Simulation.

## How to Run the System

### Step 1: Start the Simulation (Gazebo)
Open **Terminal 1** and launch the core Gazebo simulation environment with the TIAGo Pro robot:
```bash
source /opt/erc_ws/install/setup.bash
ros2 launch erc_bringup simulation.launch.py
```
*(Wait ~15 seconds for Gazebo to fully load before moving to Step 2)*

### Step 2: Open RQT Image View (Visualization)
Open **Terminal 2** and launch RQT to see the live computer vision annotations:
```bash
source /opt/erc_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /erc/annotated_image
```

### Step 3: Launch the Full Solution
Open **Terminal 3** and launch our automated solution, which runs all perception and coordination nodes simultaneously:
```bash
source /opt/erc_ws/install/setup.bash
ros2 launch wasted_potential solution.launch.py shelf_column_number:=1 book_colour:=red
```

**Full Shelf Mapping**: When the robot arrives at a shelf (approx 3m away), the system will automatically scan all 4 colors simultaneously. It calculates the Z-depth to assign Rows (1-4), sorts the books horizontally to assign Columns (1-5), and generates a fully annotated map of all 20 books! A high-resolution snapshot is automatically saved to `/root/erc_images/full_shelf_map.jpg`.

The system will then autonomously approach the specific shelf column, dynamically align with the requested book, and wait for your authorization to execute the final physical grab.

---

## Utilities

### Authorize Grasping
When the visual system perfectly aligns the robot with the book, the system will pause and wait for your authorization. To proceed with the physical grab, publish to the following topic in a new terminal:
```bash
ros2 topic pub --once /erc/proceed_grasp std_msgs/msg/Bool '{data: true}'
```

### Re-recording Waypoints
If you ever need to adjust the exact position or angle the robot faces when scanning a shelf, you can use the Pose Saver utility:
1. Open a terminal and run manual teleop to drive the robot: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
2. Open another terminal and run the Pose Saver: `ros2 run wasted_potential save_current_pose.py`
3. Drive the robot to the perfect scanning position.
4. Type a name (e.g., `shelf3`) into the Pose Saver terminal and hit Enter to overwrite/save the coordinate to `~/waypoints.yaml`.
