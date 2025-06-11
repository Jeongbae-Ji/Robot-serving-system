
# 🍽️ Robot Serving System

This project is based on ROS 2 and simulates a restaurant service robot in Gazebo. When a customer places an order through the GUI installed at each table, the robot delivers the ordered menu item from the kitchen to the corresponding table. All order details and sales data are stored in an SQL database, enabling efficient restaurant management and data analysis in the future.

---

## 📁 Project Structure

```
Robot_Serving_System/
└── B5_Serving_System/
    ├── map.yaml / map.pgm          # Restaurant map for SLAM/Navigation
    ├── src/                        # Source code for ROS2 nodes
    ├── demo video.webm             # Demo video file
    ├── README.md
    └── b-5 Serving_System.pptx     # Project presentation
```

---

## 🚀 How to Run

1. **Launch the simulation environment**  
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Launch navigation with a custom map**  
   > ⚠️ Replace the map path with your own full path  
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/your_name/map.yaml
   ```

3. **Run core service nodes**
   ```bash
   ros2 run robot_food_service order_table_node
   ros2 run robot_food_service kitchen_display_node
   ros2 run robot_food_service robot_control_node
   ```

---

## 💡 Features

- Autonomous navigation using ROS2 Navigation2
- Interactive order placement system
- Kitchen display integration
- Robot movement control for food delivery
