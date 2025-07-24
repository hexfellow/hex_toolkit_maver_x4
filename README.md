
# **hex_toolkit_maver_x4**

## **Overview**

The **hex_toolkit_maver_x4** package provides a suite of tools for **Maver X4**, including URDF models and a simple control demo.

If you want to see a demo of the Rust version, visit the [rust-robot-demos](https://github.com/hexfellow/rust-robot-demos) repo.

### **Maintainer**

**Dong Zhaorui**: [847235539@qq.com](mailto:847235539@qq.com)

### **Verified Platforms**

- [x] **x64**
- [ ] **Jetson Orin Nano**
- [ ] **Jetson Orin NX**
- [ ] **Jetson AGX Orin**
- [ ] **Horizon RDK X5**
- [ ] **Rockchip RK3588**

### **Verified ROS Versions**

- [x] **Noetic**
- [x] **Humble**
- [ ] **Jazzy**

---

## **Getting Started**

### **Quick Start**

If you want to use this repository quickly, you can refer to the [ROS-Maver-x4](https://docs.hexfellow.com/hex-base/) documentation to configure the environment needed to run the repository.

### **Dependencies**

For **Hex Chassis** users, we highly recommend using this package within our **Hex Docker Images** to ensure compatibility and an optimized setup experience.

If you prefer to set up manually, please note that the repository may not function as expected. However, if you still choose to proceed, install the following dependencies:

1. **ROS/ROS2**  
   Follow the official [ROS Installation Guide](http://wiki.ros.org/ROS/Installation) or the [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

2. **hex_toolkit_general_chassis**  
   Follow the official [Installation Guide](https://github.com/hexfellow/hex_toolkit_general_chassis.git).

3. **robot_hardware_interface**

    Follow the official [Installation Guide](https://github.com/hexfellow/robot_hardware_interface.git).

### **Installation**

0. First, finish all steps in [Dependencies](#dependencies)

1. Create a ROS workspace and navigate to the `src` directory:

   ```bash
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone the required repositories:

   ```bash
   git clone https://github.com/hexfellow/hex_toolkit_general_chassis.git   # MUST READ THE README IN IT
   git clone https://github.com/hexfellow/robot_hardware_interface.git      # MUST READ THE README IN IT
   git clone https://github.com/hexfellow/hex_toolkit_maver_x4.git
   ```

3. Build the workspace:

   - **ROS 1**:

     ```bash
     cd ../
     catkin_make
     ```

   - **ROS 2**:

     ```bash
     cd ../
     colcon build
     ```

### **Pre-Execution Steps**

Source the appropriate setup file based on your ROS version:

- **ROS 1**:

  ```bash
  source devel/setup.bash --extend
  ```

- **ROS 2**:

  ```bash
  source install/setup.bash --extend
  ```

#### **Important Note**

During real vehicle operation, you must modify the target IP address in `config/bringup.yaml` to the actual IP of the vehicle.​Refer to the [ROS-Maver-x4](https://docs.hexfellow.com/hex-base/) documentation for specific configuration steps.

---

## **Nodes**

The package provides the following nodes:

- **PID Track**: Using PID to follow target pose.

---

### **PID Track**

The **PID Track** node employs Proportional-Integral-Derivative to trace target poses.

#### **Published Topics**

| Topic Name     | Message Type                 | Description                                    |
| -------------- | ---------------------------- | ---------------------------------------------- |
| `/unsafe_ctrl` | `geometry_msgs/Twist`        | Publishes `twist` commands without timestamps. |
| `/vel_ctrl`    | `geometry_msgs/TwistStamped` | Publishes `twist` commands with timestamps.    |

#### **Subscribed Topics**

| Topic Name      | Message Type                | Description                         |
| --------------- | --------------------------- | ----------------------------------- |
| `/chassis_odom` | `nav_msgs/Odometry`         | Subscribes to chassis odometry.     |
| `/target_pose`  | `geometry_msgs/PoseStamped` | Subscribes to target poses.         |

#### **Parameters**

| Parameter Name    | Data Type        | Description                                                            |
|-------------------|------------------|------------------------------------------------------------------------|
| `rate_ros`        | `double`         | Execution rate of the ROS node (Hz).                                   |
| `rate_odom`       | `double`         | Odometry rate of the chassis driver (Hz).                              |
| `model_base`      | `string`         | Frame ID of the chassis base.                                          |
| `model_odom`      | `string`         | Frame ID of the odometry.                                              |
| `limit_vel`       | `vector<double>` | Linear & angular velocity limits (m/s, rad/s).                         |
| `limit_acc`       | `vector<double>` | Linear & angular acceleration limits (m/s², rad/s²).                   |
| `obs_weights`     | `double`         | State observer weights.                                                |
| `trace_pid`       | `vector<double>` | PID gains [kp, ki, kd] for trajectory tracking.                        |
| `trace_err_limit` | `vector<double>` | Error thresholds [position, heading] for trajectory tracking (m, rad). |

---

## **Launch Files**

The package provides the following launch files:

- **Bringup**: **Maver X4** driver.
- **Joy Ctrl**: **Maver X4** driver with gamepad control.
- **Key Ctrl**: **Maver X4** driver with keyboard control.
- **Circle Track**: **Maver X4** driver with PID for circular trajectory following.
- **List Track**: **Maver X4** driver with PID for discrete-point trajectory following.

---

### **Bringup**

#### **Introduction**

Bringup **Maver X4**.

#### **Usage**

- **ROS 1**:

  ```bash
  roslaunch hex_toolkit_maver_x4 bringup.launch
  ```

- **ROS 2**:

  ```bash
  ros2 launch hex_toolkit_maver_x4 bringup.launch.py
  ```

---

### **Joy Ctrl**

#### **Introduction**

Bringup **Maver X4** and control the chassis using a gamepad.

#### **Usage**

- **ROS 1**:

  ```bash
  roslaunch hex_toolkit_maver_x4 joy_ctrl.launch
  ```

- **ROS 2**:

  ```bash
  ros2 launch hex_toolkit_maver_x4 joy_ctrl.launch.py
  ```

---

### **Key Ctrl**

#### **Introduction**

Bringup **Maver X4** and control the chassis using the keyboard.

#### **Usage**

- **ROS 1**:

  ```bash
  roslaunch hex_toolkit_maver_x4 key_ctrl.launch
  ```

- **ROS 2**:

  ```bash
  ros2 launch hex_toolkit_maver_x4 key_ctrl.launch.py
  ```

---

### **PID Track**

#### **Introduction**

Bringup **Maver X4** and use PID to follow a trajectory for testing.

#### **Usage**

- **ROS 1**:

  1. circular trajectory tracking

    ```bash
    roslaunch hex_toolkit_maver_x4 circle_trace.launch
    ```

  2. discrete-point trajectory tracking

    ```bash
    roslaunch hex_toolkit_maver_x4 list_trace.launch
    ```

- **ROS 2**:

    1. circular trajectory tracking

    ```bash
    ros2 launch hex_toolkit_maver_x4 circle_trace.launch.py
    ```

    2. discrete-point trajectory tracking

    ```bash
    ros2 launch hex_toolkit_maver_x4 list_trace.launch.py
    ```