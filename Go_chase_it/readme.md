
# Go Chase It! Project

This project involves designing and implementing a ball-chasing robot using **ROS**, **Gazebo**, and **C++**. It includes creating and testing Gazebo plugins, implementing ROS nodes for robot control, and integrating the system to work seamlessly.

---

## Table of Contents

- [Gazebo Plugins](#gazebo-plugins)
  - [Overview of Gazebo Plugins](#overview-of-gazebo-plugins)
  - [Implementing Gazebo Plugins](#implementing-gazebo-plugins)
    - [Differential Drive Plugin](#differential-drive-plugin)
    - [Lidar Plugin](#lidar-plugin)
    - [Camera Plugin](#camera-plugin)
- [ROS Nodes](#ros-nodes)
  - [drive_bot Node](#drive_bot-node)
  - [process_image Node](#process_image-node)
- [Launch Files](#launch-files)
  - [world.launch](#worldlaunch)
  - [ball_chaser.launch](#ball_chaserlaunch)
- [Testing the Complete System](#testing-the-complete-system)
- [Conclusion](#conclusion)

---

## Gazebo Plugins

### Overview of Gazebo Plugins
Gazebo plugins simulate the robot's components and extend functionality by adding features like sensor data processing and actuator control.

### Implementing Gazebo Plugins

#### Differential Drive Plugin
Simulates robot movement based on wheel joint velocities. Add this to your URDF:

```xml
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
        <namespace>my_robot</namespace>
        <param>robotNamespace</param>
        <param>robotName</param>
    </ros>
    <updateRate>100.0</updateRate>
    <leftJoint>left_wheel_joint</leftJoint>
    <rightJoint>right_wheel_joint</rightJoint>
    <odometryTopic>odom</odometryTopic>
    <publishOdometry>true</publishOdometry>
</plugin>
```

#### Lidar Plugin
Simulates a lidar sensor. Add this to your URDF:

```xml
<plugin name="lidar" filename="libgazebo_ros_laser.so">
    <ros>
        <namespace>my_robot</namespace>
        <param>robotNamespace</param>
        <param>robotName</param>
    </ros>
    <topicName>scan</topicName>
    <updateRate>10.0</updateRate>
    <frameName>laser_frame</frameName>
</plugin>
```

#### Camera Plugin
Simulates a camera sensor. Add this to your URDF:

```xml
<plugin name="camera" filename="libgazebo_ros_camera.so">
    <ros>
        <namespace>my_robot</namespace>
        <param>robotNamespace</param>
        <param>robotName</param>
    </ros>
    <imageTopicName>camera/image_raw</imageTopicName>
    <cameraInfoTopicName>camera/camera_info</cameraInfoTopicName>
    <updateRate>30.0</updateRate>
</plugin>
```

---

## ROS Nodes

### drive_bot Node
Controls the robot's movement based on service requests.

**Key Components**:
- **Service Definition**: Provides `ball_chaser/command_robot` service.
- **Request Parameters**:
  - `linear_x`: Linear velocity (forward/backward).
  - `angular_z`: Angular velocity (rotation).

**Implementation**:

1. Create the service file `DriveToTarget.srv`:
    ```plaintext
    # Request
    float64 linear_x
    float64 angular_z
    ---
    # Response
    string msg_feedback
    ```

2. Write the node in `drive_bot.cpp`:
    ```cpp
    #include "ros/ros.h"
    #include "geometry_msgs/Twist.h"
    #include "ball_chaser/DriveToTarget.h"

    ros::Publisher motor_command_publisher;

    bool handle_drive_request(ball_chaser::DriveToTarget::Request &req,
                              ball_chaser::DriveToTarget::Response &res) {
        geometry_msgs::Twist motor_command;
        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;

        motor_command_publisher.publish(motor_command);

        res.msg_feedback = "Moving the robot: linear_x = " + std::to_string(req.linear_x) +
                           ", angular_z = " + std::to_string(req.angular_z);
        ROS_INFO_STREAM(res.msg_feedback);
        return true;
    }

    int main(int argc, char** argv) {
        ros::init(argc, argv, "drive_bot");
        ros::NodeHandle n;

        motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

        ros::spin();
        return 0;
    }
    ```

3. Update `CMakeLists.txt`:
    ```plaintext
    add_service_files(
        FILES
        DriveToTarget.srv
    )

    generate_messages(
        DEPENDENCIES
        std_msgs
    )

    include_directories(
        ${catkin_INCLUDE_DIRS}
    )

    add_executable(drive_bot src/drive_bot.cpp)
    target_link_libraries(drive_bot ${catkin_LIBRARIES})
    add_dependencies(drive_bot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    ```

### process_image Node
Analyzes the camera images to detect the white ball and commands the robot to chase it.

**Implementation**:
- Write the node in `process_image.cpp`:
    ```cpp
   #include "ros/ros.h"
   #include "ball_chaser/DriveToTarget.h"
   #include <sensor_msgs/Image.h>
  
  // Define a global client that can request services
   ros::ServiceClient client;
  
  // This function calls the command_robot service to drive the robot in the specified direction
   void drive_robot(float lin_x, float ang_z)
   {
      // Create a service request and set the values for linear and angular velocities
      ball_chaser::DriveToTarget srv;
      srv.request.linear_x = lin_x;
      srv.request.angular_z = ang_z;

    // Call the service and check if it was successful
    if (client.call(srv))
    {
        ROS_INFO("Moving the robot - Linear X: %f, Angular Z: %f", lin_x, ang_z);
    }
    else
    {
        ROS_ERROR("Failed to call the service to drive the robot");
    }
  }
  
  // This callback function continuously executes and reads the image data
  void process_image_callback(const sensor_msgs::Image img)
  {
      int width = img.width;
      int height = img.height;
      const uint8_t* data = &img.data[0];
      int white_pixel = 255;
      int len_data = height * width * 3;

    int white_pixel_count_left = 0;
    int white_pixel_count_mid = 0;
    int white_pixel_count_right = 0;

    // Loop through each pixel in the image
    for (int i = 0; i < len_data; i += 3) // Step by 3 for each pixel (R, G, B)
    {
        // Check if the pixel is white
        if (data[i] == white_pixel && data[i + 1] == white_pixel && data[i + 2] == white_pixel)
        {
            int col = (i / 3) % width;  // Find the column index of the pixel

            // Count white pixels in the respective region
            if (col < width / 3)         // Left region
            {
                white_pixel_count_left++;
            }
            else if (col > width / 3 && col < 2 * width / 3) // Middle region
            {
                white_pixel_count_mid++;
            }
            else if (col > 2 * width / 3 )                          // Right region
            {
                white_pixel_count_right++;
            }
        }
    }

    // Decision based on the region with the highest number of white pixels
    if (white_pixel_count_left > white_pixel_count_mid && white_pixel_count_left > white_pixel_count_right)
    {
        // Move the robot to the left
        drive_robot(0.0, 0.9);
    }
    else if (white_pixel_count_right > white_pixel_count_mid && white_pixel_count_right > white_pixel_count_left)
    {
        // Move the robot to the right
        drive_robot(0.0, -0.9);
    }
    else if (white_pixel_count_mid > 0)
    {
        // Move the robot straight
        drive_robot(0.1, 0.0);
    }
    else
    {
        // Stop the robot if no white pixels are found
        drive_robot(0.0, 0.0);
    }
  }

  int main(int argc, char** argv)
  {
      // Initialize the process_image node and create a handle to it
      ros::init(argc, argv, "process_image");
      ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
  }


    ```

---

## Launch Files

### world.launch
Launches the Gazebo world:
```xml
<launch>
    <arg name="world_file" default="$(find my_robot)/worlds/my_world.world"/>
    <param name="robotNamespace" value="my_robot"/>
    <node name="gazebo" pkg="gazebo_ros" type="gazebo" args="$(arg world_file)" output="screen"/>
</launch>
```

### ball_chaser.launch
Launches the `drive_bot` and `process_image` nodes:
```xml
<launch>
    <node name="drive_bot" pkg="ball_chaser" type="drive_bot" output="screen"/>
    <node name="process_image" pkg="ball_chaser" type="process_image" output="screen"/>
</launch>
```

---

## Testing the Complete System

1. Launch the Gazebo World:
    ```bash
    $ roslaunch my_robot world.launch
    ```

2. Run the ball_chaser nodes:
    ```bash
    $ roslaunch ball_chaser ball_chaser.launch
    ```

3. Verify the system:
    - Check Lidar data: `rostopic echo /my_robot/scan`
    - Check Camera data: `rostopic echo /my_robot/camera/image_raw`

---

![Description](Go_chase_it/Go_chase_it_Video-ezgif.com-video-to-gif-converter.gif)

