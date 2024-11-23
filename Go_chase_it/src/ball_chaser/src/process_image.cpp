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

