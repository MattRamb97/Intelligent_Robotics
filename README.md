# Intelligent-Robotics

# Assignment 1

// Assignment_1

The code in the file is designed for a robot navigation task using ROS and Actionlib, specifically for controlling the TIAGO robot to move to a specified goal position. The program starts by including necessary header files such as for ROS functionality, for client-side action communication and a custom action header. It also includes headers for handling messages related to the robot's movement and pose.
The ActionClient type is defined to simplify interaction with the custom action server, which is responsible for controlling the robot's final position.
The feedbackCallback function is used to process and print real-time feedback from the robot, providing updates on its navigation progress.
In the main function, ROS is initialized, and the number of input arguments is checked to ensure the user provides the correct number of parameters (X, Y and theta) for the goal pose. An ActionClient object is created, which connects to the action server responsible for handling navigation tasks. The program waits for the server to be available and ready, then constructs a goal message using the input parameters for the desired X, Y, and theta coordinates. The robot is briefly given time to prepare before starting the movement towards the goal. The goal is then sent to the action server, and the feedbackCallback function is registered to receive updates on the robot's progress. The client waits for the action to be completed, with a timeout set to 60 seconds. If the robot reaches the goal within the time limit, the final state of the action is logged; otherwise, a warning is issued indicating that the action did not finish in time. The program ends after this, and the robot's navigation process is considered complete.

// Robot

The file defines a component responsible for handling requests from the assignment_1 client, guiding the TIAGO robot to navigate safely towards a target pose while avoiding obstacles. The code begins as before by including necessary libraries for ROS, Actionlib and message types, custom action messages and a new part; sensor data for obstacle detection using LaserScan. The MoveBaseClient and actionServer types are defined to simplify communication with the move_base action client and the final_pose action server, respectively.
The final_poseActionAction class is defined to encapsulate the core logic. It includes a NodeHandle, an action server for managing goals, and feedback and result message structures. The class constructor initializes the action server with a callback function executeCB and starts the server. The navigateToGoal method handles the navigation process, sending the robot to the target pose using MoveBaseClient. The method waits for the action server to become available, constructs a goal message with the provided X, Y and theta values, and sends it to the move_base action client. During navigation, feedback is published to update the client about the robot's progress, including its status ("STARTED NAVIGATING" or "FINISHED NAVIGATING"). Upon completion, if the robot successfully reaches the goal, a success message is logged; otherwise, an error is reported.
The executeCB method serves as the callback when a new goal is received. It checks for preemption requests, ensuring that the goal can be safely pursued, and calls the navigateToGoal method to guide the robot. If the goal is achieved, the result is marked as successful, and the action server is notified.
The feedbackCallback processes real-time navigation feedback, publishing updates to the action server about the robot's current position (X, Y) and status ("IS NAVIGATING").
Finally, in the main function, the ROS node is initialized, and an instance of the final_poseActionAction class is created to manage robot navigation. The node runs using ros::spin(), waiting for incoming action requests and handling them accordingly. The program completes when the ROS node is shut down.

// Scanner

The file implements a ROS node designed to detect obstacles in the environment using laser scan data from a robot's sensor. The program processes the incoming LaserScan messages to identify potential obstacles and publishes their positions as messages of a custom type, assignment_1::obstacle_msg.
The code begins with the inclusion of necessary headers for ROS functionalities, LaserScan for handling laser scan messages, and obstacle_msg for the custom message type. It also
includes standard libraries like cmath and vector for mathematical operations and data structures, respectively.
A class named CartesianPoint is defined to represent points in Cartesian coordinates, encapsulating X and Y values with corresponding getter methods. Global vectors obstacle_x_centers and obstacle_y_centers are declared to store the X and Y coordinates of detected obstacles.
The function isWithinCircle checks if all points in a given set are close to a specified circle, based on a defined radius and tolerance for deviation. This function is for validating whether detected points form a valid obstacle.
The detectCircle function is responsible for identifying circles formed by blocks of points. It requires at least three points to define a circle and calculates midpoints and slopes to find the circle's center and radius. It checks for concavity to ensure only valid convex shapes are processed. If the identified circle's radius falls within predefined limits, the center of the circle is recorded as an obstacle, and a message is logged.
The laserScanCallback function is called whenever a new LaserScan message is received. It extracts the number of points from the scan data and populates a vector of CartesianPoint objects representing the points in the robot's environment. The points are divided into blocks based on gaps in the data, which helps in processing them more efficiently. For each block with sufficient points, the detectCircle function is called to identify potential obstacles.
In the main function, the ROS node is initialized, and a subscriber is created to listen for LaserScan messages on the /scan_raw topic. A publisher is also established to send obstacle data using the assignment_1::obstacle_msg message type. The node runs in a loop, periodically publishing the detected obstacle positions while continuously processing incoming messages using ros::spinOnce().
This code integrates laser scan data processing to detect obstacles in the robot's environment, enabling further navigation and obstacle avoidance strategies for the TIAGO robot.

# Assignment_2

// node_a navigation

As outlined, we have predefined the initial positions for Tiago, the three objects to be manipulated (a blue hexagon, a red cube, and a green triangle), their respective delivery points, and a base position. This base serves as a return point after each object is handled, ensuring Tiago is in the correct position before starting the next task.
In the main function, the robot receives instructions from a human-controlled node on which object Tiago should search for. The loop runs a maximum of three times, which corresponds to the total number of objects in the environment. For each object ID provided by the human node, the Main function calls the objectPath function, which allows Tiago to complete the assigned task for each object.
The objectPath function is responsible for guiding Tiago through several steps:
• Moving towards the object’s location.
• Detecting the object.
• Picking up the object.
• Returning to the base position.
• Moving to the final delivery point, where Tiago will drop the object.
This function communicates with two other nodes: one for detecting the object and another for controlling Tiago's arm during the pickup and placement actions. The process for each object is determined by the object’s ID, ensuring Tiago executes the appropriate sequence
of movements and actions for the given object. Additionally, there are three key auxiliary functions:
• moveTiago: This function is responsible for moving Tiago to specific coordinates using ROS’s action client for navigation.
• detectionMessage and placeMessage: Both send messages to confirm when Tiago has successfully detected and place the object.

// node_b detection

In the main function, through the positionTorso function, we adjust Tiago's torso height to a suitable level for picking up objects.
At this point, using a loop with iterations equal to the number of objects to be picked up, which is three, we position Tiago's head specifically for each of the three objects using the positionHead function. This function repositions Tiago's head based on the object ID, using pre-defined coordinates for each of the three objects.
Then, we detect all the items near our ID, changing them pose thought the transformPose function that transforms the detected pose of an object from its reference frame to the robot's reference frame, and through a message, we are informed of how many objects are detected near our target ID, and they are saved in an array.
At this point with the last auxiliary function publishDetectionResult we send to the node_c moving_arm the information about the items.

// node_c moving_arm

The node_c.cpp file is responsible for handling the manipulation of objects in the fetch and delivery task for a Tiago robot using the MoveIt! framework. The node interfaces with the robot’s motion planning system, manipulates objects, and ensures safe grasping and placing of objects in a Gazebo simulation environment. The code operates in two main phases: the picking phase and the placing phase. During the picking phase, objects are detected, added to the planning scene, and picked up using the gripper. In the placing phase, the robot navigates to the correct table and places the object at the designated position.
During the picking phase, the node listens for messages on the "pick" topic, receiving information about the object ID and AprilTag detections. The detected objects are added to the MoveIt! planning scene as collision objects through the insertCollisionObject helper function, which generates geometric primitives such as cylinders, cones, or cubes to ensure collision-free motion during arm movements. Once the collision objects are set up, the controlArm function is used to plan and execute the arm's movement. This function
adjusts the arm's pose to reach the object, coordinates its movement for grasping, and calls the attachToGripper helper function to attach the object to the gripper in both the MoveIt! planning scene and the Gazebo simulation environment via the gazebo_ros_link_attacher service.
After successfully picking up the object, the node moves to the placing phase. Here, the node listens for messages on the "place" topic to determine the delivery location. The setDeliveryLocation function calculates the appropriate position on the table for placing the object based on its ID, setting the exact coordinates of the destination table. The controlArm function is again called to manage the arm's movements as it places the object. At this stage, it uses the detachFromGripper function to detach the object from the gripper and place it at the correct location on the delivery table, ensuring a successful drop-off.
Throughout the entire process, the node communicates with other nodes by publishing messages on the "move" topic to indicate the progress of the task. It also periodically cleans up the planning scene by removing the collision objects after each pick-and-place operation is completed. The main function continues this process in a loop, repeating the pick-and-place cycle for three objects before shutting down.
