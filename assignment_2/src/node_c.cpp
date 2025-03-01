//node_c moving_arm

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <tf/transform_listener.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


#include <gazebo_ros_link_attacher/Attach.h>

#include "assignment_2/pick.h"
#include "assignment_2/move.h"
#include "assignment_2/place.h"

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <string>
#include <vector>
#include <map>

// Global constants
const float PI = 3.14;
const std::vector<std::string> object_shape = {"tiago", "Hexagon", "Triangle", "cube"};
const std::string reference_frame = "base_footprint";
const std::vector<std::string> object_ids = {"table", "blue_hexagon", "green_triangle", "red_cube", "gold_obs_0", "gold_obs_1", "gold_obs_2", "gold_obs_3", "cylinder", "wall"}; 

// Dimensions for different collision objects
const std::vector<float> blue_hex_dimensions = {0.03, 0.1};
const std::vector<float> gold_hex_dimensions = {0.055, 0.3};
const std::vector<float> green_tri_dimensions = {0.04, 0.05};
const float red_cube_dimension = 0.04;
const std::vector<float> table_dimensions = {1, 1.5, 1};
const std::vector<float> wall_dimensions = {2.5, 0.5, 2};
const std::vector<float> delivery_dimensions = {0.25, 0.8};
const std::vector<float> delivery_pose = {0.75, 0, 0.36, 0.75, 1};


// Helper functions for each shape
void createCylinder(shape_msgs::SolidPrimitive* primitive, const float radius, const float height) {

    primitive->type = shape_msgs::SolidPrimitive::CYLINDER;

    primitive->dimensions.resize(2);
    primitive->dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;
    primitive->dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;  
}

void createCone(shape_msgs::SolidPrimitive* primitive, const float radius, const float height) {

    primitive->type = shape_msgs::SolidPrimitive::CONE;

    primitive->dimensions.resize(2);
    primitive->dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] = radius;
    primitive->dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = height;  
}

void createCube(shape_msgs::SolidPrimitive* primitive, const float depth, const float width, const float height) {

    primitive->type = shape_msgs::SolidPrimitive::BOX;

    primitive->dimensions.resize(3);
    primitive->dimensions[shape_msgs::SolidPrimitive::BOX_X] = depth;
    primitive->dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
    primitive->dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height; 
}


// Helper function to create primitives based on the object id
shape_msgs::SolidPrimitive createPrimitive(const int id, geometry_msgs::Pose& pose) {
    shape_msgs::SolidPrimitive primitive;

    switch (id) {
        case 1:
            createCylinder(&primitive, blue_hex_dimensions[0], blue_hex_dimensions[1]);
            break;
        case 2:
            createCone(&primitive, green_tri_dimensions[0], green_tri_dimensions[1]);
            // Set specific orientation for the cone
            pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, PI);
            break;
        case 3:
            createCube(&primitive, red_cube_dimension, red_cube_dimension, red_cube_dimension);
            break;
        case 0:
            createCube(&primitive, table_dimensions[0], table_dimensions[1], table_dimensions[2]);
            break;
        case 8:
            createCylinder(&primitive, delivery_dimensions[0], delivery_dimensions[1]);
            break;
        case 9:
            createCube(&primitive, wall_dimensions[0], wall_dimensions[1], wall_dimensions[2]);
            break;
        default:
            createCylinder(&primitive, gold_hex_dimensions[0], gold_hex_dimensions[1]);
            break;
    }

    return primitive;
}


// Function to insert collision objects into the planning scene
void insertCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene, const int id, const std::string& frame_id, geometry_msgs::Pose& pose) {

    moveit_msgs::CollisionObject collision_object;
    
    collision_object.header.frame_id = frame_id;
    collision_object.id = object_ids[id];

    // Create the primitive using the helper function
    shape_msgs::SolidPrimitive primitive = createPrimitive(id, pose);

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    // Add the object to the planning scene
    ros::Duration(2).sleep();  // Sleep to allow system to catch up
    planning_scene.addCollisionObjects({collision_object});  
}


// Helper function to attach an object in MoveIt! Planning Scene
void attachInMoveIt(moveit::planning_interface::PlanningSceneInterface& planning_scene, const std::string& object_id) {
    // Create an attached collision object
    moveit_msgs::AttachedCollisionObject attach_object;
    attach_object.object.id = object_id;
    attach_object.link_name = "arm_7_link";
    attach_object.object.operation = attach_object.object.ADD;

    // Remove the object from the collision list and attach it
    planning_scene.removeCollisionObjects({object_id});
    planning_scene.applyAttachedCollisionObject(attach_object);
}

// Helper function to set up the Gazebo attachment request
gazebo_ros_link_attacher::Attach createGazeboAttachmentRequest(const std::string& object_name, const std::string& robot_name) {
    gazebo_ros_link_attacher::Attach req;
    req.request.model_name_1 = object_name;
    req.request.link_name_1 = object_name + "_link";
    req.request.model_name_2 = robot_name;
    req.request.link_name_2 = "arm_7_link";
    return req;
}

// Main function to attach object to gripper
gazebo_ros_link_attacher::Attach attachToGripper(moveit::planning_interface::PlanningSceneInterface& planning_scene, const int id) {
    // Get the object ID and shape names
    const std::string& object_id = object_ids[id];
    const std::string& object_name = object_shape[id];
    const std::string& robot_name = object_shape[0];

    // Attach the object in MoveIt! Planning Scene
    attachInMoveIt(planning_scene, object_id);

    // Create and return the Gazebo attachment request
    return createGazeboAttachmentRequest(object_name, robot_name);
}


// Helper function to detach object from MoveIt! Planning Scene
void detachInMoveIt(moveit::planning_interface::PlanningSceneInterface& planning_scene) {
    // Create a detach object message
    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.link_name = "arm_7_link";
    detach_object.object.operation = detach_object.object.REMOVE;

    // Apply the detachment in the planning scene
    planning_scene.applyAttachedCollisionObject(detach_object);
}

// Helper function to set up the Gazebo detachment request
gazebo_ros_link_attacher::Attach createGazeboDetachmentRequest(const std::string& object_name, const std::string& robot_name) {
    gazebo_ros_link_attacher::Attach req;
    req.request.model_name_1 = object_name;
    req.request.link_name_1 = object_name + "_link";
    req.request.model_name_2 = robot_name;
    req.request.link_name_2 = "arm_7_link";
    return req;
}

// Main function to detach object from the gripper
gazebo_ros_link_attacher::Attach detachFromGripper(moveit::planning_interface::PlanningSceneInterface& planning_scene, const int id) {
    // Get object and robot names from arrays
    const std::string& object_name = object_shape[id];
    const std::string& robot_name = object_shape[0];  // Assuming object_shape[0] is the robot name

    // Detach object in MoveIt! Planning Scene
    detachInMoveIt(planning_scene);

    // Create and return Gazebo detachment request
    return createGazeboDetachmentRequest(object_name, robot_name);
}


// Function to control the gripper's movement
void moveGripper(float target_position) {
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_client("/parallel_gripper_controller/follow_joint_trajectory", true);
    //gripper_client.waitForServer();

	// Wait for the action server to start
    if (!gripper_client.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Gripper action server did not start.");
        return;
    }

	// Define the joint trajectory
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names.push_back("arm_7_joint");

	// Define the trajectory point
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(target_position);
    point.time_from_start = ros::Duration(1.0);

	// Add the trajectory point to the trajectory
    trajectory.points.push_back(point);

	// Create and send the goal
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    gripper_client.sendGoal(goal);

	// Wait and check the result of the action
    gripper_client.waitForResult();
    if (gripper_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Gripper movement succeeded.");
    } else {
        ROS_ERROR("Gripper movement failed.");
    }
}

// Function to execute the arm's planned movement
void executeMovementPlan(moveit::planning_interface::MoveGroupInterface& arm_group) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Start planning
    auto start_time = ros::Time::now();
    bool success = (arm_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    auto end_time = ros::Time::now();
    
    if (success) {
        ROS_INFO("Planning successful. Time taken: %.2f seconds", (end_time - start_time).toSec());

        // Try executing the plan
        moveit::planning_interface::MoveItErrorCode exec_result = arm_group.execute(plan);
        
        if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("Execution successful.");
        } else {
            ROS_ERROR("Execution failed.");
        }
    } else {
        ROS_WARN("Planning failed.");
    }
}


// Function to publish a message to another node
void publishMessage(ros::Publisher* publisher) {

	// Set the rate of message publishing to 10 Hz
    ros::Rate rate(10);
    
    for (int counter = 0; counter < 10 && ros::ok(); counter++) {
		// Instance of the move message 
        assignment_2::move move_msg;
		// Sets the start field of the message to true
        move_msg.start = true;
        publisher->publish(move_msg);
        ros::spinOnce();
        rate.sleep();
    }
}

// Function to transform coordinates from Gazebo to the robot's reference frame
void transformCoordinatesFromGazebo(geometry_msgs::Pose& pose) {

    // Apply the initial manual offset to the pose's position
    pose.position.x += 6.58;
    pose.position.y += -1.36;

    // Set up the TF listener to transform the pose
    tf::TransformListener listener;
    geometry_msgs::PoseStamped stamped_pose;
    
    // Wait for the transformation from "map" to the reference_frame
    if (!listener.waitForTransform("map", reference_frame, ros::Time(0), ros::Duration(3.0))) {
        ROS_ERROR("Could not find the transform from 'map' to '%s'", reference_frame.c_str());
        return;
    }

    // Set the pose in the "map" frame and apply a fixed orientation (90 degrees yaw)
    stamped_pose.header.frame_id = "map";
    stamped_pose.pose.position = pose.position;
    stamped_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.57);  // 90 degrees yaw
    
    // Transform the pose from "map" to the reference_frame
    geometry_msgs::PoseStamped transformed_pose;
    listener.transformPose(reference_frame, stamped_pose, transformed_pose);
    
    // Update the original pose with the transformed position and orientation
    pose.position = transformed_pose.pose.position;
    pose.orientation = transformed_pose.pose.orientation;
}

// Function to determine the delivery location for an object
void setDeliveryLocation(geometry_msgs::Pose& pose, int id) {

    //Coordinates from Gazebo
    const std::vector<float> delivery_coordinates = {6.007, 5.007, 4.007};
    pose.position.x = delivery_coordinates[id-1] - 0.1;
    pose.position.y = 1.016;
    pose.position.z = 0.346;
}


// Helper function to set target pose and move the arm
bool moveToPose(moveit::planning_interface::MoveGroupInterface& arm_group, const geometry_msgs::Pose& target_pose) {
    arm_group.setPoseTarget(target_pose);
    arm_group.setStartStateToCurrentState();
    moveit::planning_interface::MoveItErrorCode result = arm_group.move();
    return result == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

// Helper function to control the gripper and attach/detach objects
void controlGripper(moveit::planning_interface::PlanningSceneInterface& planning_scene, int object_id, bool is_picking) {
    if (is_picking) {
        gazebo_ros_link_attacher::Attach attach_request = attachToGripper(planning_scene, object_id);
        ros::service::call("/link_attacher_node/attach", attach_request);
		// Close gripper to pick
        moveGripper(0.0);  
    } else {
        gazebo_ros_link_attacher::Attach detach_request = detachFromGripper(planning_scene, object_id);
        ros::service::call("/link_attacher_node/detach", detach_request);
		// Open gripper to release
        moveGripper(1.0);  
    }
}

// Main control function for arm movement
void controlArm(moveit::planning_interface::MoveGroupInterface& arm_group, 
                moveit::planning_interface::PlanningSceneInterface& planning_scene, 
                const geometry_msgs::Pose& object_pose, 
                int object_id, 
                bool is_picking) {
    
    // Parameters
    const std::vector<float> z_offsets = {0.3, 0.163, 0.2345, 0.2075, 0.75, 0.65}; 
	//const std::vector<float> z_offsets = {0.3, 0.1, 0.1, 0.1, 0.75, 0.65}; 
    const float x_offset = 0.017; 
    const float roll_angle = -0.011; 
    const float pitch_angle = 1.57; 
    const std::vector<float> yaw_angles = {0.037, 0.8}; 

    const std::vector<double> intermediate_pose = {0.0, 0.25, -0.3, 2, 1.7, -1.5, 0.0};
    const std::vector<double> secure_pose = {0.0, -1.5, -0.3, 2, 1.7, -1.3, 0.0}; 
    const std::vector<float> wall_position = {-0.242, -1.341, 0.3};
    
    geometry_msgs::Pose target_pose = object_pose;
    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_angle, pitch_angle, yaw_angles[0]);
    ros::Rate loop_rate(0.2);

    // Setup for arm group movement parameters
    arm_group.setMaxVelocityScalingFactor(1.0);
    arm_group.setMaxAccelerationScalingFactor(1.0);
    arm_group.setNumPlanningAttempts(20.0); 
    arm_group.setPlanningTime(30.0); 
    arm_group.setPlannerId("SBLkConfigDefault");
    arm_group.setPoseReferenceFrame(reference_frame);

    // Add a virtual wall if object is the red cube (ID 3)
    if (object_id == 3) {
        geometry_msgs::Pose wall_pose;
        wall_pose.position.x = wall_position[0];
        wall_pose.position.y = wall_position[1];
        wall_pose.position.z = wall_position[2];
        insertCollisionObject(planning_scene, 9, reference_frame, wall_pose);
    }

    if (!is_picking) {
        // Placing the object
        arm_group.setJointValueTarget(intermediate_pose);
        executeMovementPlan(arm_group);

        // Adjust target z-position for placing the object
        target_pose.position.z = object_pose.position.z + z_offsets[5];
        moveToPose(arm_group, target_pose);

        // Control gripper to release the object
        controlGripper(planning_scene, object_id, false);

        // Insert collision object after placing
        geometry_msgs::Pose arm_pose = object_pose;
        arm_pose.position.z = z_offsets[4];
        insertCollisionObject(planning_scene, 1, reference_frame, arm_pose);
    } else {
        // Picking the object
        target_pose.position.z = object_pose.position.z + z_offsets[0];
        moveToPose(arm_group, target_pose);

        // If the object is the green triangle, apply extra movements
        if (object_id == 2) {
            target_pose.position.x += x_offset;
            target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_angle, pitch_angle, yaw_angles[1]); 
        }
        
        // Adjust target z-position for picking the object
        target_pose.position.z = object_pose.position.z + z_offsets[object_id];

        if (moveToPose(arm_group, target_pose)) {
			// Attach and close gripper
            controlGripper(planning_scene, object_id, true);  
        }

        // Lift object slightly after picking
        target_pose.position.z = object_pose.position.z + z_offsets[0]; 
        moveToPose(arm_group, target_pose);
        loop_rate.sleep();
    }

    // Move arm back to intermediate and secure poses
    arm_group.setJointValueTarget(intermediate_pose);
    executeMovementPlan(arm_group);
    arm_group.setJointValueTarget(secure_pose);
    executeMovementPlan(arm_group);
}

// Main function for the ROS node
int main(int argc, char** argv) {
    ros::init(argc, argv, "node_c");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(6); 
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    const std::vector<float> table_coords = {1.25, -1.61, 0.300};
    geometry_msgs::Pose table_pose; 
    geometry_msgs::Pose place_location;
    bool pick_phase;

    for (int k = 0; k < 3; k++) {

        pick_phase = true;
        boost::shared_ptr<assignment_2::pick const> pick_msg;
        pick_msg = ros::topic::waitForMessage<assignment_2::pick>("pick", nh);
        int object_id = pick_msg->id;
        std::vector<apriltag_ros::AprilTagDetection> detections = pick_msg->detections;
        
        // Add the collision objects to the planning scene
        for (int i = 0; i < detections.size(); i++) {
            insertCollisionObject(planning_scene_interface, detections[i].id[0], detections[i].pose.header.frame_id, detections[i].pose.pose.pose);
        }
        
        // Transform the table coordinates
        table_pose.position.x = table_coords[0];
        table_pose.position.y = table_coords[1];
        table_pose.position.z = table_coords[2];
        transformCoordinatesFromGazebo(table_pose);
        insertCollisionObject(planning_scene_interface, 0, reference_frame, table_pose);
        
        // Control the arm movement
        spinner.start();
        for (int i = 0; i < detections.size(); i++) {
            if (detections[i].id[0] == object_id) {
                controlArm(move_group, planning_scene_interface, detections[i].pose.pose.pose, object_id, pick_phase);
            }
        }

        pick_phase = false;
        spinner.stop();
        // Message to move to the delivery location
        ros::Publisher publisher = nh.advertise<assignment_2::move>("move", 1000);
        publishMessage(&publisher);
        
        // Clean the planning scene from previous collision objects
        std::vector<std::string> objects_to_remove = planning_scene_interface.getKnownObjectNames(false);
        planning_scene_interface.removeCollisionObjects(objects_to_remove);
        boost::shared_ptr<assignment_2::place const> place_msg;
        place_msg = ros::topic::waitForMessage<assignment_2::place>("place", nh);
        spinner.start();
        
        // Transform the cylinder coordinates
        setDeliveryLocation(place_location, object_id);
        transformCoordinatesFromGazebo(place_location);
        // Add the collision object to the planning scene
        insertCollisionObject(planning_scene_interface, 8, reference_frame, place_location);
        controlArm(move_group, planning_scene_interface, place_location, object_id, pick_phase);
        spinner.stop(); 
        // Clean the planning scene from previous collision objects
        objects_to_remove = planning_scene_interface.getKnownObjectNames(false);
        planning_scene_interface.removeCollisionObjects(objects_to_remove);
        // Communication with node_a
        publishMessage(&publisher);
    }
    
    ros::spin();
    ros::shutdown();
    return 0;
}

