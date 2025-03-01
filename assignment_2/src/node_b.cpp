//node_b detection

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "assignment_2/detection.h"
#include "assignment_2/pick.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "tf/transform_listener.h"

// Joint velocity constant for movements
constexpr float joint_velocity = 0.1f;

using torsoClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
using headClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

// Adjusts the robot's torso height by sending a goal to the torso action client
void adjustTorsoHeight(float height) {

    torsoClient torso_client("/torso_controller/follow_joint_trajectory", true);
    if (!torso_client.waitForServer(ros::Duration(3.0))) {
        ROS_ERROR("Unable to connect to torso controller");
        return;
    }

    control_msgs::FollowJointTrajectoryGoal torso_goal;
    torso_goal.trajectory.joint_names.push_back("torso_lift_joint");
    torso_goal.trajectory.points.resize(1);
    torso_goal.trajectory.points[0].positions.resize(1);
    torso_goal.trajectory.points[0].velocities.resize(1);
    torso_goal.trajectory.points[0].positions[0] = height;
    torso_goal.trajectory.points[0].velocities[0] = joint_velocity;
    torso_goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

    torso_client.sendGoal(torso_goal);
    bool goal_finished = torso_client.waitForResult(ros::Duration(30.0));
    if (goal_finished) {
        ROS_INFO("Torso adjustment successful.");
    } else {
        ROS_ERROR("Torso adjustment timeout.");
    }
}

// Function to move Tiago's head based on the object ID. Head positions are pre-defined.
void positionHead(int object_id) {
    const std::vector<float> blue_hexagon_position = {-0.3f, -1.6f};
    const std::vector<float> green_triangle_position = {-0.25f, -0.75f};
    const std::vector<float> red_cube_position = {0.5f, -0.8f};


    headClient head_client("/head_controller/follow_joint_trajectory", true);

    if (!head_client.waitForServer(ros::Duration(3.0))) {
        ROS_ERROR("Unable to connect to torso controller");
        return;
    }

    control_msgs::FollowJointTrajectoryGoal head_goal;
    head_goal.trajectory.joint_names.push_back("head_1_joint");
    head_goal.trajectory.joint_names.push_back("head_2_joint");
    head_goal.trajectory.points.resize(1);
    head_goal.trajectory.points[0].positions.resize(2);
    head_goal.trajectory.points[0].velocities.resize(2);

    switch (object_id) {
        case 1:
            head_goal.trajectory.points[0].positions[0] = blue_hexagon_position[0];
            head_goal.trajectory.points[0].positions[1] = blue_hexagon_position[1];
            break;
        case 2:
            head_goal.trajectory.points[0].positions[0] = green_triangle_position[0];
            head_goal.trajectory.points[0].positions[1] = green_triangle_position[1];
            break;
        case 3:
            head_goal.trajectory.points[0].positions[0] = red_cube_position[0];
            head_goal.trajectory.points[0].positions[1] = red_cube_position[1];
            break;
    }

    head_goal.trajectory.points[0].velocities[0] = joint_velocity;
    head_goal.trajectory.points[0].velocities[1] = joint_velocity;
    head_goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

    head_client.sendGoal(head_goal);
    bool head_goal_finished = head_client.waitForResult(ros::Duration(30.0));
    if (head_goal_finished) {
        ROS_INFO("Head movement successful.");
    } else {
        ROS_ERROR("Head movement timeout.");
    }
}

// Converts the detected pose to the robot's reference frame using `tf`
apriltag_ros::AprilTagDetection transformPose(const std::string& target_frame, apriltag_ros::AprilTagDetection detection) {
    tf::TransformListener listener(ros::Duration(10.0));
    geometry_msgs::PoseStamped input_pose, transformed_pose;
    apriltag_ros::AprilTagDetection result_detection;

    try {
        listener.waitForTransform(target_frame, detection.pose.header.frame_id, ros::Time(0), ros::Duration(3.0));

        input_pose.header.frame_id = detection.pose.header.frame_id;
        input_pose.pose = detection.pose.pose.pose;

        listener.transformPose(target_frame, input_pose, transformed_pose);

        result_detection.id = detection.id;
        result_detection.size = detection.size;
        result_detection.pose.pose.pose = transformed_pose.pose;
        result_detection.pose.pose.covariance.fill(0.0);
        result_detection.pose.header.frame_id = transformed_pose.header.frame_id;
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Pose transformation error: %s", ex.what());
    }

    return result_detection;
}

// Sends the detection results to the moving_arm node via the ROS publisher
void publishDetectionResult(ros::Publisher* publisher, int object_id, const std::vector<apriltag_ros::AprilTagDetection>& detections) {
    ros::Rate rate(10);
    assignment_2::pick message;
    message.detections = detections;
    message.id = object_id;

    for (int i = 0; i < 10 && ros::ok(); ++i) {
        publisher->publish(message);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "detection_node");
    ros::NodeHandle nh;

    const float torso_height = 1.25f;
    const std::string target_frame = "base_footprint";
    int max_iterations = 3;
    bool initial_lift = true;

    std::vector<apriltag_ros::AprilTagDetection> detected_objects;

    ros::Publisher pick_publisher = nh.advertise<assignment_2::pick>("pick", 1000);

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        ROS_INFO("Waiting for detection on the detection topic.");
        boost::shared_ptr<assignment_2::detection const> detection_msg = ros::topic::waitForMessage<assignment_2::detection>("detection", nh);
        int object_id = detection_msg->obj;
        ROS_INFO("Detected object ID: %d", object_id);

        if (initial_lift) {
            adjustTorsoHeight(torso_height);
            initial_lift = false;
        }

        positionHead(object_id);

        boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> detection_array = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", nh);
        
        for(int i=0;i<detection_array->detections.size();i++){
            detected_objects.push_back(transformPose(target_frame, detection_array->detections[i]));
        }

        ROS_INFO("TIAGO detected %lu objects", detected_objects.size());

        publishDetectionResult(&pick_publisher, object_id, detected_objects);
        detected_objects.clear();
        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}


