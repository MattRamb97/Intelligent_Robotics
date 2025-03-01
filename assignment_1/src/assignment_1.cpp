#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "assignment_1/final_poseActionAction.h"
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using ActionClient = actionlib::SimpleActionClient<assignment_1::final_poseActionAction>;

void feedbackCallback(const assignment_1::final_poseActionFeedbackConstPtr& feedback)
{
	ROS_INFO("Current position TIAGO: X=%.2f, Y=%.2f | TIAGO State: %s ",feedback->status.c_str(), feedback->current_x, feedback->current_y);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "assignment_1");
	
	if (argc!=4){ROS_ERROR("You have insert an incorrect number of parameters. Use: <program> x y theta"); return EXIT_FAILURE;}
	ActionClient client("final_poseAction",true);
	ROS_INFO("Wait for the server...");
	client.waitForServer();
	ROS_INFO("Action server started. Sending goal...");
	
	assignment_1::final_poseActionGoal goal;
	
	// Parse and set goal coordinates
    goal.pose.x = atof(argv[1]);
    goal.pose.y = atof(argv[2]);
    goal.pose.theta = atof(argv[3]);
	
	// Wait briefly for the robot to prepare	
	//const ros::Duration wait_time(1);
	const float estimatedWaitingTuckTime = 0.1;
	assignment_1::final_poseActionFeedback feedback;
	ros::Rate waiting(estimatedWaitingTuckTime);
	ROS_INFO("Waiting for TIAGO to prepare itself...");
	waiting.sleep();
	ROS_INFO("The robot start move.");
	
	// Send goal with feedback callback
	client.sendGoal(goal, ActionClient::SimpleDoneCallback(), ActionClient::SimpleActiveCallback(), boost::bind(&feedbackCallback, _1));
	
	// Wait for the action to finish with a timeout
	bool completed_in_time = client.waitForResult(ros::Duration(60.0));
	
	if(completed_in_time)
	{
		actionlib::SimpleClientGoalState state = client.getState();
		ROS_INFO("Action completed: %s",state.toString().c_str());
	}
	else
	{
		ROS_WARN("Action did not finish within the timeout period.");
	}
	
//return EXIT_SUCCESS;
}
