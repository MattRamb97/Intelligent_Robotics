// This component handles requests from the assignment_1 server, calculating obstacle positions to successfully reach the desired final pose while avoiding any obstacles.
// It also provides updates back to assignment_1 throughout the process.
// Upon completion, it communicates the detected obstacle positions back to the server.

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "assignment_1/final_poseActionAction.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "sensor_msgs/LaserScan.h"
#include "assignment_1/obstacle_msg.h"
#include <sstream>
#include <string>

//DEFINITION OF THE MOVE BASE TYPE TO MOVE THE ROBOT
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ;
using actionServer = actionlib::SimpleActionServer<assignment_1::final_poseActionAction> ;

void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback,actionServer* action_server_);


class final_poseActionAction{
	protected:
		ros::NodeHandle nh_;
		actionServer action_server_;
		std::string action_name_;
		assignment_1::final_poseActionFeedback feedback_;
		assignment_1::final_poseActionResult result_;
	public:
		final_poseActionAction(std::string name) : action_server_(nh_,name,boost::bind(&final_poseActionAction::executeCB,this,_1),false),action_name_(name){
		action_server_.start();
	}
	
	// Navigation method
	void navigateToGoal(float x, float y, float theta){
		MoveBaseClient action_client_("move_base", true);
		move_base_msgs::MoveBaseGoal goal;

		const float correctionFactor = 1.0;//It avoids having not valid quaternions.

		while(!action_client_.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting to send the goal [%f,%f,%f] to the server...",x,y,theta);
		}
		
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.pose.position.x = x;
		goal.target_pose.pose.position.y = y ;
		if(theta<0)goal.target_pose.pose.orientation.z =-theta;
		else goal.target_pose.pose.orientation.z = theta;
		goal.target_pose.pose.orientation.w = theta;
		
		ROS_INFO("Sending goal: [%f, %f], orientation: [%f]", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.w);

		action_client_.sendGoal(goal,MoveBaseClient::SimpleDoneCallback(), MoveBaseClient::SimpleActiveCallback(), boost::bind(&feedbackCallback, _1,&action_server_));
		
		feedback_.status = "STARTED NAVIGATING";
		action_server_.publishFeedback(feedback_);
		action_client_.waitForResult();
		
		feedback_.status = "FINISHED NAVIGATING";
		action_server_.publishFeedback(feedback_);
		
		if(action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Successfully navigated to the desired position.");
		else
			ROS_ERROR("Failed to navigate.");
	}


	// Action execution callback
	void executeCB(const assignment_1::final_poseActionGoalConstPtr &goal){
		move_base_msgs::MoveBaseGoal tiagoGoal;

		bool success = true;
		for(int i=0;i<1;i++){
			if (action_server_.isPreemptRequested() || !ros::ok()) {
		        ROS_INFO("%s: Preempted", action_name_.c_str());
		        action_server_.setPreempted();
		        success = false;
		    }
			navigateToGoal(goal->pose.x,goal->pose.y,goal->pose.theta);
		}	
		
		if(success){
			result_.goal_reached = true;
			ROS_INFO("%s, succedeed",action_name_.c_str());
			action_server_.setSucceeded(result_);
		}
	}
	
	~final_poseActionAction(void){}
};

// Feedback callback
void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback, actionServer* action_server) {
        assignment_1::final_poseActionFeedback feedback_msg;
        feedback_msg.current_x = feedback->base_position.pose.position.x;
        feedback_msg.current_y = feedback->base_position.pose.position.y;
        feedback_msg.status = "IS NAVIGATING";
        action_server->publishFeedback(feedback_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot");
	final_poseActionAction final_pose("final_poseAction");
	ros::spin();
	return 0;
}
