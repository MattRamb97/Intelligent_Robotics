//node_a navigation

#include "ros/ros.h"
#include <stdlib.h>

#include "assignment_2/detection.h"
#include "assignment_2/move.h"
#include "assignment_2/place.h"

#include "assignment_1/final_poseActionAction.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tiago_iaslab_simulation/Objs.h"

using ActionClient = actionlib::SimpleActionClient<assignment_1::final_poseActionAction>;

// Constants from Gazebo
const std::vector<float> tiago_initialpose = {-6.58, 1.37};
const std::vector<float> tiago_basepose = {1.90 - tiago_initialpose[0], 1.25 - tiago_initialpose[1], -0.7};

const std::vector<float> blue_hexagon_position = {1.40 - tiago_initialpose[0], -0.85 - tiago_initialpose[1], -0.67};
const std::vector<float> red_cube_position = {0.86 - tiago_initialpose[0], -0.84 - tiago_initialpose[1], -0.67};
const std::vector<float> green_triangle_position = {1.12 - tiago_initialpose[0], -2.45 - tiago_initialpose[1], 0.67};


const std::vector<float> blue_Final = {5.85 - tiago_initialpose[0], 1.8 - tiago_initialpose[1], -0.83};
const std::vector<float> red_Final = {3.85 - tiago_initialpose[0], 1.66 - tiago_initialpose[1], -0.73};
const std::vector<float> green_Final = {4.85 - tiago_initialpose[0], 1.73 - tiago_initialpose[1], -0.68};



// Function to send messages
void detectionMessage(ros::Publisher* publisher, int object_id) {
    for (int i = 0; i < 150 && ros::ok(); ++i) {
        assignment_2::detection det_msg;
        det_msg.obj = object_id;
        publisher->publish(det_msg);
        ros::spinOnce();
    }
}

void placeMessage(ros::Publisher* publisher, int object_id) {
    for (int i = 0; i < 150 && ros::ok(); ++i) {
        assignment_2::place place_msg;
        place_msg.start = true;
        publisher->publish(place_msg);
        ros::spinOnce();
    }
}

// Function to move Tiago
void moveTiago(ActionClient* client, float x, float y, float w) {
    client->waitForServer();
    assignment_1::final_poseActionGoal goal;
    goal.pose.x = x;
    goal.pose.y = y;
    goal.pose.theta = w;
    client->sendGoal(goal);
    
    if (client->waitForResult(ros::Duration(60.0))) {
        ROS_INFO("Tiago has reached the target position.");
        actionlib::SimpleClientGoalState state = client->getState(); 
    } else {
        ROS_ERROR("Tiago failed the navigation");
    }
}

// Function to reach and position the object
void objectPath(ActionClient* client, int obj_id) {
    ros::NodeHandle nh1;
    ros::NodeHandle nh2;
    ros::Publisher pd = nh1.advertise<assignment_2::detection>("detection", 1000);
    ros::Publisher pp = nh1.advertise<assignment_2::place>("place", 1000);
    boost::shared_ptr<assignment_2::move const> msg;   

    moveTiago(client, tiago_basepose[0], tiago_basepose[1], tiago_basepose[2]);

    switch (obj_id) {
        case 1:
            ROS_INFO("Tiago is going to Blue hexagon"); 
            moveTiago(client, blue_hexagon_position[0], blue_hexagon_position[1], blue_hexagon_position[2]);
            detectionMessage(&pd,obj_id); 
            ROS_INFO("Tiago is picking the Blue hexagon");
            msg=ros::topic::waitForMessage<assignment_2::move>("move",nh2); 
            moveTiago(client, blue_Final[0], blue_Final[1], blue_Final[2]);
            placeMessage(&pp, obj_id);
            ROS_INFO("Tiago is putting the Blue hexagon");
            msg=ros::topic::waitForMessage<assignment_2::move>("move",nh2);
            break;
        case 2:
            ROS_INFO("Tiago is going to Green triangle");
            moveTiago(client, green_triangle_position[0]+ 1, green_triangle_position[1], green_triangle_position[2] -1.36);
            moveTiago(client, green_triangle_position[0], green_triangle_position[1], green_triangle_position[2]);
            detectionMessage(&pd,obj_id); 
            ROS_INFO("Tiago is picking the Green triangle");
            msg=ros::topic::waitForMessage<assignment_2::move>("move",nh2);
            moveTiago(client, green_triangle_position[0]+ 1, green_triangle_position[1], green_triangle_position[2] +1.36);
            moveTiago(client, tiago_basepose[0], tiago_basepose[1], -1 * tiago_basepose[2]);
            moveTiago(client, green_Final[0],  green_Final[1], green_Final[2]);
            placeMessage(&pp, obj_id);
            ROS_INFO("Tiago is putting the Green triangle");
            msg=ros::topic::waitForMessage<assignment_2::move>("move",nh2);
            break;
        case 3:
            ROS_INFO("Tiago is going to Red cube"); 
            moveTiago(client, red_cube_position[0], red_cube_position[1], red_cube_position[2]);
            detectionMessage(&pd,obj_id); 
            ROS_INFO("Tiago is picking the Red cube");
            msg=ros::topic::waitForMessage<assignment_2::move>("move",nh2);
            moveTiago(client, red_Final[0], red_Final[1], red_Final[2]);
            placeMessage(&pp, obj_id);
            ROS_INFO("Tiago is putting the Red cube");
            msg=ros::topic::waitForMessage<assignment_2::move>("move",nh2);
            break;
    }
}

//main of the node_a
int main(int argc, char** argv) {
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;
    ros::Rate loop_rate(0.33);
    std::vector<int> objid;
    ros::ServiceClient humanClient = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
    tiago_iaslab_simulation::Objs humanService;

    humanService.request.ready = true;
    humanService.request.all_objs = false;

    while (ros::ok() && objid.size() < 3) {
        if (humanClient.call(humanService)) {
            int obj_id = humanService.response.ids[0];
            if (std::find(objid.begin(), objid.end(), obj_id) == objid.end()) {
                objid.push_back(obj_id);
                ActionClient client("final_poseAction", true);
                objectPath(&client, obj_id);
            }
        } else {
            ROS_ERROR("Failed to communicate with the human node.");
        }
        ros::spinOnce();
    }
    ROS_INFO("Tiago has finished all tasks.");
    ros::shutdown();
    return 0;
}

