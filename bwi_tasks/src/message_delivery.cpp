
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <sound_play/sound_play.h>

#include <ros/ros.h>

#include <string>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

std::string sender;
std::string recipient;
std::string location;
std::string message;

void goToLocation() {  
	/*from go_to_location*/
	Client client("action_executor/execute_plan", true);
	client.waitForServer();
	  
	bwi_kr_execution::ExecutePlanGoal goal;
	  
	bwi_kr_execution::AspRule rule;
	bwi_kr_execution::AspFluent fluent;
	fluent.name = "not at";
	  
	fluent.variables.push_back(recipient);
	 
	rule.body.push_back(fluent);
	goal.aspGoal.push_back(rule);
	  
	ROS_INFO("sending goal");
	client.sendGoal(goal);
	  
	ros::Rate wait_rate(10);
	while(ros::ok() && !client.getState().isDone())
	  wait_rate.sleep();
	    
	if (!client.getState().isDone()) {
		ROS_INFO("Canceling goal");
		client.cancelGoal();
	    //and wait for canceling confirmation
		for(int i = 0; !client.getState().isDone() && i < 50; ++i)
			wait_rate.sleep();
	}
	if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
		ROS_INFO("Aborted");
	}
	else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
		ROS_INFO("Preempted");
	}
	else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Succeeded!");
	}
	else
		ROS_INFO("Terminated");
}

void sleepok(int t, ros::NodeHandle &nh)
{
	if (nh.ok())
		sleep(t);
}

void deliverMessage(ros::NodeHandle &nh) {
	sound_play::SoundClient sc;
	sleepok(2, nh);
	sc.say("Message delivery." + message);
	sleepok(2, nh);	
}

int main(int argc, char** argv) {
	//initialize the node
	ros::init(argc, argv, "message_delivery");

	ros::NodeHandle privateNode("~");

	privateNode.param<std::string>("location",location, /*default*/ ""); 
	privateNode.param<std::string>("message",message,"");

	goToLocation();
	deliverMessage(privateNode);
}
