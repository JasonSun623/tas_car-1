/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

#define PI 3.14159265359
using namespace std;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
}

/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation
    tf2::Quaternion quat;
/*
    geometry_msgs::Pose waypoint1;
    double offset = -0.1;

    
    
    waypoint1.position.x = 22.0;
    waypoint1.position.y = 19.4;
    waypoint1.position.z = 0.000;
    quat.setEuler(0.0,0.0,0.0*PI + offset);
    waypoint1.orientation.x = quat.getX();
    waypoint1.orientation.y = quat.getY();
    waypoint1.orientation.z = quat.getZ();
    waypoint1.orientation.w = quat.getW();
    waypoints.push_back(waypoint1);

    waypoint1.position.x = 23.5;
    waypoint1.position.y = 19.0;
    waypoint1.position.z = 0.000;
    quat.setEuler(0.0,0.0,-0.25*PI + offset);
    waypoint1.orientation.x = quat.getX();
    waypoint1.orientation.y = quat.getY();
    waypoint1.orientation.z = quat.getZ();
    waypoint1.orientation.w = quat.getW();
    waypoints.push_back(waypoint1);

    waypoint1.position.x = 24;
	waypoint1.position.y = 18.4;
	waypoint1.position.z = 0.000;
    quat.setEuler(0.0,0.0,-0.5*PI + offset);
	waypoint1.orientation.x = quat.getX();
	waypoint1.orientation.y = quat.getY();
	waypoint1.orientation.z = quat.getZ();
	waypoint1.orientation.w = quat.getW();
	waypoints.push_back(waypoint1);
*/
    //std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation


     	int reverse = 0; // driving direction
	int start = 0; 	 // selected start waypoint
	int wpcount = 2 * 24; // number of waypoints, times two if start position is not located at waypoint zero.
	int i,j;
	int loopindex = 0;


//*****defining waypoints*********

	
	if(reverse == 0){

		for(i = start; i<wpcount; i++){

		j = i;

		if(j > wpcount){
			j = i-wpcount;
		}

		if(loopindex < wpcount){

			if(j == 0){
				geometry_msgs::Pose waypoint1;
				waypoint1.position.x = 21.47;
				waypoint1.position.y = 19.41;
				waypoint1.position.z = 0.000;
				waypoint1.orientation.x = 0.0;
				waypoint1.orientation.y = 0.000;
				waypoint1.orientation.z = -0.00431352504905;
				waypoint1.orientation.w = 0.9990;
				waypoints.push_back(waypoint1);
			} else if(j == 1){
				geometry_msgs::Pose waypoint2;
				waypoint2.position.x = 22.15259;
				waypoint2.position.y = 19.2544711914;
				waypoint2.position.z = 0.000;
				waypoint2.orientation.x = 0.0;
				waypoint2.orientation.y = 0.000;
				waypoint2.orientation.z = -0.170379744029;
				waypoint2.orientation.w = 0.984789;
				waypoints.push_back(waypoint2); ;
			} else if(j == 2){
				geometry_msgs::Pose waypoint3;
				waypoint3.position.x = 23.1699;
				waypoint3.position.y = 18.5622;
				waypoint3.position.z = 0.000;
				waypoint3.orientation.x = 0.0;
				waypoint3.orientation.y = 0.000;
				waypoint3.orientation.z = -0.3922;
				waypoint3.orientation.w = 0.9198;
				waypoints.push_back(waypoint3); 
			} else if(j == 3){
				geometry_msgs::Pose waypoint4;
				waypoint4.position.x = 23.76817;
				waypoint4.position.y = 17.86629;
				waypoint4.position.z = 0.000;
				waypoint4.orientation.x = 0.0;
				waypoint4.orientation.y = 0.000;
				waypoint4.orientation.z = -0.53266;
				waypoint4.orientation.w = 0.84632;
				waypoints.push_back(waypoint4); 
			} else if(j == 4){
					geometry_msgs::Pose waypoint5;
				waypoint5.position.x = 24.0322;
				waypoint5.position.y = 16.73575;
				waypoint5.position.z = 0.000;
				waypoint5.orientation.x = 0.0;
				waypoint5.orientation.y = 0.000;
				waypoint5.orientation.z = -0.65814;
				waypoint5.orientation.w = 0.752892;
				waypoints.push_back(waypoint5); 
			} else if(j == 5){
				// new coordinates for small door after 2nd corner
				geometry_msgs::Pose waypoint6;
				waypoint6.position.x = 23.8138656616;
				waypoint6.position.y = 12.6736183167;
				waypoint6.position.z = 0.000;
				waypoint6.orientation.x = 0.0;
				waypoint6.orientation.y = 0.000;
				waypoint6.orientation.z = -0.731839682019;
				waypoint6.orientation.w = 0.681476837334;
				waypoints.push_back(waypoint6); 
			} else if(j == 6){
				geometry_msgs::Pose waypoint7;
				waypoint7.position.x = 23.5699291229;
				waypoint7.position.y = 9.45263767242;
				waypoint7.position.z = 0.000;
				waypoint7.orientation.x = 0.0;
				waypoint7.orientation.y = 0.000;
				waypoint7.orientation.z = -0.700271167259;
				waypoint7.orientation.w = 0.713876944792;
				waypoints.push_back(waypoint7); 
			} else if(j == 7){
				geometry_msgs::Pose waypoint8;
				waypoint8.position.x = 23.515045166;
				waypoint8.position.y = 8.60765838623;
				waypoint8.position.z = 0.000;
				waypoint8.orientation.x = 0.0;
				waypoint8.orientation.y = 0.000;
				waypoint8.orientation.z = -0.689495400189;
				waypoint8.orientation.w = 0.724290061452;
				waypoints.push_back(waypoint8); 
			} else if(j == 8){
				geometry_msgs::Pose waypoint9;
				waypoint9.position.x = 23.5662746429;
				waypoint9.position.y = 7.60303974152;
				waypoint9.position.z = 0.000;
				waypoint9.orientation.x = 0.0;
				waypoint9.orientation.y = 0.000;
				waypoint9.orientation.z = -0.70548447403;
				waypoint9.orientation.w = 0.670230981934;
				waypoints.push_back(waypoint9); 
			} else if(j == 9){
				// new goals for fucking door
				geometry_msgs::Pose waypoint10;
				waypoint10.position.x = 23.4078350067;
				waypoint10.position.y = 6.22215652466;
				waypoint10.position.z = 0.000;
				waypoint10.orientation.x = 0.0;
				waypoint10.orientation.y = 0.000;
				waypoint10.orientation.z = -0.742152565754;
				waypoint10.orientation.w = 0.68287463261;
				waypoints.push_back(waypoint10); 
			} else if(j == 10){
				geometry_msgs::Pose waypoint11;
				waypoint11.position.x = 22.8965625763;
				waypoint11.position.y = 5.53546524048;
				waypoint11.position.z = 0.000;
				waypoint11.orientation.x = 0.0;
				waypoint11.orientation.y = 0.000;
				waypoint11.orientation.z = 0.907038191825;
				waypoint11.orientation.w = -0.421048356571;
				waypoints.push_back(waypoint11); 
			} else if(j == 11){
				// new goals end
				geometry_msgs::Pose waypoint12;
				waypoint12.position.x = 22.0997390747;
				waypoint12.position.y = 5.36555290222;
				waypoint12.position.z = 0.000;
				waypoint12.orientation.x = 0.0;
				waypoint12.orientation.y = 0.000;
				waypoint12.orientation.z = 0.999817214686;
				waypoint12.orientation.w = -0.0191190276192;
				waypoints.push_back(waypoint12); 
			} else if(j == 12){
				geometry_msgs::Pose waypoint13;
				waypoint13.position.x = 21.877948761;
				waypoint13.position.y = 5.65591526031;
				waypoint13.position.z = 0.000;
			 	waypoint13.orientation.x = 0.0;
				waypoint13.orientation.y = 0.000;
				waypoint13.orientation.z = 0.999342467256;
				waypoint13.orientation.w =  -0.0362578700366;
				waypoints.push_back(waypoint13); 
			} else if(j == 13){
				geometry_msgs::Pose waypoint14;
				waypoint14.position.x = 13.079041;
				waypoint14.position.y = 6.00733;
				waypoint14.position.z = 0.0;
				waypoint14.orientation.x = 0.0;
				waypoint14.orientation.y = 0.000;
				waypoint14.orientation.z = 0.99998;
				waypoint14.orientation.w = 0.057269;
				waypoints.push_back(waypoint14); 
			} else if(j == 14){
				//3rd curve
				geometry_msgs::Pose waypoint15;
				waypoint15.position.x = 12.140117;
				waypoint15.position.y = 6.15295;
				waypoint15.position.z = 0.0;
				waypoint15.orientation.x = 0.0;
				waypoint15.orientation.y = 0.000;
				waypoint15.orientation.z = 0.97998;
				waypoint15.orientation.w = 0.1990821;
				waypoints.push_back(waypoint15); 
			} else if(j == 15){
				geometry_msgs::Pose waypoint16;
				waypoint16.position.x = 10.90514;
				waypoint16.position.y = 6.679841;
				waypoint16.position.z = 0.0;
				waypoint16.orientation.x = 0.0;
				waypoint16.orientation.y = 0.000;
				waypoint16.orientation.z = 0.9160376;
				waypoint16.orientation.w = 0.4010923;
				waypoints.push_back(waypoint16); 
			} else if(j == 16){
				geometry_msgs::Pose waypoint17;
				waypoint17.position.x = 10.348248;
				waypoint17.position.y = 7.5590996;
				waypoint17.position.z = 0.0;
				waypoint17.orientation.x = 0.0;
				waypoint17.orientation.y = 0.000;
				waypoint17.orientation.z = 0.81829600;
				waypoint17.orientation.w = 0.57479705;
				waypoints.push_back(waypoint17); 
			} else if(j == 17){
				geometry_msgs::Pose waypoint18;
				waypoint18.position.x = 9.985097;
				waypoint18.position.y = 8.182350;
				waypoint18.position.z = 0.0;
				waypoint18.orientation.x = 0.0;
				waypoint18.orientation.y = 0.000;
				waypoint18.orientation.z = 0.73606587;
				waypoint18.orientation.w = 0.67690990;
				waypoints.push_back(waypoint18); 
			} else if(j == 18){
				geometry_msgs::Pose waypoint19;
				waypoint19.position.x = 9.869755;
				waypoint19.position.y = 8.919201;
				waypoint19.position.z = 0.0;
				waypoint19.orientation.x = 0.0;
				waypoint19.orientation.y = 0.000;
				waypoint19.orientation.z = 0.7202055;
				waypoint19.orientation.w = 0.6937607;
				waypoints.push_back(waypoint19); 
			} else if(j == 19){
				geometry_msgs::Pose waypoint20;
				waypoint20.position.x = 10.414083;
				waypoint20.position.y = 16.60067;
			 	waypoint20.position.z = 0.0;
				waypoint20.orientation.x = 0.0;
				waypoint20.orientation.y = 0.000;
				waypoint20.orientation.z = 0.6951144;
				waypoint20.orientation.w = 0.7188990;
				waypoints.push_back(waypoint20); 
			} else if(j == 20){
				geometry_msgs::Pose waypoint21;
				waypoint21.position.x = 10.461678;
				waypoint21.position.y = 17.794158;
				waypoint21.position.z = 0.0;
				waypoint21.orientation.x = 0.0;
				waypoint21.orientation.y = 0.000;
				waypoint21.orientation.z = 0.5819110;
				waypoint21.orientation.w = 0.8132525;
				waypoints.push_back(waypoint21); 
			} else if(j == 21){
				//last edge
				geometry_msgs::Pose waypoint22;
				waypoint22.position.x = 10.869010;
				waypoint22.position.y = 18.629840;
				waypoint22.position.z = 0.0;
				waypoint22.orientation.x = 0.0;
				waypoint22.orientation.y = 0.000;
				waypoint22.orientation.z = 0.4282293;
				waypoint22.orientation.w = 0.9036396;
				waypoints.push_back(waypoint22); 
			} else if(j == 22){
				geometry_msgs::Pose waypoint23;
				waypoint23.position.x = 12.2722015381;
				waypoint23.position.y = 19.7042503357;
				waypoint23.position.z = 0.0;
				waypoint23.orientation.x = 0.0;
				waypoint23.orientation.y = 0.000;
				waypoint23.orientation.z = 0.154799187061;
				waypoint23.orientation.w = 0.987945955852;
				waypoints.push_back(waypoint23); 
			} else if(j == 23){
				geometry_msgs::Pose waypoint24;
				waypoint24.position.x = 15.120347023;
				waypoint24.position.y = 19.3760147095;
				waypoint24.position.z = 0.0;
				waypoint24.orientation.x = 0.0;
				waypoint24.orientation.y = 0.000;
				waypoint24.orientation.z = 0.0112774766628;
				waypoint24.orientation.w = 0.999936407238;
				waypoints.push_back(waypoint24);
			}
			loopindex++;

			}
		}
	///////////////clockwise end
	} else {
	////////////// counter clockwise
/*	    geometry_msgs::Pose waypoint1;
	    waypoint1.position.x = 13.0371351242;
	    waypoint1.position.y = 19.7142848969;
	    waypoint1.position.z = 0.0;
	    waypoint1.orientation.x = 0.0;
	    waypoint1.orientation.y = 0.000;
	    waypoint1.orientation.z = 0.999991116465;
	    waypoint1.orientation.w = 0.0042150908353;
	    waypoints.push_back(waypoint1);

	    waypoint1.position.x = 11.18616;
	    waypoint1.position.y = 19.1099681;
	    waypoint1.position.z = 0.0;
	    waypoint1.orientation.x = 0.0;
	    waypoint1.orientation.y = 0.000;
	    waypoint1.orientation.z = 0.940065125041;
	    waypoint1.orientation.w = -0.340994956974;
	    waypoints.push_back(waypoint1);

	    waypoint1.position.x = 10.74444;
	    waypoint1.position.y = 18.47312;
	    waypoint1.position.z = 0.0;
	    waypoint1.orientation.x = 0.0;
	    waypoint1.orientation.y = 0.000;
	    waypoint1.orientation.z = -0.748200207636;
	    waypoint1.orientation.w = 0.663473020773;
	    waypoints.push_back(waypoint1);

	    waypoint1.position.x = 10.4165325165;
	    waypoint1.position.y = 13.0590572357;
	    waypoint1.position.z = 0.0;
	    waypoint1.orientation.x = 0.0;
	    waypoint1.orientation.y = 0.000;
	    waypoint1.orientation.z = -0.747783451119;
	    waypoint1.orientation.w = 0.663942701016;
	    waypoints.push_back(waypoint1);

	    waypoint1.position.x = 9.91613006592;
	    waypoint1.position.y = 8.55067253113;
	    waypoint1.position.z = 0.0;
	    waypoint1.orientation.x = 0.0;
	    waypoint1.orientation.y = 0.000;
	    waypoint1.orientation.z = -0.682636338534;
	    waypoint1.orientation.w = 0.730758256411;
	    waypoints.push_back(waypoint1);

	    waypoint1.position.x = 10.5870800018;
	    waypoint1.position.y = 6.89875030518;
	    waypoint1.position.z = 0.0;
	    waypoint1.orientation.x = 0.0;
	    waypoint1.orientation.y = 0.000;
	    waypoint1.orientation.z = -0.40557681324;
	    waypoint1.orientation.w = 0.914060965452;
	    waypoints.push_back(waypoint1);

	    waypoint1.position.x = 12.1829147339;
	    waypoint1.position.y = 6.08992481232;
	    waypoint1.position.z = 0.0;
	    waypoint1.orientation.x = 0.0;
	    waypoint1.orientation.y = 0.000;
	    waypoint1.orientation.z = -0.0367410559939;
	    waypoint1.orientation.w = 0.999324819468;
	    waypoints.push_back(waypoint1);

	    waypoint1.position.x = 19.3452377319;
	    waypoint1.position.y = 5.53564834595;
	    waypoint1.position.z = 0.0;
	    waypoint1.orientation.x = 0.0;
	    waypoint1.orientation.y = 0.000;
	    waypoint1.orientation.z = 0.0247929321808;
	    waypoint1.orientation.w = 0.999692608012;
	    waypoints.push_back(waypoint1);
*/
	}

/////////// end of define waypoints



	


    MoveBaseClient ac("move_base", true); // action client to spin a thread by default

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

    for(int i = 0; i < waypoints.size(); ++i) { // loop over all goal points, point by point
        goal.target_pose.header.stamp = ros::Time::now(); // set current time
        goal.target_pose.pose = waypoints.at(i);
        ROS_INFO("Sending goal");
        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
        ac.waitForResult(); // wait for goal result

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal", i);
        } else {
            ROS_INFO("The base failed to move to %d goal for some reason", i);
        }
    }
    return 0;
}
