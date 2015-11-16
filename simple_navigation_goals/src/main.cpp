/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>

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

   double offset = -0.1;

     	int reverse = 0; // driving direction: 0 == clockwise(as before); 1 == counterclockwise(edit ackermann_vehicle.launch --> set "yaw" to 3.14)
	int start = 0; 	 // first move_base goal (number of waypoint)
	int wpcount = 18; 	// number of waypoints
	int i,j;
	int loopindex = 0; // increments after every waypoint. if loopindex == wpcount-1, one round has been completed

	int start_position = 2; // number of start position;

	struct points {
		float positionx;
		float positiony;
		float orientationz;
		float orientationw;
	};

	if(start_position == 1){
		start = 9;
		reverse = 1;
	} else if(start_position == 2){
		start = 0;
		reverse = 1;
	}

	points wp[wpcount];

	// defining waypoints

	wp[0].positionx = 23.1699; // passt
	wp[0].positiony = 18.5622;
	wp[0].orientationz = -0.3922;
	wp[0].orientationw = 0.9198;

	wp[1].positionx = 23.76817; // passt
	wp[1].positiony = 17.86629;
	wp[1].orientationz = -0.53266;
	wp[1].orientationw = 0.84632;

	wp[2].positionx = 24.0322; //passt
	wp[2].positiony = 16.73575;
	wp[2].orientationz = -0.65814;
	wp[2].orientationw = 0.752892;

	wp[3].positionx = 23.4373519897; //passt
	wp[3].positiony = 9.39782142639;
	wp[3].orientationz = -0.762017899634;
	wp[3].orientationw = 0.647555959465;

	wp[4].positionx = 23.4267253876; //passt
	wp[4].positiony = 7.39782142639;
	wp[4].orientationz = -0.742152565754;
	wp[4].orientationw = 0.68287463261;

	wp[5].positionx = 22.8083724976; //passt
	wp[5].positiony = 6.05123901367;
	wp[5].orientationz = 0.907038191825;
	wp[5].orientationw = -0.421048356571;

	wp[6].positionx =  21.8664417267;
	wp[6].positiony =  5.56505346298;
	wp[6].orientationz = 0.999342467256;
	wp[6].orientationw = -0.0362578700366;

	wp[7].positionx = 13.079041; // passt
	wp[7].positiony = 6.00733;
	wp[7].orientationz = 0.99998;
	wp[7].orientationw = 0.057269;

	wp[8].positionx = 11.5154514313;
	wp[8].positiony = 6.46096897125;
	wp[8].orientationz = 0.983820663527;
	wp[8].orientationw = 0.179156082834;

	wp[9].positionx = 10.348248; // passt 
	wp[9].positiony = 8.182350;
	wp[9].orientationz = 0.81829600;
	wp[9].orientationw = 0.57479705;

	wp[10].positionx = 10.869010; // passt
	wp[10].positiony = 16.9761638641;
	wp[10].orientationz = 0.624597872723;
	wp[10].orientationw = 0.780946539393;

	wp[11].positionx = 12.2722015381; // passt
	wp[11].positiony = 19.7042503357;
	wp[11].orientationz = 0.154799187061;
	wp[11].orientationw = 0.987945955852;

	wp[12].positionx = 15.120347023;// passt
	wp[12].positiony = 19.3760147095;
	wp[12].orientationz = 0.0112774766628;
	wp[12].orientationw = 0.999936407238;

// new coordinates for room.

	wp[13].positionx = 17.8811149597;// passt
	wp[13].positiony = 19.2989730835;
	wp[13].orientationz = 0.000432655286265;
	wp[13].orientationw = 0.999999906405;

	wp[14].positionx = 18.8447647095;// passt
	wp[14].positiony = 19.3355770111;
	wp[14].orientationz = -0.00581348130463;
	wp[14].orientationw = 0.999983101575;

	wp[15].positionx = 19.8447647095;// passt
	wp[15].positiony = 19.3155770111;
	wp[15].orientationz = -0.00581348130463;
	wp[15].orientationw = 0.999983101575;

	wp[16].positionx = 20.8811149597;// passt
	wp[16].positiony = 19.25;
	wp[16].orientationz = 0.000432655286265;
	wp[16].orientationw = 0.999999906405;

	wp[17].positionx = 21.8811149597;// passt
	wp[17].positiony = 19.2;
	wp[17].orientationz = 0.000432655286265;
	wp[17].orientationw = 0.999999906405;
/*
	wp[18].positionx = 22.5;// passt
	wp[18].positiony = 19.1;
	wp[18].orientationz = 0.000432655286265;
	wp[18].orientationw = 0.999999906405;
*/
	//localisation with wifi coordinates
	/*
	// example coordinates;
	float wp_wifix = 17.0;
	float wp_wifiy = 19.0;


	points error[wpcount];
	float radius[wpcount];

	float temp;
	float min;
	int index;

	for(i=0;i<wpcount;i++){
		error[i].positionx = wp[i].positionx - wp_wifix;
		error[i].positiony = wp[i].positiony - wp_wifiy;

		temp = (error[i].positionx*error[i].positionx)+(error[i].positiony*error[i].positiony);
	
		radius[i] = sqrt(temp);
	}


	min = radius[0];

	for(i=0;i<wpcount;i++){
		if(radius[i] < min){
			min = radius[i];
			index = i;
		}
		cout << radius[i] << endl;
	}

	cout << index;

	if(reverse == 1){
		start = wpcount-index+1;
		if(start < 0){
			start = wpcount-1;
		} else if(start >= wpcount){
			start = 0;
		}
	}else{
		start = index+1;
		if(start < 0){
			start = wpcount-1;
		} else if(start >= wpcount){
			start = 0;
		}
	}
/*
	points start_wp[4];

	// start 2 & counterclockwise orientation --> reverse
	start_wp[0].positionx = 23.7238750458;
	start_wp[0].positiony = 18.9728317261;
	start_wp[0].orientationz = 0.998501481217;
	start_wp[0].orientationw = 0.0547246928458;
	// start 2 & clockwise orientation
	start_wp[1].positionx = 23.7238750458;
	start_wp[1].positiony = 18.9728317261;
	start_wp[1].orientationz = -0.709141761734;
	start_wp[1].orientationw = 0.705065927247;
	// start 1 & counterclockwise orientation --> reverse
	start_wp[2].positionx = 11.3871631622;
	start_wp[2].positiony = 19.629447937;
	start_wp[2].orientationz = -0.733808296283;
	start_wp[2].orientationw = 0.679356595836;
	// start 1 & clockwise orientation;
	start_wp[3].positionx = 11.3871631622;
	start_wp[3].positiony = 19.629447937;
	start_wp[3].orientationz = -0.000768608907865;
	start_wp[3].orientationw = 0.99999970462;	

*/
	float euler[wpcount];

	// SET WAYPOINTS

	if(reverse == 0){ // clockwise route

	for(i = start; i<2*wpcount; i++){ // count from start to 2*wpcout

		j = i;

		if(j >= wpcount){ // if j > wpcount --> start again with 0.
			j = i-wpcount;
		}

		if(loopindex < wpcount){
			geometry_msgs::Pose waypoint1;
			waypoint1.position.x = wp[j].positionx;
			waypoint1.position.y = wp[j].positiony;
			waypoint1.position.z = 0.0;
			waypoint1.orientation.x = 0.0;
			waypoint1.orientation.y = 0.000;
			waypoint1.orientation.z = wp[j].orientationz;
			waypoint1.orientation.w = wp[j].orientationw;
			waypoints.push_back(waypoint1);

			loopindex++;
		}
	}

	} else { // counter-clockwise route

	for(i = start; i<2*wpcount; i++){

		j = wpcount-i;

		if(j < 0){
			j = 2*wpcount - i;
		}

		if(loopindex < wpcount){

			// for reverse route, use inverse of quaternion orientation.
			// --> orientationz = orientation[w];
			// --> orientationw = -orientation[z];

			geometry_msgs::Pose waypoint1;
			waypoint1.position.x = wp[j].positionx;
			waypoint1.position.y = wp[j].positiony;
			waypoint1.position.z = 0.0;
			waypoint1.orientation.x = 0;
			waypoint1.orientation.y = 0;
			waypoint1.orientation.z = wp[j].orientationw;
			waypoint1.orientation.w = -wp[j].orientationz;
			waypoints.push_back(waypoint1);

			loopindex++;
		}
	}

	}

/////////// end calculating waypoints



	


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
