#include "control.h"

double signum(double x)
{
	if(x > 0)
		return 1.0;
	else if(x = 0)
		return 0.0;
	else
		return -1.0;
}

control::control()
{
    control_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &control::cmdCallback,this);

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom",10,&control::odomCallback,this);

    wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&control::wiiCommunicationCallback,this);

//    Fp = 10;// need to test! defult:125

//    current_ServoMsg.x = 1500;
//    current_ServoMsg.y = 1500;

//    previous_ServoMsg.x = 1500;
//    previous_ServoMsg.y = 1500;

}
// We can subscribe to the odom here and get some feedback signals so later we can build our controllers
void control::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	// convert Quaternion to Euler angles
    Eigen::Quaterniond quat;

    quat.x() = msg->pose.pose.orientation.x;
    quat.y() = msg->pose.pose.orientation.y;
    quat.z() = msg->pose.pose.orientation.z;
    quat.w() = msg->pose.pose.orientation.w;

    Eigen::Matrix3d R = quat.toRotationMatrix();

    Eigen::Vector3d RPY = R.eulerAngles(0,1,2);

    double x,y; 
	// linear velocity in x direction
    x = msg->twist.twist.linear.x;
	// linear velocity in y direction
    y = msg->twist.twist.linear.y;

	// absolute value of the velocity
    odom_linearVelocity = sqrt(x*x+y*y);

	// make sure that yaw angle is between -pi and pi
    while(RPY(2) - M_PI > 0 )
    {
	RPY(2) = RPY(2) - 2*M_PI;
    }
    while(RPY(2) + M_PI < 0 )
    {
	RPY(2) = RPY(2) + 2*M_PI;
    }

	// decide if the vehicle is driving forwards or backwards
    if((RPY(2) + M_PI_4 > 0) && (RPY(2) - M_PI_4 < 0))
    {
	odom_linearVelocity *= signum(x);    	
    }
    else if((RPY(2) - M_PI_4 > 0) && (RPY(2) - 3*M_PI_4 < 0))
    {
	odom_linearVelocity *= signum(y);    	
    }
    else if((RPY(2) + 3*M_PI_4 > 0) && (RPY(2) + M_PI_4 < 0))
    {
	odom_linearVelocity *= signum(-y);    	
    }
    else
    {
	odom_linearVelocity *= signum(-x);    	
    }

	// angular yaw velocity
    odom_angularVelocity = msg->twist.twist.angular.z;

    odom_steeringAngle = 180/PI*atan(odom_angularVelocity/cmd_linearVelocity*CAR_LENGTH);

    odom_steeringAngle = 1500 + 500/30*odom_steeringAngle;

    if(odom_steeringAngle > 2000)
    {
        odom_steeringAngle = 2000;
    }
    else if(odom_steeringAngle < 1000)
    {
        odom_steeringAngle = 1000;
    }
}

//Subscribe to the local planner and map the steering angle (and the velocity-but we dont do that here-) to pulse width modulation values.
void control::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_linearVelocity = msg->linear.x;
    cmd_angularVelocity = msg->angular.z;

    cmd_steeringAngle = 180/PI*atan(cmd_angularVelocity/cmd_linearVelocity*CAR_LENGTH);

    cmd_steeringAngle = 1500 + 500/30*cmd_steeringAngle;

    if(cmd_steeringAngle > 2000)
    {
        cmd_steeringAngle = 2000;
    }
    else if(cmd_steeringAngle < 1000)
    {
        cmd_steeringAngle = 1000;
    }
}
// a flag method that tells us if we are controlling the car manually or automatically
void control::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
    control_Brake.data = msg->data[1];
}

//geometry_msgs::Vector3 control::P_Controller()
//{
//    current_ServoMsg.x = previous_ServoMsg.x + Fp*(cmd_linearVelocity - odom_linearVelocity);

//    current_ServoMsg.y = cmd_steeringAngle;


//    if(current_ServoMsg.x > 1580)
//    {
//        current_ServoMsg.x = 1580;
//    }
//    else if(current_ServoMsg.x < 1300)
//    {
//        current_ServoMsg.x = 1300;
//    }

//    if(current_ServoMsg.y > 2000)
//    {
//        current_ServoMsg.y = 2000;
//    }
//    else if(current_ServoMsg.y < 1000)
//    {
//        current_ServoMsg.y = 1000;
//    }

//    previous_ServoMsg = current_ServoMsg;

//    return current_ServoMsg;
//}
