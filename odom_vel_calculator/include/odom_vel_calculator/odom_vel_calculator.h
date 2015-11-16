#ifndef ODOM_VEL_CALCULATOR_H
#define ODOM_VEL_CALCULATOR_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <stdexcept>

#include <Eigen/Dense>
#include <math.h>

class Butter2Order
{
public:
    Butter2Order(uint inputcount, double A1, double A2, double A3, double B1, double B2, double B3);
    ~Butter2Order();

    // input the new value -> returns the filtered value
    const Eigen::VectorXd &filter(const Eigen::VectorXd &input);
	
    // reset to start with the next input value
    void reset();
private:
    uint _inputcount;

    bool _valid;

    double _A[3];
    double _B[3];

    Eigen::VectorXd _x_1; //previous input value
    Eigen::VectorXd _x_2; //previous previous input value
    Eigen::VectorXd _xf_1; //previous filtered value
    Eigen::VectorXd _xf_2; //previous previous filtered value

    Eigen::VectorXd _xf; //filtered value
};

class ODOMVELCALCULATOR
{

    public:
		/** Constructor */
		ODOMVELCALCULATOR();
		/** Destructor */
		~ODOMVELCALCULATOR();

	private:
		/** ROS node handle */
		ros::NodeHandle m_Node;

		/** cmd_vel subscriber */
		ros::Subscriber m_odom_sub;

		/** publisher for ackermann */
		ros::Publisher m_odom_pub;

		double rotZ, rotZ_old;
		Eigen::VectorXd vel, vel_filtered;
		Butter2Order* filter;

		nav_msgs::Odometry m_old_odometry;
		bool m_old_odometry_valid;

		void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_without_vel);
};
#endif
