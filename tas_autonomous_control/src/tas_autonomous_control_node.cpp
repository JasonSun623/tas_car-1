#include "control/control.h"

#include <math.h>

#include <control_toolbox/pid.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

    ros::Rate loop_rate(50);


    control_toolbox::Pid pid_forward;
    control_toolbox::Pid pid_back;
    //control_toolbox::Pid pid_steering;

    ros::Time current_time, last_time;

    bool resetback = true;
    bool resetforward = true;
    
    while(ros::ok())
    {
        last_time = current_time;
        current_time = ros::Time::now();
        
        if(autonomous_control.control_Mode.data==0)
        {
            ROS_INFO("Manually Control!");
            
            // reset PID controllers
            resetback = true;
            resetforward = true;
        }
        else
        {
            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
            else
            {
                ROS_INFO("Automatic Control!");
                // threshold for standing still
                if(fabs(autonomous_control.cmd_linearVelocity) < 0.001)
                {
                    resetback = true;
                    resetforward = true;
                    autonomous_control.control_servo.x = 1500;
                }
                else if(autonomous_control.cmd_linearVelocity > 0.0)
                {
                    if(resetforward)
                    {
                        pid_forward.initPid(150, 30.0, 3.0, 100, -100);
                        resetforward = false;
                    }

                    autonomous_control.control_servo.x = 1500 + pid_forward.computeCommand(autonomous_control.cmd_linearVelocity - autonomous_control.odom_linearVelocity, current_time - last_time);
                    //std::cout << autonomous_control.cmd_linearVelocity << std::endl;
                    //std::cout << autonomous_control.odom_linearVelocity << std::endl;
                    //std::cout << autonomous_control.control_servo.x << std::endl;
            
                    resetback = true;
                }
                else
                {
                    if(resetback)
                    {
                        pid_back.initPid(300, 10.0, 1.0, 50, -50);
                        resetback = false;
                    }
                    
                    autonomous_control.control_servo.x = 1435 + pid_back.computeCommand(autonomous_control.cmd_linearVelocity - autonomous_control.odom_linearVelocity, current_time - last_time);
                    //std::cout << autonomous_control.cmd_linearVelocity << std::endl;
                    //std::cout << autonomous_control.odom_linearVelocity << std::endl;
                    //std::cout << autonomous_control.control_servo.x << std::endl;
                    resetforward = true;
                }
                //limit for backward velocity
                if(autonomous_control.control_servo.x < 1300)
                {
                    autonomous_control.control_servo.x = 1300;
                }
            
                //limit for forward velocity
                if(autonomous_control.control_servo.x > 1600)
                {
                    autonomous_control.control_servo.x = 1600;
                }
    


                //autonomous_control.control_servo.x = 1450+ 100*autonomous_control.cmd_linearVelocity;

                // pid control for steering angle - not used
                //autonomous_control.control_servo.y = 1500 + pid_steering.computeCommand(autonomous_control.cmd_steeringAngle - autonomous_control.odom_steeringAngle, current_time - last_time);
                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
                
                if(autonomous_control.control_servo.y < 1000)
                {
                    autonomous_control.control_servo.y = 1000;
                }

                if(autonomous_control.control_servo.y > 2000)
                {
                    autonomous_control.control_servo.y = 2000;
                }
                
                //std::cout << autonomous_control.cmd_steeringAngle << std::endl;
                //std::cout << autonomous_control.odom_steeringAngle << std::endl;
                //std::cout << autonomous_control.control_servo.y << std::endl;
            }
            
            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);

        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
