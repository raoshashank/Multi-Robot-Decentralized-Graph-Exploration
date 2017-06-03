/**
 * @file website_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#define pi 3.14159265359
int bot_id =2;
bool fl_T_init=false;
geometry_msgs::Twist Vel;
nav_msgs::Odometry feedback;


void Odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    feedback = *msg;
    //printf("%lf\n",feedback.pose.pose.position.x);
}


// if (joyvar.buttons[3] == 1 && fl_manual==true)
// 	{fl_manual=false;
// 	fl_auto=true;
// 	fl_land=false;	
// ROS_INFO("Manual OFF");}

// if (joyvar.buttons[1] == 1 && fl_manual==false)
// 	{fl_manual=true;
// 		fl_auto=false;
// 	fl_land=false;
// 	ROS_INFO("Manual ON");}
// if (joyvar.buttons[2] == 1 )
// 	{fl_manual=false;
// 		fl_auto=false;
// 	fl_land=true;
// 	ROS_INFO("Landing Commanded");}
// }





int main(int argc, char **argv)
{
    ros::init(argc, argv, "syscon_bot_0_ctrl");
    ros::NodeHandle nh;

    ros::Subscriber Odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("syscon_bot_0/odom", 10, Odom_cb);
    ros::Publisher Vel_pub = nh.advertise<geometry_msgs::Twist>
            ("syscon_bot_0/cmd_vel", 10);
   
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

 

   
    float q[4];
    float ang;
    int x_d = 0, y_d = 0, z_d = 2;
	 float v_xi, v_yi;
	 float V_max=0.5;
	 float s_dot, s=0;
	 double to,t,T;
	 bool fl_T_init=false;
	 float x_l, y_l,x_bot,y_bot,D_lb, theta, heading_cmd, heading, heading_err ;
	 float A=5,B=5, a=3, b=2;

    ros::Time last_request = ros::Time::now();

    s_dot=V_max/sqrt(A*A*a*a+B*B*b*b);
to=ros::Time::now().toSec();
   
while(ros::ok() && to<2 ){to=ros::Time::now().toSec();}
    while(ros::ok()){

       



  


	  if (fl_T_init==false)
	  	{to=ros::Time::now().toSec();
	  	fl_T_init=true;
        printf("init time = %f",to);}

	 	t=ros::Time::now().toSec();
		T=t-to;
	// 	//ROS_INFO("%lf",t-to);

	 	s=s_dot*T;

		x_l=A*cos(2*pi*(bot_id-1)/(a+b)-a*s);
	 	y_l=B*sin(2*pi*(bot_id-1)/(a+b)+b*s);

	   q[0] = feedback.pose.pose.orientation.w;
	   q[1] = feedback.pose.pose.orientation.x;
	   q[2] = feedback.pose.pose.orientation.y;
	   q[3] = feedback.pose.pose.orientation.z;
       heading = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));

 // 		 pose.pose.position.x = x_cmd;
 //    	pose.pose.position.y = y_cmd;
 //    	pose.pose.position.z = 1;

	// 	local_pos_pub.publish(pose);
	 		x_bot=feedback.pose.pose.position.x;
	 		y_bot=feedback.pose.pose.position.y;
            heading_cmd=atan2(y_l-y_bot,x_l-x_bot);
            D_lb=sqrt((x_bot-x_l)*(x_bot-x_l)+(y_bot-y_l)*(y_bot-y_l));
     //printf(" to=%f, t=%f , T=%f \n",to,t,T );

     printf("t = %f, x_l =%f, y_l=%f, x_bot =%f, y_bot=%f , Distance = %f\n",T,x_l,y_l,x_bot,y_bot,D_lb );
		heading_err=heading_cmd-heading;
        if (heading_err>pi)
            heading_err=heading_err-2*pi;
        if (heading_err<=-pi)
            heading_err=heading_err+2*pi;
        
         Vel.linear.y = 0;
         Vel.linear.x = 1* D_lb*cos(heading_err);
         Vel.linear.z = 0;

         Vel.angular.z=-0.8*heading_err;
	//}

  	Vel_pub.publish(Vel);
   





  ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
