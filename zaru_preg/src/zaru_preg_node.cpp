/* Mainly copied from/inspired by odometry_publisher.cpp from fub_modelcar... */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define DEBUG
#define RATE 50.0
#define YAW_OFFSET -8.0
#define KP 20
#define KD 5
#define DISTANCE 4.0
#define DRIVE_CIRCLE 1
#define SET_SPEED -400

double x0 = 0.0;
double y0 = 0.0;
double dist = 0.0; // distance along direction
double th = 0.0;

double head = 0.0;
double last_head = 0.0;

double vx = 0.0; // velocity along direction
//double vy = 0.0; // allways zero
double vth = 0.0;

bool init=false;
double desired_yaw = 0.0;
double current_yaw = 0.0;
double yaw_difference = 0.0;
double xycurrent = 0.0;
double distance = DISTANCE;
double xy0 = 0.0;
int cycle = 0;
std_msgs::Int16 speed;
std_msgs::Int16 steering;


ros::Time current_time_twist, last_time_twist;

void twistCallback(const geometry_msgs::Twist& msg) {
  vx = round(round(msg.linear.x0 / (57.29578))*(-31.0))/100; // Result: **.**
}

void headingCallback(const std_msgs::Float32& msg) {
  current_yaw = msg.data + 180.0; // 0...360
  if (init == false) {
    init = true;
    desired_yaw = current_yaw;
    head = msg.data * (M_PI / 180.0); //rad -Pi...Pi
    last_head = head;
    th = head;
    vth = 0.0;
    current_time_twist = ros::Time::now();
    last_time_twist = current_time_twist;
  } else {
    last_time_twist = current_time_twist;
    current_time_twist = ros::Time::now();
    last_head = head;
    head = msg.data * (M_PI / 180.0); //rad
    double dt_twist = (current_time_twist - last_time_twist).toSec();
    double delta_head = head - last_head;
    if (delta_head > M_PI)
      delta_head = delta_head - (2.0 * M_PI);
    else if (delta_head < -M_PI)
      delta_head = delta_head + (2.0 * M_PI);
    vth = roundf(delta_head * 100000) / 1000;  /* Result: 37.78 */
    th = roundf(head * 1000) / 1000;
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "zaru_preg_node");
  ros::NodeHandle n, sn, nh, oh, dh, sh;
  ros::Subscriber twist_sub = n.subscribe( "motor_control/twist", 10, twistCallback);
  ros::Subscriber theta_sub = n.subscribe( "model_car/yaw", 10, headingCallback);//degree
  ros::Time current_time, last_time;

  current_time = ros::Time::now();

  ros::Publisher pubSteering_ = sh.advertise<std_msgs::Int16>(sh.resolveName("manual_control/steering"), 10, false);
  ros::Publisher pubSpeed_ = nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 10, true);
  speed.data = SET_SPEED;
  current_yaw = 0.0;
  //int count = 0;
  
  steering.data = 90 + YAW_OFFSET;
  double kp = KP;
  double kd = KD;
  if(argc > 1) steering.data = atoi(argv[1]); 
  if(argc > 2) kp = double(atoi( argv[2]));
  if(argc > 3) kd = double(atoi( argv[3]));
  if(argc > 4) distance = double(atoi( argv[3]));
  if(argc > 5) { ROS_INFO("To many arguments! - usage:%s |<steering value>|<Kp>|<Kd>|<Distance>||||",argv[0]); return 1; }
  
  xy0 = dist;
  xycurrent = xy0;

  ros::Rate wait_r(100);
  while(pubSpeed_.getNumSubscribers()==0) {
    ros::spinOnce();
    wait_r.sleep();
  }

  pubSpeed_.publish(speed); // set speed only once

  ros::Rate r(RATE); // Rate: loops per second!
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    last_time = current_time;
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    dt = roundf(dt * 10000) / 10000;
    double delta_x0 = (vx * cos(th - (desired_yaw / (M_PI / 180.0))) /*- vy * sin(th)*/) * dt; // /**/ because vy is allways zero
    double delta_y0 = (vx * sin(th - (desired_yaw / (M_PI / 180.0))) /*+ vy * cos(th)*/) * dt; // /**/ because vy is allways zero
    double delta_dist = vx * dt;
    double delta_th = vth * 0.01 ; //* dt;
    x0 += delta_x0;
    y0 += delta_y0;
    vx0 = delta_x0 / dt;
    vy0 = delta_y0 / dt;
    dist += delta_dist;
    //th += delta_th;
    
    xycurrent = dist;

    #ifdef DEBUG
    //ROS_INFO_THROTTLE(0.1,"t %0.6f - dist %0.6f - heading %0.6f - vx %0.6f",dt,dist,head,vx);
    #endif
    
    //pubSpeed_.publish(speed);
    pubSteering_.publish(steering);

    #ifdef DEBUG
    ROS_INFO_THROTTLE(0.1,"t,%0.6f,dist,%0.6f,head,%0.6f,vx,%0.6f,mdist,%f,spd,%i,kp,%f,kd,%f,yawdif,%f,steer,%i,desYaw,%f,curYaw,%f,x0,%f,y0,%f",dt,dist,head,vx,xycurrent - xy0,speed.data,kp,kd,yaw_difference,steering.data,desired_yaw,current_yaw,x0,y0);
    //ROS_INFO_THROTTLE(0.1,"Distance: [%f] Speed : [%i]", xycurrent - xy0, speed.data);
    //ROS_INFO_THROTTLE(0.1,"KP [%f] Yaw Difference: [%f] Steering [%i] , Desried Yaw [%f] current yaw [%f]",
    //				  kp, yaw_difference,steering.data,desired_yaw, current_yaw);
    //ROS_INFO_THROTTLE(0.1,"X : [%f] Y: [%f]",x0,y0);	
    #endif

    yaw_difference = current_yaw - desired_yaw; 	
    // Find yaw difference
    if(yaw_difference > 180.0) yaw_difference = yaw_difference -360.0;
    else if (yaw_difference < -180.0) yaw_difference = yaw_difference +360.0;

    steering.data = int(kp * x0) + int(kd * vx0);
    //steering.data = int(kp * (yaw_difference)) + int(kd * vth) + 90.0 + YAW_OFFSET;
    // For the steering not to exceed the allowed values.
    if(steering.data > 180.0) steering.data = 180.0;
    if(steering.data < 0.0)	steering.data = 0.0;

#ifdef DEBUG
    //ROS_INFO_THROTTLE(0.1,"KP [%f] Yaw Difference: [%f] Steering [%i] , Desried Yaw [%f] current yaw [%f]",
    				  //kp, yaw_difference,steering.data,desired_yaw, current_yaw);
    //ROS_INFO_THROTTLE(0.1,"X : [%f] Y: [%f]",x0,y0);	
#endif
	
    // If DRIVE_CIRCLE is enabled, change direction by 90 degrees each DISTANCE meters
    if (DRIVE_CIRCLE) {
      if(abs(xycurrent - xy0) > distance) {
	    // change driving direction of the car
	    desired_yaw -= 90.0;
	    if(desired_yaw >= 360.0) desired_yaw -= 360.0;
	    if(desired_yaw <= 0.0) desired_yaw += 360.0;

	    // reset the traveled distance
	    xy0 = dist;
	
	    cycle++;
	    if (cycle == 5) {
	      speed.data = 0; // stop the car after 5 turns...
          pubSpeed_.publish(speed);
          wait_r.sleep(); // be sure that the published message received by subscriber
	      break; // terminate node after last cycle
	    }
      }
    }
    r.sleep();
  } // end of while loop
  
  return 0;
}



//=======================================================
// %EndTag(FULLTEXT)%



