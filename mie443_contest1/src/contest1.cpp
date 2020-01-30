#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

// #define N_BUMPER(3)
#define RAD2DEG(rad) ((rad) * 180 / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180)
#define INF std::numeric_limits<float>::infinity()

float angular = 0.0;
float linear = 0.0;

float posX= 0.0, posY = 0.0, yaw = 0.0;
float minLaserDist = INF;
int minLaserIdx= 0;
int32_t nLasers= 0, desiredNLasers= 0, desiredAngle= 5; //smallest angle lasers can detect

const float ANGULAR_VEL= 0.5; //positive = turn left

uint8_t bumper[3]= {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper]= msg->state;

}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	nLasers= (msg->angle_max-msg->angle_min)/msg->angle_increment;
    desiredNLasers= DEG2RAD(desiredAngle)/msg->angle_increment; 
    // ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    minLaserDist = INF;
    
    if(desiredAngle * M_PI/180 < msg->angle_max && -desiredAngle * M_PI/180>msg->angle_min)
    {
        for(uint32_t laser_idx= nLasers/2-desiredNLasers; laser_idx < nLasers/2 + desiredNLasers; ++laser_idx)
        {
            if(minLaserDist > msg->ranges[laser_idx]){
                minLaserDist= msg->ranges[laser_idx];
                minLaserIdx= laser_idx;
            }
            ROS_INFO("minLaserIdx: %f", minLaserIdx);

        }
    }else
    {
        for(uint32_t laser_idx= 0; laser_idx < nLasers; ++laser_idx)
        {
            if(minLaserDist > msg->ranges[laser_idx]){
                minLaserDist= msg->ranges[laser_idx];
                minLaserIdx= laser_idx;
            }
            ROS_INFO("minLaserIdx: %f", minLaserIdx);
        }
    }
    ROS_INFO("Minimum distance from object: %f", minLaserDist);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
    // ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees", posX, posY, yaw, RAD2DEG(yaw));
}

float time_to_turn(float angle_rad){
    return angle_rad/ANGULAR_VEL;
}

void decelerate(geometry_msgs::Twist vel, ros::Publisher vel_pub)
{
    vel.angular.z = 0.0;
    for(int i=0.2; i>=0; i=i-0.05)
    {
        vel.linear.x = i;
        vel_pub.publish(vel);
    }
}
void turn(int dir, int angle_deg, geometry_msgs::Twist vel, ros::Publisher vel_pub){
    std::chrono::time_point<std::chrono::system_clock> turn_start;
    turn_start= std::chrono::system_clock::now();
    uint64_t run_time= 0;

    float angle_rad= DEG2RAD(angle_deg);
    float turn_vel= ANGULAR_VEL; 
    if(dir == 0){
        turn_vel = ANGULAR_VEL * -1;
    }

    while(run_time <= time_to_turn(angle_rad)){
        vel.angular.z = turn_vel; 
        vel.linear.x = 0.0;
        vel_pub.publish(vel);

        run_time= std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-turn_start).count();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom= nh.subscribe("odom",1,&odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // float angular = 0.0;
    // float linear = 0.1;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        // std::cout<<minLaserDist<<std::endl;
        

        if(minLaserDist < 0.6 || minLaserDist == INF){
            decelerate(vel, vel_pub);
            if(minLaserIdx < desiredNLasers/2){ 
                turn(0, 45, vel, vel_pub); //turn right
            }else{
                turn(1, 45, vel, vel_pub); //turn left
            }
        }else{
            vel.angular.z = 0.0;
            vel.linear.x = 0.25;
            vel_pub.publish(vel);
        }

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
