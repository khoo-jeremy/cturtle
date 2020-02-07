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

class ContestOne {
public:
    ContestOne(ros::NodeHandle& nh) : m_nh(nh)
    {
        bumper_sub = m_nh.subscribe<kobuki_msgs::BumperEvent>("/mobile_base/events/bumper", 10, &ContestOne::bumperCallback, this);
        laser_sub = m_nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &ContestOne::laserCallback, this);
        odom_sub = m_nh.subscribe<nav_msgs::Odometry>("/odom",1,&ContestOne::odomCallback, this);
        vel_pub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

        ros::Rate loop_rate(10);
        start = std::chrono::system_clock::now();

        while(ros::ok() && secondsElapsed <= 480) {
            ros::spinOnce();
    
            ROS_INFO("seconds elapsed: %i", secondsElapsed);
            if(secondsElapsed % 45 == 0){
                map_surroundings();
            }else{
                turnAtWall();
            }

            // The last thing to do is to update the timer.
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
            loop_rate.sleep();
        }
    }

    void map_surroundings(){
        ROS_INFO("Mapping");
        vel.angular.z = 0.0;
        vel.linear.x = 0.0;
        vel_pub.publish(vel);
        turn(1, 360);
    }

    void turnAtWall(){

        if(minLaserDist < 0.6 || minLaserDist == INF){
            // ROS_INFO("minLaserIdx: %i", minLaserIdx);
            // ROS_INFO("desiredNLasers: %i", desiredNLasers);
            if(minLaserIdx < nLasers/2){ 
                turn(1, 45); //turn left
                turn_left++;
            }else{
                turn(-1, 45); //turn right
                turn_right++;
            }
        }else{
            moveForward();
            turn_right= 0;
            turn_left= 0;
        }

        //in a corner
        // ROS_INFO("Left turn #: %i   Right turn #: %i", turn_left, turn_right);
        if(turn_right > 0 && turn_left > 0){
            turn(1, 180);
            turn_right= 0;
            turn_left= 0;
        }
    }

    void moveForward(){
        vel.angular.z = 0.0;
        vel.linear.x = FORWARD_VEL;
        vel_pub.publish(vel);
    }

    void moveForwardDist(float dist){
        float startX= posX;
        float startY= posY;
        float moved= 0;
        while(moved < dist){
            moveForward();
            ros::spinOnce();
            moved= calculateDist(startX, startY, posX, posY);
            // ROS_INFO("Moved distance: %f", moved);
        }

    }    

    float calculateDist(float posX_s, float posY_s, float posX_f, float posY_f){
        float distX, distY;
        distX= posX_f - posX_s;
        distY= posY_f - posY_s;
        return sqrt(distX * distX + distY * distY);
    }

    // void checkBumper(){
    //     if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    //     {
    //         vel.linear.x = 0;
    //         vel.angular.z = 0;
    //         vel_pub.publsh(vel);
    //         std::chrono::time_point<std::chrono::system_clock> back_start;
    //         back_start= std::chrono::system_clock::now();
    //         uint64_t run_time = 0;
            
    //         while (run_time < 2)
    //         {
    //             vel.linear.x = -0.25;
    //             vel_pub.publsh(vel);
    //             run_time= td::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()- back_start).count();
    //         }

    //         if(bumper[0] == 1)
    //         {
    //             turn(0, 45, vel, vel_pub); //turn right
    //         }
    //         if(bumper[2] = 1)
    //         {
    //             turn(1, 45, vel, vel_pub); //turn left
    //         }
    //     }
    // }

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
        
        right_most_dist= msg->ranges[0];
        left_most_dist= msg->ranges[nLasers];

        if(desiredAngle * M_PI/180 < msg->angle_max && -desiredAngle * M_PI/180>msg->angle_min)
        {
            for(uint32_t laser_idx= nLasers/2-desiredNLasers; laser_idx < nLasers/2 + desiredNLasers; ++laser_idx)
            {
                if(minLaserDist > msg->ranges[laser_idx]){
                    minLaserDist= msg->ranges[laser_idx];
                    minLaserIdx= laser_idx;
                }

            }
        }else
        {
            for(uint32_t laser_idx= 0; laser_idx < nLasers; ++laser_idx)
            {
                if(minLaserDist > msg->ranges[laser_idx]){
                    minLaserDist= msg->ranges[laser_idx];
                    minLaserIdx= laser_idx;
                }
            }
        }
        // ROS_INFO("Minimum distance from object: %f", minLaserDist);
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

    // 1= left, -1= right
    void turn(int dir, int angle_deg){
        turn_start= std::chrono::system_clock::now();
        uint64_t run_time= 0;
        float angle_rad= DEG2RAD(angle_deg);
        float turn_vel = ANGULAR_VEL * dir;
        float start_angle = yaw;
        float last_position = start_angle;
        float turned = 0.0;

        // vel.linear.x = 0.0;
        // vel_pub.publish(vel);
        // ros::Duration(0.5).sleep();

        // ROS_INFO("dir: %i, start: %f, planned_turn_amount: %f", dir, RAD2DEG(start_angle), RAD2DEG(angle_rad)*dir);
        while(turned < angle_rad){
            ros::spinOnce();
            // ROS_INFO("yaw: %f, dir: %i", yaw, dir);
            // ros::Duration(0.5).sleep();
            vel.angular.z = turn_vel;
            vel.linear.x = 0.0;
            vel_pub.publish(vel);
            if ((yaw - last_position + dir)*dir >= 0){
                turned += std::abs(yaw - last_position);
            }else{
                turned += std::abs(2*M_PI*dir + yaw - last_position);
            }
            last_position = yaw;
            run_time= std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-turn_start).count();
        }
        // ROS_INFO("finish turn, start: %f, end: %f, turned: %f", RAD2DEG(start_angle), RAD2DEG(yaw), RAD2DEG(turned));
    }

private:
    ros::NodeHandle m_nh;
    ros::Publisher vel_pub;
    ros::Subscriber bumper_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    geometry_msgs::Twist vel;

    // const float ANGULAR_VEL= M_PI/6;
    const float ANGULAR_VEL= M_PI/2;
    const float FORWARD_VEL= 0.25;
    float angular = 0.0;
    float linear = 0.0;
    float posX= 0.0, posY = 0.0, yaw = 0.0;
    float minLaserDist = INF;
    int minLaserIdx= 0;
    int turn_right= 0;
    int turn_left= 0;
    int32_t nLasers= 0, desiredNLasers= 0, desiredAngle= 10; // desiredAngle * 2 = field of view
    uint8_t bumper[3]= {kobuki_msgs::BumperEvent::RELEASED, 
                        kobuki_msgs::BumperEvent::RELEASED, 
                        kobuki_msgs::BumperEvent::RELEASED};

    std::chrono::time_point<std::chrono::system_clock> start;
    uint64_t secondsElapsed = 0;
    std::chrono::time_point<std::chrono::system_clock> turn_start;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image_listener");
    ROS_INFO("Contest 1 node is starting");
    ros::NodeHandle nh;
    ContestOne obj(nh);
    return 0;
}