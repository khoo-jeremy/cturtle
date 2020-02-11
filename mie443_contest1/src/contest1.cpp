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
            if (secondsElapsed == 180){
                strat = 1;
            }
            // ROS_INFO("seconds elapsed: %i", secondsElapsed);
            if (strat == 0){
                if(secondsElapsed % 45 == 0){
                    map_surroundings();
                }else{
                    turnAtWall();
                }
            } else {
                wallFollow();
            }

            // turn(-1, 30);

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
                turn(1, 30); //turn left
                left_turns++;
            }else{
                turn(-1, 30); //turn right
                right_turns++;
            }
        }else{
            moveForward();
            right_turns= 0;
            left_turns= 0;
        }

        //in a corner

        // ROS_INFO("Left turn #: %i   Right turn #: %i", left_turns, right_turns);
        if(right_turns > 0 && left_turns > 0){
            turn(1, 90);
            right_turns= 0;
            left_turns= 0;
        }
    }

    void moveForward(){
        vel.angular.z = 0.0;
        vel.linear.x = FORWARD_VEL;
        vel_pub.publish(vel);
    }

    void moveBackward(){
        vel.angular.z = 0.0;
        vel.linear.x = -FORWARD_VEL;
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

    void moveBackwardDist(float dist){
        float startX= posX;
        float startY= posY;
        float moved= 0;
        while(moved < dist){
            moveBackward();
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

    void bumperAdjust(){
        vel.linear.x = 0;
        vel.angular.z = 0;
        vel_pub.publish(vel);
        std::chrono::time_point<std::chrono::system_clock> back_start;
        back_start= std::chrono::system_clock::now();
        uint64_t run_time = 0;

        int bumper0= bumper[0];
        int bumper2= bumper[2];

        moveBackwardDist(0.1);

        if(bumper0 == kobuki_msgs::BumperEvent::PRESSED) // left bumper hit
        {
            turn(-1, 30); //turn right
        }
        if(bumper2 == kobuki_msgs::BumperEvent::PRESSED) // right bumper hit
        {
            turn(1, 30); //turn left
        }
        ROS_INFO("Bumper 0: %i  Bumper 2: %i", bumper0, bumper2);
    }

    void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
    {
        bumper[msg->bumper]= msg->state;
        if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
            bumperAdjust();
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

        // ROS_INFO(len(msg.ranges)); 
        regions_[0] = INF;
        regions_[1] = INF;
        regions_[2] = INF;
        for(int i = 1; i < 213; i++){
            if(msg->ranges[i] < regions_[0])
                regions_[0] = msg->ranges[i];
        }
        for(int i = 213; i < 426; i++){
            if(msg->ranges[i] < regions_[1])
                regions_[1] = msg->ranges[i];
        }
        for(int i = 426; i <= 638; i++){
            if(msg->ranges[i] < regions_[2])
                regions_[2] = msg->ranges[i];
        }

        // ROS_INFO("%i", (msg->ranges).size());

        // ROS_INFO("Right Region: %i. Front Region: %i. Left Region %i", regions[0], regions[1], regions[2]);
        take_action();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        posX = msg->pose.pose.position.x;
        posY = msg->pose.pose.position.y;
        yaw = tf::getYaw(msg->pose.pose.orientation);
        tf::getYaw(msg->pose.pose.orientation);
        // ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees", posX, posY, yaw, RAD2DEG(yaw));

        if (strat == 1)
        {
            strat_current_time= std::chrono::system_clock::now();
            int diff= std::chrono::duration_cast<std::chrono::seconds>(strat_current_time - strat_prev_time).count();
            if (diff >= 20 || strat_prev_time == 0){
                loop_points.push_back(std::pair<float, float>( posX, posY ));
                strat_prev_time = strat_current_time;
            }
            float start_x = wall_follow_start_pos[0];
            float start_y = wall_follow_start_pos[1];
            
            if (!robot_left_initial_area)
            {
                if (!((loop_points.back().first - 0.8 <= posX && posX <= loop_points.back().first + 0.8) &&
                        (loop_points.back().second - 0.8 <= posY && posY <= loop_points.back().second + 0.8)))
                    robot_left_initial_area = true;
            }
            else
            {
                for (std::pair<float, float> p : loop_points){
                    if ((p.first - 0.6 <= posX && posX <= p.first + 0.6) &&
                        (p.second - 0.6 <= posY && posY <= p.second + 0.6))
                    {
                        robot_left_initial_area = false;
                        loop_points.clear();
                        strat_prev_time = 0;
                        ROS_INFO("LOOPED AROUND THE WALL OBSTACLE");
                    }
                }
            }
        }
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
        vel.angular.z = 0.0;
        vel.linear.x = 0.0;
        vel_pub.publish(vel);
        // ROS_INFO("finish turn, start: %f, end: %f, turned: %f", RAD2DEG(start_angle), RAD2DEG(yaw), RAD2DEG(turned));
    }
        
    void change_state(int state){
        if (state != state_){
        // ROS_INFO("Wall follower - [%s] - %s", % (state, state_dict_[state]));
            state_ = state;
        }
    }

    void take_action(){
        regions[0] = regions_[0]; //right
        regions[1] = regions_[1]; //front
        regions[2] = regions_[2]; //left
        // geometry_msg::Twist msg;
        
        float d = 0.8;
        
        ROS_INFO("Right Region: %f. Front Region: %f. Left Region %f", regions[0], regions[1], regions[2]);

        if (regions[1] > d && regions[2] > d && regions[0] > d)
            change_state(0);
        else if ((regions[1] < d || regions[1] == INF) && regions[2] > d && regions[0] > d)
            change_state(1);
        else if (regions[1] > d && regions[2] > d && (regions[0] < d || regions[0] == INF))
            change_state(2);
        else if (regions[1] > d && (regions[2] < d || regions[2] == INF) && regions[0] > d)
            change_state(0);
        else if ((regions[1] < d || regions[1] == INF) && regions[2] > d && (regions[0] < d || regions[0] == INF))
            change_state(1);
        else if ((regions[1] < d || regions[1] == INF) && (regions[2] < d || regions[2] == INF) && regions[0] > d)
            change_state(1);
        else if ((regions[1] < d || regions[1] == INF) && (regions[2] < d || regions[2] == INF) && (regions[0] < d || regions[0] == INF))
            change_state(1);
        else if (regions[1] > d && (regions[2] < d || regions[2] == INF) && (regions[0] < d || regions[0] == INF))
            change_state(0);
        else
            ROS_INFO("Right Region: %f. Front Region: %f. Left Region %f", regions[1], regions[2], regions[3]);
    }
        
    geometry_msgs::Twist find_wall(){
        geometry_msgs::Twist msg;
        msg.linear.x = 0.2;
        msg.angular.z = -0.3;
        return msg;
    }
    
    geometry_msgs::Twist turn_left(){
        geometry_msgs::Twist msg;
        msg.angular.z = 0.3;
        return msg;
    }
    
    geometry_msgs::Twist follow_the_wall(){
        geometry_msgs::Twist msg;
        msg.linear.x = FORWARD_VEL;
        return msg;
    }

    void wallFollow(){
        geometry_msgs::Twist msg;
        if (state_ == 0){
            msg = find_wall();
            ROS_INFO("Finding wall");
        }
        else if (state_ == 1){
            msg = turn_left();
            ROS_INFO("Turn left");
        }
        else if (state_ == 2){
            msg = follow_the_wall();
            ROS_INFO("Follow the wall");
        }
        else{
            ROS_INFO("Unknown state!");
        }
        
        vel_pub.publish(msg);
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
    int right_turns= 0;
    int left_turns= 0;
    int strat = 0;
    std::vector<std::pair<float, float>> loop_points; 
    int32_t nLasers= 0, desiredNLasers= 0, desiredAngle= 10; // desiredAngle * 2 = field of view
    uint8_t bumper[3]= {kobuki_msgs::BumperEvent::RELEASED, 
                        kobuki_msgs::BumperEvent::RELEASED, 
                        kobuki_msgs::BumperEvent::RELEASED};

    float regions_[3]; // 0 = right side, 1 = front, 2 = left side
    float regions[3];
    int state_; // 0 = find wall, 1 = turn left, 2 = follow wall
    int state;
    
    std::chrono::time_point<std::chrono::system_clock> start;
    uint64_t secondsElapsed = 0;
    std::chrono::time_point<std::chrono::system_clock> turn_start;
    std::chrono::time_point<std::chrono::system_clock> strat_current_time;
    std::chrono::time_point<std::chrono::system_clock> strat_prev_time = 0;

    float wall_follow_start_pos[2] = {-1, -1};
    bool robot_left_initial_area = false;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "contest1_node");
    ROS_INFO("Contest 1 node is starting");
    ros::NodeHandle nh;
    ContestOne obj(nh);
    return 0;
}