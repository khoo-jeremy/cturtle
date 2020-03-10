#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <math.h>

int bestOfThree(std::vector<int> ids)
{
    int id;
    if(ids[0]==ids[1] || ids[0]==ids[2])
        id = ids[0];
    else if(ids[1]==ids[2])
        id = ids[1]
    else
        id = -2;
    
    return id;
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.

    float x1 = boxes.coords[0][0]+cos(boxes.coords[0][2]);
    float y1 = boxes.coords[0][1]+sin(boxes.coords[0][2]);
    float z1;
    if(boxes.coords[0][2] > 0)
        z1 = boxes.coords[0][2] - M_PI;
    else
        z1 = boxes.coords[0][2] + M_PI;

    float x2 = boxes.coords[1][0]+cos(boxes.coords[1][2]);
    float y2 = boxes.coords[1][1]+sin(boxes.coords[1][2]);
    float z2;
    if(boxes.coords[1][2] > 0)
        z2 = boxes.coords[1][2] - M_PI;
    else
        z2 = boxes.coords[1][2] + M_PI;

    float x3 = boxes.coords[2][0]+cos(boxes.coords[2][2]);
    float y3 = boxes.coords[2][1]+sin(boxes.coords[2][2]);
    float z3;
    if(boxes.coords[2][2] > 0)
        z3 = boxes.coords[2][2] - M_PI;
    else
        z3 = boxes.coords[2][2] + M_PI;

    float x4 = boxes.coords[3][0]+cos(boxes.coords[3][2]);
    float y4 = boxes.coords[3][1]+sin(boxes.coords[3][2]);
    float z4;
    if(boxes.coords[3][2] > 0)
        z4 = boxes.coords[3][2] - M_PI;
    else
        z4 = boxes.coords[3][2] + M_PI;

    float x5 = boxes.coords[4][0]+cos(boxes.coords[4][2]);
    float y5 = boxes.coords[4][1]+sin(boxes.coords[4][2]);
    float z5;
    if(boxes.coords[4][2] > 0)
        z5 = boxes.coords[4][2] - M_PI;
    else
        z5 = boxes.coords[4][2] + M_PI;

    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        Navigation::moveToGoal(x1, y1, z1);
        ros::Duration(2).sleep();

        Navigation::moveToGoal(x2, y2, z2);
        ros::Duration(2).sleep();

        Navigation::moveToGoal(x3, y3, z3);
        ros::Duration(2).sleep();

        Navigation::moveToGoal(x4, y4, z4);
        ros::Duration(2).sleep();

        Navigation::moveToGoal(x5, y5, z5);
        ros::Duration(2).sleep();

        std::vector<int> ids;
        for(int i=0;i<3;i++)
            ids.push_back(imagePipeline.getTemplateID(boxes));
        int id = bestOfThree(ids);
            
        ROS_INFO("%i", id);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
