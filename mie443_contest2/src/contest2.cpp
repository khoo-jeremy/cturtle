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
        id = ids[1];
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
    std::vector<std::vector<float>> coordinates;

    for(int i=0;i<5;i++)
    {
        float x1 = boxes.coords[i][0]+ 0.8 * cos(boxes.coords[i][2]);
        float y1 = boxes.coords[i][1]+ 0.8 * sin(boxes.coords[i][2]);
        float z1;
        if(boxes.coords[i][2] > 0)
            z1 = boxes.coords[i][2] - M_PI;
        else
            z1 = boxes.coords[i][2] + M_PI;
        coordinates.push_back({x1, y1, z1});
    }
    
    int counter= 0;
    // bool cont;
    while(ros::ok() and counter <= 4) {
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        Navigation::moveToGoal(coordinates[counter][0], coordinates[counter][1], coordinates[counter][2]);
        ros::Duration(2).sleep();

        ros::spinOnce();

        std::vector<int> ids;
        for(int i=0;i<3;i++)
            ids.push_back(imagePipeline.getTemplateID(boxes));
        int id = bestOfThree(ids);
            
        
        ros::Duration(0.01).sleep();

        if(id >= -1){
            ROS_INFO("%i", id);
            // cont= true;
        }else{
            ROS_INFO("Could not read image");
        }
        counter++;
    }
    return 0;
}
