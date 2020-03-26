#include <boxes.h>
#include <navigation.h>
#include <nn.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <math.h>
#include <fstream>

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
        float x1 = boxes.coords[i][0]+ 0.6 * cos(boxes.coords[i][2]);
        float y1 = boxes.coords[i][1]+ 0.6 * sin(boxes.coords[i][2]);
        float z1;
        if(boxes.coords[i][2] > 0)
            z1 = boxes.coords[i][2] - M_PI;
        else
            z1 = boxes.coords[i][2] + M_PI;
        coordinates.push_back({x1, y1, z1});
    }

    while(!robotPose.flag)
    {
        ros::spinOnce();
        ROS_INFO("Waiting for robotPose to update");
    }

    std::vector<float> origin;
    if (ros::ok()){
        ros::Duration(2).sleep();
        ros::spinOnce();
        origin = {robotPose.x, robotPose.y, robotPose.phi};
        ROS_INFO("origin, (%f, %f, %f)", robotPose.x, robotPose.y, robotPose.phi);

        coordinates = nn(coordinates, origin);
        for (int i = 0; i < coordinates.size(); i++)
            ROS_INFO("destination %i, (%f, %f, %f)", i, coordinates[i][0], coordinates[i][1], coordinates[i][2]);
    }



    // Create a file
    std::ofstream myfile;
    myfile.open("contest2.txt");
    
    int counter= 0;
    // bool cont;
    while(ros::ok() && counter < coordinates.size()) {
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        float x = coordinates[counter][0];
        float y = coordinates[counter][1];
        float z = coordinates[counter][2];
        Navigation::moveToGoal(x, y, z);
        ros::Duration(2).sleep();

        ros::spinOnce();

        if(counter < coordinates.size() - 1)
        {
            std::vector<int> ids;
            for(int i=0;i<3;i++)
                ids.push_back(imagePipeline.getTemplateID(boxes));
            int id = bestOfThree(ids);

            ros::Duration(0.01).sleep();

            if(id >= -1){
                ROS_INFO("Template id: %i, @ Coordinates x : %f, y : %f", id, x, y);
                myfile << "Template id: " << id << ". X-pos: " << x << ". Y-pos: " << y << "\n";
                // cont= true;
            }else{
                ROS_INFO("Could not read image");
            }
        }
        counter++;
    }
    myfile.close();
    return 0;
}
