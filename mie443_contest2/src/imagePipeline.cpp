#include <imagePipeline.h>
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;
using namespace cv::xfeatures2d;


#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" 
// #define IMAGE_TOPIC "camera/image"  // webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

bool isRectangle(std::vector<Point2f> corners){
    Point2f p1= corners[0];
    Point2f p2= corners[1];
    Point2f p3= corners[2];
    Point2f p4= corners[3];
    double cx, cy;
    double dd1, dd2, dd3, dd4;

    cx= (p1.x + p2.x + p3.x + p4.x)/4;
    cy= (p1.y + p2.y + p3.y + p4.y)/4;

    dd1= pow(cx - p1.x, 2) + pow(cy - p1.y, 2);
    dd2= pow(cx - p2.x, 2) + pow(cy - p2.y, 2);
    dd3= pow(cx - p3.x, 2) + pow(cy - p3.y, 2);
    dd4= pow(cx - p4.x, 2) + pow(cy - p4.y, 2);

    if(dd1 + dd3 > (dd2 + dd4) /1.5 && (dd1 + dd3) < (dd2 + dd4) * 1.5){
        // check if it detected the background cereal tag by determining the area of the parallelogram (area = cross product of width and length)
        Point2f length = cvPoint(p2.x - p1.x, p2.y - p1.y);
        Point2f width = cvPoint(p4.x - p1.x, p4.y - p1.y);
        float area = std::abs(length.x*width.y - width.x*length.y);
        // ROS_INFO("area: %f", area);
        if(area < 8000) return false;
        return true;
    }else {
        return false;
    }
    // if(dd1 + dd3 > (dd2 + dd4) /1.5 && (dd1 + dd3) < (dd2 + dd4) * 1.5){
    //     return true;
    // }else {
    //     return false;
    // }

    // std::cout << dd1 << "\t" << dd2 << "\t" << dd3 << "\t" << dd4 << std::endl;
}

bool matchImages(Mat img_scene, Mat img_object, int minHessian)
{
    Ptr<SURF> detector = SURF::create(minHessian);

    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;

    detector->detectAndCompute(img_object, Mat(), keypoints_object, descriptors_object);
    detector->detectAndCompute(img_scene, Mat(), keypoints_scene, descriptors_scene);

    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match(descriptors_object, descriptors_scene, matches);
    double max_dist = 0;
    double min_dist = 100;

    for(int i = 0;i < descriptors_object.rows;i++){
        double dist = matches[i].distance;
        if(dist < min_dist){
            min_dist = dist;
        }
        if(dist > max_dist){
            max_dist = dist;
        }
    }

    std::vector<DMatch> good_matches;

    for(int i= 0; i < descriptors_object.rows;i++){
        if(matches[i].distance < 3 * min_dist){
            good_matches.push_back(matches[i]);
        }
    }

    Mat img_matches;
    drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for(int i= 0;i < good_matches.size();i++){
        //Get the keypoints from the good matches
        obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }

    Mat H= findHomography(obj, scene, RANSAC);

    //Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0]= cvPoint(0,0);
    obj_corners[1]= cvPoint(img_object.cols, 0);
    obj_corners[2]= cvPoint(img_object.cols, img_object.rows);
    obj_corners[3]= cvPoint(0, img_object.rows);
    std::vector<Point2f> scene_corners(4);

    if(!H.empty())
    {
        perspectiveTransform(obj_corners, scene_corners, H);

        // Draw lines between the corners (the mapped object in the scene- image_2)
        line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0), scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0), scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0), scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0), scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);

        imshow("Good Matches & Object detection", img_matches);
        cv::waitKey(10);
        
        return isRectangle(scene_corners);
    }
    else
        return false;
    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/

        //convert to grayscale
        Mat img_scene;
        cv::cvtColor(img, img_scene, cv::COLOR_BGR2GRAY);

        int minHessian = 400;
        bool match=false;
        int count = 0;
        for(auto img_object: boxes.templates)
        {
            for(int i=0; i<6; i++)
            {
                match = matchImages(img_scene, img_object, minHessian);
                if(!match)
                    break;
            }

            if(match)
            {
                template_id = count;
                break;
            }
            count++;
        }
    }  
    return template_id;
}
