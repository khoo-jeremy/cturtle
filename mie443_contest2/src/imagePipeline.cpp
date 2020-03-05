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
// #define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" 
#define IMAGE_TOPIC "camera/image"  // webcam:"camera/image"

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
        Mat img_bw;

        cv::cvtColor(img, img_bw, cv::COLOR_BGR2GRAY);

        int minHessian = 400;

        Ptr<SURF> detector = SURF::create(minHessian);

        std::vector<KeyPoint> keypoints_img, keypoints_box;
        Mat descriptors_img, descriptors_box;

        detector->detectAndCompute(boxes.templates[0], Mat(), keypoints_img, descriptors_img);
        detector->detectAndCompute(img_bw, Mat(), keypoints_box, descriptors_box);

        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        matcher.match(descriptors_img, descriptors_box, matches);
        double max_dist = 0;
        double min_dist = 100;

        for(int i = 0;i < descriptors_img.rows;i++){
            double dist = matches[i].distance;
            if(dist < min_dist){
                min_dist = dist;
            }
            if(dist > max_dist){
                max_dist = dist;
            }
        }

        std::vector<DMatch> good_matches;

        for(int i= 0; i < descriptors_img.rows;i++){
            if(matches[i].distance < 3 * min_dist){
                good_matches.push_back(matches[i]);
            }
        }

        //scene= boxes.templates[0], object=img

        Mat img_matches;
        drawMatches(img_bw, keypoints_img, boxes.templates[0], keypoints_box, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        std::vector<Point2f> obj;
        std::vector<Point2f> scene;

        for(int i= 0;i < good_matches.size();i++){
            //Get the keypoints from the good matches
            obj.push_back(keypoints_img[good_matches[i].queryIdx].pt);
            scene.push_back(keypoints_box[good_matches[i].trainIdx].pt);
        }

        Mat H= findHomography(obj, scene, RANSAC);

        //Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0]= cvPoint(0,0);
        obj_corners[1]= cvPoint(img_bw.cols, 0);
        obj_corners[2]= cvPoint(img_bw.cols, img_bw.rows);
        obj_corners[3]= cvPoint(0, img_bw.rows);
        std::vector<Point2f> scene_corners(4);

        perspectiveTransform(obj_corners, scene_corners, H);

        // Draw lines between the corners (the mapped object in the scene- image_2)
        line(img_matches, scene_corners[0] + Point2f(img_bw.cols, 0), scene_corners[1] + Point2f(img_bw.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[1] + Point2f(img_bw.cols, 0), scene_corners[2] + Point2f(img_bw.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[2] + Point2f(img_bw.cols, 0), scene_corners[3] + Point2f(img_bw.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[3] + Point2f(img_bw.cols, 0), scene_corners[0] + Point2f(img_bw.cols, 0), Scalar(0, 255, 0), 4);
        
        imshow("Good Matches & Object detection", img_matches);

        cv::imshow("view", img_bw);
        // cv::imshow("view", boxes.templates[0]);
        cv::waitKey(10);
    }  
    return template_id;
}
