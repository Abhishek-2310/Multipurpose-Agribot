#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

using namespace std;
using namespace cv;

float error_angle = 0.0;
float error_dist = 0.0;

void DetectPlant(cv::Mat &image)
{
    // convert to HSV color space
    cv::Mat hsvImage;
    cv::cvtColor(image, hsvImage, CV_BGR2HSV);
    
    // split the channels
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsvImage, hsvChannels);

    // is the color within the lower hue range?
    cv::Mat hueMask;
    cv::Mat hueImg = hsvChannels[0];
    cv::inRange(hueImg, 50, 80, hueMask);
    cv::Mat saturationMask;
    cv::Mat saturationImg = hsvChannels[1];
    cv::inRange(saturationImg, 0, 255, saturationMask);
    cv::Mat valueMask;
    cv::Mat valueImg = hsvChannels[2];
    cv::inRange(valueImg, 0, 255, valueMask);

    hueMask = hueMask & saturationMask & valueMask;
    
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    // finds cluster/contour in the image
    cv::findContours(hueMask, contours, hierarchy, CV_RETR_TREE,
                CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // draw contours
    cv::Mat img_contour = Mat::zeros(hueMask.size(), CV_8UC3);
    cv::Scalar color = cv::Scalar(0, 255, 0);
    for(size_t i = 0; i < contours.size(); i++)
    {
        cv::drawContours(img_contour, contours, i, color, 1, 8, hierarchy, 0, Point());
    }

    if(contours.size() > 0)
    {
        // approximate contours to polygons + get bounding rects and circles
        vector<vector<Point>> contours_poly(contours.size());
        vector<Point2f> center(contours.size());
        vector<Point2f> filtered_center;
        vector<float> radius(contours.size());

        // find enclosing Polygon which fits arround the contures and get their centers 
        for (size_t i = 0; i < contours.size(); i++) 
        {
            approxPolyDP(Mat(contours[i]), contours_poly[i], 2, true);
            minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);

            if(radius[i] > 20.0)
            {
                //draw the circle enclosing the contours
                cv::circle(img_contour, Point(center[i].x, center[i].y), radius[i], Scalar(0, 0, 255), 2, 8, 0);
                filtered_center.push_back(center[i]);
            }
        }
            
        if(filtered_center.size() > 0)
        {
            // draw desired path line(center of the camera frame)
            cv::line(image, Point(image.cols / 2, 0), Point(image.cols / 2, image.rows), cv::Scalar(255, 0, 255), 2, cv::LINE_AA);

            // line fitting
            Vec4f linefit;
            cv::fitLine(filtered_center, linefit, CV_DIST_L2, 0, 0.01, 0.01);
            
            Point2f pt1;
            pt1.x = linefit[2] + linefit[0] * 1000;
            pt1.y = linefit[3] + linefit[1] * 1000;
            
            Point2f pt2;
            pt2.x = linefit[2] - linefit[0] * 1000;
            pt2.y = linefit[3] - linefit[1] * 1000;

            // draw fitted line
            cv::line(image, pt1, pt2, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

            cv::imshow("opencv_window", image);
            cv::waitKey(3);

            //compute angle error;
            float angle = CV_PI / 2;
            if(abs(pt1.x - pt2.x) > 0.0f)
            {
                float slope = (pt2.y - pt1.y) / (pt2.x - pt1.x); 
                angle = atan(slope);
            }

            error_angle = (CV_PI / 2) - abs(angle);

            if(angle < 0.0f)
            {
                error_angle *= -1; 
            }

            // compute lateral distance error (ditance between mid points of the desired and detected line)
            float lateral_dist = 0.0;
            if(abs(angle) < CV_PI/2)
            {
                float slope = tan(angle);
                float y_intercept = pt1.y - (slope * pt1.x);
                float x_coord = ((image.rows / 2) - y_intercept) / slope;
                lateral_dist = x_coord - (image.cols / 2);
            }
            else
            {
                lateral_dist = pt1.x - (image.cols / 2);
            }
            
            error_dist = lateral_dist;

            //ROS_INFO("err_dist: %0.3f, err_angle: %0.3f deg", error_dist, error_angle * 180 / CV_PI);
        }
    }
    
    return;
}

void ImageCB(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        DetectPlant(cv_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");

    ros::NodeHandle nh;
    ros::Subscriber image_subscriber = nh.subscribe("/agribot/front_camera/image_raw", 1, ImageCB);

    ros::Publisher cmdvel_pub = nh.advertise<geometry_msgs::Twist>("/agribot/mobile_base_controller/cmd_vel", 1);
    ros::Publisher error_dist_pub = nh.advertise<std_msgs::Float32>("/error_dist", 1);
    ros::Publisher error_angle_pub = nh.advertise<std_msgs::Float32>("/error_angle", 1);

    ros::Rate loopRate(100);

    cv::namedWindow("opencv_window");

    while(ros::ok())
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.4;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = (error_angle * 30.0f) + (-error_dist * 0.2);

        // limiting angular velocity to 20 rad/s
        if(abs(cmd_vel.angular.z) > 20.0f)
        {
            cmd_vel.angular.z = 0.0;
        }

        cmdvel_pub.publish(cmd_vel);

        std_msgs::Float32 err_dist;
        err_dist.data = error_dist;
        error_dist_pub.publish(err_dist);

        std_msgs::Float32 err_angle;
        err_angle.data =  error_angle * 180 / CV_PI;
        error_angle_pub.publish(err_angle);

        ROS_INFO("angular_vel: %0.3f", cmd_vel.angular.z);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}