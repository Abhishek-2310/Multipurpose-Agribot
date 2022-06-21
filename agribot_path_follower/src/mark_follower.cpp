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

unsigned int frameCount = 0;

enum ROBOT_STATE
{
    REST,
    FOLLOW,
    TURN_LEFT,
    TURN_RIGHT,
    MOVE_TOWARDS_TURN
};

enum DIRECTION
{
    LEFT,
    RIGHT
};

unsigned int state = REST;

char *StateToString(int state)
{
    switch (state)
    {
    case REST:
        return "rest";
    case TURN_LEFT:
        return "turn left";
    case TURN_RIGHT:
        return "turn right";
    case FOLLOW:
        return "follow";
    case MOVE_TOWARDS_TURN:
        return "towards turn sign";
    default:
        return "unkown state";
    }
}

// given three points of a triangle returns pointing direction(assuming triangle marker is isosceles)
int GetTriangleDirection(vector<Point> points)
{
    Point sorted[3];

    // sorting points based on distance from x axis
    if (points[0].x >= points[1].x)
    {
        sorted[0] = points[0];
        sorted[1] = points[1];
    }
    else
    {
        sorted[0] = points[1];
        sorted[1] = points[0];
    }

    if (points[2].x >= sorted[0].x)
    {
        sorted[2] = sorted[1];
        sorted[1] = sorted[0];
        sorted[0] = points[2];
    }
    else
    {
        if (points[2].x >= sorted[1].x)
        {
            sorted[2] = sorted[1];
            sorted[1] = points[2];
        }
        else
        {
            sorted[2] = points[2];
        }
    }

    int d1 = sorted[0].x - sorted[1].x;
    int d2 = sorted[2].x - sorted[1].x;

    if (abs(d1) > abs(d2))
        return RIGHT;
    else
        return LEFT;
}

void DetectMarkers(cv::Mat &image)
{
    frameCount++;

    error_angle = 0;
    error_dist = 0;

    // cv::imshow("opencv_window", image);
    // cv::waitKey(3);

    // convert to grey scale image
    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);

    // every object having grey scale value greater than threshold is made 0
    cv::Mat binary;
    int threshold = 3;
    cv::threshold(greyImage, binary, threshold, 255, THRESH_BINARY_INV);

    // finds contour in the binary image
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    cv::Mat img_contour = image;
    cv::putText(img_contour, StateToString(state), Point(5, 25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2, 8, false);

    if (contours.size() > 0)
    {
        // cv::Mat img_contour = Mat::zeros(greyImage.size(), CV_8UC3);

        // approximate contours to polygons + get bounding rects and circles
        vector<vector<Point>> contours_poly(contours.size());
        vector<Point2f> center(contours.size());
        vector<float> radius(contours.size());
        vector<Point2f> circle_center;
        Point2f triangle_center;

        // find enclosing polygon which fits arround the contures and get their centers
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::approxPolyDP(Mat(contours[i]), contours_poly[i], 2, true);
            cv::minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);

            // accecpting ploygons with onl certain radius
            if (radius[i] > 7)
            {
                if (contours_poly[i].size() == 3)
                {
                    // triangle marker
                    cv::drawContours(img_contour, contours, i, cv::Scalar(0, 255, 0), 2, 8, hierarchy, 0, Point());
                    triangle_center = center[i];

                    state = MOVE_TOWARDS_TURN;

                    // checking if robot reached the turn sign
                    if (center[i].y > (image.rows * 0.6))
                    {
                        int dir = GetTriangleDirection(contours_poly[i]);
                        if (dir == LEFT)
                        {
                            state = TURN_LEFT;
                            cv::putText(img_contour, "LEFT", center[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8, false);
                        }
                        else if (dir == RIGHT)
                        {
                            state = TURN_RIGHT;
                            cv::putText(img_contour, "RIGHT", center[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8, false);
                        }
                        frameCount = 0;
                    }
                    break;
                }
                else if (contours_poly[i].size() > 5)
                {
                    // circle marker
                    cv::drawContours(img_contour, contours, i, cv::Scalar(0, 255, 0), 2, 8, hierarchy, 0, Point());
                    cv::putText(img_contour, "FOLLOW", center[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8, false);
                    circle_center.push_back(center[i]);

                    if (state == TURN_LEFT || state == TURN_RIGHT)
                    {
                        if (frameCount > 30)
                        {
                            state = FOLLOW;
                            frameCount = 0;
                        }
                    }
                    else
                    {
                        state = FOLLOW;
                    }
                }
            }
            else
            {
                cv::drawContours(img_contour, contours, i, cv::Scalar(255, 255, 255), 2, 8, hierarchy, 0, Point());
            }
        }

        // correcting course while moving towards turn sign
        if (state == MOVE_TOWARDS_TURN)
        {
            float angle = CV_PI / 2;
            if (abs(triangle_center.x - (image.cols / 2)) > 0.0f)
            {
                float slope = (triangle_center.y - image.rows) / (triangle_center.x - (image.cols / 2));
                angle = atan(slope);
            }
            error_angle = (CV_PI / 2) - abs(angle);

            if (angle < 0.0f)
            {
                error_angle *= -1;
            }
        }

        if (circle_center.size() > 0 && state == FOLLOW)
        {
            // line fitting
            Vec4f linefit;
            cv::fitLine(circle_center, linefit, CV_DIST_L2, 0, 0.01, 0.01);

            Point2f pt1;
            pt1.x = linefit[2] - linefit[0] * 1000;
            pt1.y = linefit[3] - linefit[1] * 1000;

            Point2f pt2;
            pt2.x = linefit[2] + linefit[0] * 1000;
            pt2.y = linefit[3] + linefit[1] * 1000;

            // ROS_INFO("P1{x: %0.3f, y: %0.3f}, P2{x: %0.3f, y: %0.3f}", pt1.x, pt1.y, pt2.x, pt2.y);

            // draw fitted line
            cv::line(img_contour, pt1, pt2, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

            // draw desired path line(center of the camera frame)
            cv::line(img_contour, Point(image.cols / 2, 0), Point(image.cols / 2, image.rows), cv::Scalar(255, 0, 255), 2, cv::LINE_AA);

            // compute angle error;
            float angle = CV_PI / 2;
            if (abs(pt1.x - pt2.x) > 0.0f)
            {
                float slope = (pt2.y - pt1.y) / (pt2.x - pt1.x);
                angle = atan(slope);
            }
            error_angle = (CV_PI / 2) - abs(angle);

            if (angle < 0.0f)
                error_angle *= -1;

            // compute lateral distance error (ditance between mid points of the desired and detected line)
            float lateral_dist = 0.0;
            if (abs(angle) < CV_PI / 2)
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

            float error_dist_limit = 200.0f;
            if (abs(error_dist) > error_dist_limit)
            {
                if (error_dist > 0.0)
                {
                    error_dist = error_dist_limit;
                }
                else
                {
                    error_dist = -error_dist_limit;
                }
            }
            // ROS_INFO("err_dist: %0.3f, err_angle: %0.3f deg", error_dist, error_angle * 180 / CV_PI);
        }

        cv::imshow("opencv_window", img_contour);
        cv::waitKey(3);
    }
}

void ImageCallBack(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        DetectMarkers(cv_ptr->image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mark_follower");

    ros::NodeHandle nh;
    ros::Subscriber image_subscriber = nh.subscribe("/agribot/front_camera/image_raw", 1, ImageCallBack);

    ros::Publisher cmdvel_pub = nh.advertise<geometry_msgs::Twist>("/agribot_v2/mobile_base_controller/cmd_vel", 1);
    ros::Publisher error_dist_pub = nh.advertise<std_msgs::Float32>("/error_dist", 1);
    ros::Publisher error_angle_pub = nh.advertise<std_msgs::Float32>("/error_angle", 1);

    ros::Rate loopRate(100);

    cv::namedWindow("opencv_window", cv::WindowFlags::WINDOW_KEEPRATIO);

    float rotSpeedLimit = 30.0;
    float speedLimit = 1;

    while (ros::ok())
    {
        geometry_msgs::Twist cmd_vel;

        // setting cmd_vel depending on the state of the robot
        if (state == FOLLOW)
        {
            cmd_vel.linear.x = speedLimit;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = (error_angle * 30.0) + (-error_dist * 0.2);

            // limiting angular velocity
            if (abs(cmd_vel.angular.z) >= rotSpeedLimit)
            {
                cmd_vel.angular.z = cmd_vel.angular.z > 0 ? rotSpeedLimit : -rotSpeedLimit;
            }

            std_msgs::Float32 err_dist;
            err_dist.data = error_dist;
            error_dist_pub.publish(err_dist);

            std_msgs::Float32 err_angle;
            err_angle.data = error_angle * 180.0 / CV_PI;
            error_angle_pub.publish(err_angle);
        }
        else if (state == TURN_LEFT)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 3.0;
        }
        else if (state == TURN_RIGHT)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = -3.0;
        }
        else if (state == MOVE_TOWARDS_TURN)
        {
            cmd_vel.linear.x = speedLimit;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = (error_angle * 30.0);

            // limiting angular velocity
            if (abs(cmd_vel.angular.z) >= rotSpeedLimit)
            {
                cmd_vel.angular.z = cmd_vel.angular.z > 0 ? rotSpeedLimit : -rotSpeedLimit;
            }
        }
        else if (state == REST)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
        }

        cmdvel_pub.publish(cmd_vel);

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}