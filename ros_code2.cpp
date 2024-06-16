#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pyzbar/pyzbar.h>

using namespace std;

// Define global variables to store the x, y, w values from QR code
double qr_x = 0.0; 
double qr_y = 0.0;
double qr_w = 0.0;

// Callback function to process the image message
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Convert ROS Image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Detect QR codes in the image
    vector<cv::Point> points;
    vector<cv::Point> qr_center;
    vector<cv::Point> qr_polygon;
    vector<cv::Point> qr_edge;
    vector<cv::Rect> qr_bounding_box;
    vector<string> qr_messages;
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, gray, 127, 255, cv::THRESH_BINARY);
    cv::bitwise_not(gray, gray);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(gray, gray, cv::MORPH_DILATE, kernel);
    cv::dilate(gray, gray, cv::Mat(), cv::Point(-1, -1), 2);
    cv::erode(gray, gray, cv::Mat(), cv::Point(-1, -1), 1);
    vector<vector<cv::Point>> contours;
    cv::findContours(gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > 1000) {
            cv::RotatedRect box = cv::minAreaRect(contours[i]);
            vector<cv::Point> approx;
            cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);
            if (approx.size() == 4 && cv::isContourConvex(approx)) {
                cv::Point center;
                center.x = (box.center.x + approx[0].x + approx[2].x) / 3;
                center.y = (box.center.y + approx[0].y + approx[2].y) / 3;
                qr_center.push_back(center);
                qr_polygon.push_back(approx[0]);
                qr_polygon.push_back(approx[1]);
                qr_polygon.push_back(approx[2]);
                qr_polygon.push_back(approx[3]);
                for (size_t j = 0; j < approx.size(); j++) {
                    qr_edge.push_back(approx[j]);
                }
                qr_bounding_box.push_back(cv::boundingRect(contours[i]));
                cv::Mat qr_roi = gray(qr_bounding_box[qr_bounding_box.size() - 1]);
                cv::Mat qr_image = cv_ptr->image(qr_bounding_box[qr_bounding_box.size() - 1]);
                string qr_message = "Undefined";
                vector<pyzbar::ZBarSymbol> qr_code = pyzbar::ZBarDecoder().decodeMulti(qr_roi);
                if (qr_code.size() > 0) {
                    qr_message = qr_code[0].getData();
                }
                qr_messages.push_back(qr_message);
                points.push_back(box.center);
            }
        }
    }

    // Process the QR code data if found
    if (!points.empty()) {
        // Extract x, y, w values from QR code message
        string qr_data = qr_messages[0]; // Assuming only one QR code is present
        stringstream ss(qr_data);
        string token;
        getline(ss, token, ',');
        qr_x = stod(token);
        getline(ss, token, ',');
        qr_y = stod(token);
        getline(ss, token, ',');
        qr_w = stod(token);

        // Print the extracted values (for debugging)
        ROS_INFO("QR Code Data: x=%f, y=%f, w=%f", qr_x, qr_y, qr_w);
    }
}

int main(int argc, char** argv) {
    // argc : 메인함수에 전달되는 정보 개수
    // argv : 메인함수에 전달할 정보 내용 []
    // Initialize ROS node (노드 초기화)
    ros::init(argc, argv, "qr_code_navigation");
    // 노드 이름 : qr_code_navigation
    ros::NodeHandle nh;
    // 노드핸들러 이름 : nh (노드 핸들러 : 센서 데이터 수신, 제어 명령 송신, 토픽 발행/구독 전용 노드 실행용 클래스) 
    // Subscribe to the camera image topic (받는 곳)
    // qr_code_navigation 에서 보낸걸 /camera/image 에서 받는 용도 -> subscriber
    ros::Subscriber sub = nh.subscribe("/camera/image", 1, imageCallback);

    // Create a publisher to publish goals to the move_base action
    // qr_code_navigation 에서 보내는 용도 -> publish
    ros::Publisher pub = nh.advertise<move_base_msgs::MoveBaseAction>("/move_base", 10);

    // 통신 방식 중 action 방식 사용
    // Define a move_base action client
    // move_base_msgs::MoveBaseAction 형식의 클라이언트 정의
    //
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("/move_base", true);

    // Wait for move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Main loop
    ros::Rate rate(10);
    while (ros::ok()) {
        // Check if QR code data is available
        if (qr_x != 0.0 || qr_y != 0.0 || qr_w != 0.0) {
            // Create a MoveBaseGoal message with the extracted QR code data
            move_base_msgs::MoveBaseAction goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = qr_x;
            goal.target_pose.pose.position.y = qr_y;
            goal.target_pose.pose.orientation.w = qr_w;

            // Publish the goal to the move_base action
            pub.publish(goal);

            // Reset QR code data
            qr_x = 0.0;
            qr_y = 0.0;
            qr_w = 0.0;

            // Log
            ROS_INFO("Published goal to move_base");
        }

        // Spin and sleep
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
