/*********************************************************************
*
*********************************************************************/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/fill_image.h>

cv::Mat g_last_image;
std::string g_window_name;

// min and max 
int MIN_DEPTH = 0;
int MAX_DEPTH = 10000;

// image data
sensor_msgs::Image img_depth;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    char* img_mono8 = new char[msg->height*msg->width];
    float* img_32FC1 = new float[msg->height*msg->width];

    memcpy(img_32FC1,&msg->data[0],sizeof(float)*msg->height*msg->width);


    // change disp to depth
    for(int i = 0 ; i < msg->height; i++)
    {
        for(int j = 0; j<msg->width; j++)
        {
            float data_tmp = img_32FC1[i*msg->width + j] > MAX_DEPTH ? MAX_DEPTH : img_32FC1[i*msg->width + j];
            if (data_tmp == 0){
                data_tmp = MAX_DEPTH;
            }
            else{
             data_tmp = data_tmp < MIN_DEPTH ? MIN_DEPTH : data_tmp;
            }
            img_mono8[i*msg->width + j] =  ( unsigned char )255 * (1.0 - (data_tmp - MIN_DEPTH)/(MAX_DEPTH - MIN_DEPTH));
        }
    }

    fillImage(img_depth, "mono8", msg->height, msg->width, msg->width, img_mono8);

    delete[] img_mono8;
    delete[] img_32FC1;

    // Convert to OpenCV native BGR color
    try {
        g_last_image = cv_bridge::toCvCopy(img_depth,"mono8")->image;

    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR( "Unable to convert '%s' image for display: '%s'",
                           msg->encoding.c_str(), e.what());
    }
    if (!g_last_image.empty()) {
        const cv::Mat &image = g_last_image;
        cv::imshow(g_window_name, image);
        cv::waitKey(3);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_view");

    if (ros::names::remap("image")=="image"){
        ROS_WARN("Topic 'image' has not been remapped! Typical command-line usage:\n"
                 "/t$ rosrun movesense_sensor image_view image:=<image topic>");
    }
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    // Default window name is the resolved topic name
    std::string topic = nh.resolveName("image");
    local_nh.param("window_name",g_window_name,topic);

    // Handle window size
    bool autosize;
    local_nh.param("autosize",autosize, true);
    cv::namedWindow(g_window_name, autosize ? (CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED) : 0);

    // CreateTrackbar
    cvCreateTrackbar( "Depth_Min", g_window_name.c_str(), &MIN_DEPTH, 10000);
    cvCreateTrackbar( "Depth_Max", g_window_name.c_str(), &MAX_DEPTH, 50000);

    // Start the OpenCV window thread so we don't have to waitKey() somewhere
    cv::startWindowThread();

    // Handle transport
    std::string transport;
    local_nh.param("image_transport", transport, std::string("raw"));
    ROS_INFO_STREAM("Using transport \"" << transport << "\"");

    image_transport::ImageTransport it(nh);
    image_transport::TransportHints hints(transport, ros::TransportHints(), local_nh);
    image_transport::Subscriber sub = it.subscribe(topic, 1, imageCallback, hints);

    ros::spin();

    cv::destroyWindow(g_window_name);

    return 0;
}
