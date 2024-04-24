#include <sl/Camera.hpp>
#include <ros/init.h>
# include <ros/node_handle.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "zed_node");

    // TODO why priv
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    // Publishers and subscribers
    ros::Publisher pub_left_image = nh.advertise<sensor_msgs::Image>("zed/left/image", 1);
    ros::Publisher pub_left_pc = nh.advertise<sensor_msgs::PointCloud2>("zed/left/point_cloud", 1);

    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 60;
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;

    if (zed.open(init_params) != sl::ERROR_CODE::SUCCESS) {
        ROS_ERROR("Failed to open ZED camera");
        return 1;
    }
    
    sl::RuntimeParameters runtimeParams;

    sl::Mat image;
    cv::Mat bgr;
    sl::Mat point_cloud;

    while (ros::ok()) {
        if (zed.grab(runtimeParams) ==sl::ERROR_CODE::SUCCESS) {
            // grab from zed
            zed.retrieveImage(image, sl::VIEW::LEFT);
            zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

            cv::Mat bgra{static_cast<int>(image.getHeight()), static_cast<int>(image.getWidth()), CV_8UC4, image.getPtr<sl::uchar1>()};
            cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

            // publish left image
            sensor_msgs::Image img_msg;
            img_msg.header.stamp = ros::Time::now();
            img_msg.height = bgr.rows;
            img_msg.width = bgr.cols;
            img_msg.encoding = "bgr8";
            img_msg.is_bigendian = false;
            img_msg.step = bgr.step;
            img_msg.data = std::vector<uint8_t>(bgr.data, bgr.data + bgr.total() * bgr.elemSize());
            pub_left_image.publish(img_msg);
        } else {
            throw std::runtime_error("Failed to grab");
        }
    }


    return 0;
}