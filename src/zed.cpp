#include <sl/Camera.hpp>
#include <ros/init.h>
# include <ros/node_handle.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <manif/manif.h>

// TODO: relative path
#include "../utils/lie/lie.hpp"

void fillPointCloudMessageHeader(sensor_msgs::PointCloud2 &msg) {
    sensor_msgs::PointCloud2Modifier modifier{msg};
    modifier.setPointCloud2Fields(
            8,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::PointField::FLOAT32, // by convention rgb is stored as float32 even thought it is three bytes
            "normal_x", 1, sensor_msgs::PointField::FLOAT32,
            "normal_y", 1, sensor_msgs::PointField::FLOAT32,
            "normal_z", 1, sensor_msgs::PointField::FLOAT32,
            "curvature", 1, sensor_msgs::PointField::FLOAT32);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "zed_node");

    // TODO why priv
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};
    tf2_ros::TransformBroadcaster tfBroadcaster;
    
    // Publishers and subscribers
    ros::Publisher pub_left_image = nh.advertise<sensor_msgs::Image>("zed/left/image", 1);
    ros::Publisher pub_left_pc = nh.advertise<sensor_msgs::PointCloud2>("zed/left/point_cloud", 1);

    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 60;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Match ROS
    init_params.depth_maximum_distance = 12.0f;
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;

    if (zed.open(init_params) != sl::ERROR_CODE::SUCCESS) {
        ROS_ERROR("Failed to open ZED camera");
        return 1;
    }
    ROS_INFO("ZED opened successfully");

    sl::RuntimeParameters runtimeParams;

    // TODO: err handle
    sl::PositionalTrackingParameters trackingParams;
    trackingParams.enable_pose_smoothing = true;
    trackingParams.enable_area_memory = true;
    auto err = zed.enablePositionalTracking(trackingParams);

    sl::Mat image;
    cv::Mat bgr;
    sl::Mat point_cloud;
    sl::Pose pose;

    while (ros::ok()) {
        if (zed.grab(runtimeParams) == sl::ERROR_CODE::SUCCESS) {

            // grab from zed
            zed.retrieveImage(image, sl::VIEW::LEFT);
            zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

            cv::Mat bgra{static_cast<int>(image.getHeight()), static_cast<int>(image.getWidth()), CV_8UC4, image.getPtr<sl::uchar1>()};
            cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

            // pc is width x height with each point having 8 32-bit float channels (x, y, z, rgb, normal_x, normal_y, normal_z, curvature)
            // since there's no CV_32FC8, we use CV_64FC4 cuz same number of bytes
            cv::Mat point_cloud_mat{static_cast<int>(point_cloud.getHeight()), static_cast<int>(point_cloud.getWidth()), CV_32FC4, point_cloud.getPtr<sl::uchar1>()};
            ROS_INFO("Matrix width: %d, height: %d", point_cloud_mat.cols, point_cloud_mat.rows);
            sl::POSITIONAL_TRACKING_STATE status = zed.getPosition(pose);
            if (status == sl::POSITIONAL_TRACKING_STATE::OK) {
                sl::Translation const& translation = pose.getTranslation();
                sl::Orientation const& orientation = pose.getOrientation();
                try {
                    SE3d leftCameraInOdom{{translation.x, translation.y, translation.z},
                                            Eigen::Quaterniond{orientation.w, orientation.x, orientation.y, orientation.z}.normalized()};
                    SE3d baseLinkToLeftCamera = SE3Conversions::fromTfTree(tfBuffer, "base_link", "zed_left_camera_frame");
                    SE3d baseLinkInOdom = leftCameraInOdom * baseLinkToLeftCamera;
                    SE3Conversions::pushToTfTree(tfBroadcaster, "base_link", "odom", baseLinkInOdom);
                } catch (tf2::TransformException& e) {
                    ROS_WARN_STREAM("Failed to get transform: " << e.what());
                }
            } else {
                ROS_WARN_STREAM("Positional tracking failed: " << status);
            }

            auto header = std_msgs::Header();
            header.stamp = ros::Time::now();
            header.frame_id = "zed_left_camera_frame";

            // publish left image
            sensor_msgs::Image img_msg;
            img_msg.header = header;
            img_msg.height = bgr.rows;
            img_msg.width = bgr.cols;
            img_msg.encoding = "bgr8";
            img_msg.is_bigendian = false;
            img_msg.step = bgr.step;
            img_msg.data = std::vector<uint8_t>(bgr.data, bgr.data + bgr.total() * bgr.elemSize());
            pub_left_image.publish(img_msg);

            // publish left point cloud
            sensor_msgs::PointCloud2 pc_msg;
            pc_msg.header = header;
            pc_msg.height = point_cloud_mat.rows;
            pc_msg.width = point_cloud_mat.cols;
            fillPointCloudMessageHeader(pc_msg);
            pc_msg.is_bigendian = false;
            // TODO: ?????? math is not mathing??
            pc_msg.point_step = 16;
            pc_msg.row_step = 16 * point_cloud_mat.cols;
            pc_msg.data = std::vector<uint8_t>(point_cloud_mat.data, point_cloud_mat.data + point_cloud_mat.total() * point_cloud_mat.elemSize());
            pub_left_pc.publish(pc_msg);

        } else {
            throw std::runtime_error("Failed to grab");
        }
    }
    return 0;
}