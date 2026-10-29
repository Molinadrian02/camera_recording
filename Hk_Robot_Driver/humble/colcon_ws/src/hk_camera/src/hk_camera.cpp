#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "hk_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
#define FIT_min_x 420
#define FIT_min_y 70
#define FIT_max_x 2450
#define FIT_max_y 2000
#endif

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    //********** rosnode init **********/
    rclcpp::init(argc, argv);
    // Permite pasar los parametros de la camara con --ros-args -p FrameRate:=60 ...
    auto hk_camera = std::make_shared<rclcpp::Node>(
        "hk_camera", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    camera::Camera MVS_cap(*hk_camera);
    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hk_camera);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hk_camera/rgb", 1);
    
    sensor_msgs::msg::Image image_msg;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // 就是rgb格式

    rclcpp::Time last_time = hk_camera->now(); // 记录循环开始时间

    while (rclcpp::ok())
    {
        // Publica cada frame en cuanto llega, al ritmo que marque la camara
        if (!MVS_cap.WaitImg(src, 100))
        {
            continue;
        }
#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x, FIT_min_y, FIT_max_x - FIT_min_x, FIT_max_y - FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
        cv_ptr->image = src;
#endif

        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = hk_camera->get_clock()->now(); // ros发出的时间不是快门时间
        image_msg.header.frame_id = "hk_camera";
        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.header.stamp = image_msg.header.stamp;
        image_pub.publish(image_msg, camera_info_msg);    

        rclcpp::Time current_time = hk_camera->now();              // 记录当前时间
        rclcpp::Duration loop_duration = current_time - last_time; // 计算循环耗时
        // 输出循环耗时
        RCLCPP_INFO(hk_camera->get_logger(), "Loop duration: %f seconds", loop_duration.seconds());
        last_time = current_time;                                  // 更新循环开始时间
    }
    rclcpp::shutdown();

    return 0;
}
