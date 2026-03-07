#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

class RealSenseNode : public rclcpp::Node
{
public:
    RealSenseNode() : Node("realsense_node"), running_(true)
    {
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 25);
        imu_pub_   = create_publisher<sensor_msgs::msg::Imu>("/camera/imu", 25);
        pc_pub_    = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/pointcloud", 25);

        // --- Color + Depth pipeline ---
        color_config_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        color_config_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        try {
            color_pipeline_.start(color_config_);
            RCLCPP_INFO(get_logger(), "Color+Depth pipeline started");
        } catch (const rs2::error & e) {
            RCLCPP_FATAL(get_logger(), "Color pipeline error: %s", e.what());
            rclcpp::shutdown(); return;
        }

        // --- IMU pipeline ---
        imu_config_.enable_stream(RS2_STREAM_ACCEL);
        imu_config_.enable_stream(RS2_STREAM_GYRO);
        try {
            imu_pipeline_.start(imu_config_);
            RCLCPP_INFO(get_logger(), "IMU pipeline started");
        } catch (const rs2::error & e) {
            RCLCPP_FATAL(get_logger(), "IMU pipeline error: %s", e.what());
            rclcpp::shutdown(); return;
        }

        // Color + depth at 30fps
        color_timer_ = create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&RealSenseNode::publish_color_and_pc, this));

        // IMU at 100hz
        imu_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&RealSenseNode::publish_imu, this));
    }

    ~RealSenseNode()
    {
        running_ = false;
        try { color_pipeline_.stop(); } catch (...) {}
        try { imu_pipeline_.stop(); } catch (...) {}
    }

private:
    void publish_color_and_pc()
    {
        rs2::frameset frames;
        if (!color_pipeline_.poll_for_frames(&frames)) return;

        // --- Color image ---
        rs2::video_frame color_frame = frames.get_color_frame();
        if (color_frame) {
            cv::Mat image(cv::Size(640, 480), CV_8UC3,
                          (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat image_copy = image.clone();
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_copy).toImageMsg();
            msg->header.stamp    = get_clock()->now();
            msg->header.frame_id = "camera_color_frame";
            image_pub_->publish(*msg);
        }

        // --- Point cloud from depth ---
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (!depth_frame) return;

        // Align depth to color for colored point cloud
        rs2::align align_to_color(RS2_STREAM_COLOR);
        rs2::frameset aligned = align_to_color.process(frames);
        rs2::depth_frame aligned_depth = aligned.get_depth_frame();
        rs2::video_frame aligned_color = aligned.get_color_frame();
        if (!aligned_depth || !aligned_color) return;

        rs2::pointcloud pc;
        rs2::points points;
        pc.map_to(aligned_color);
        points = pc.calculate(aligned_depth);

        auto vertices   = points.get_vertices();
        auto tex_coords = points.get_texture_coordinates();
        int n = points.size();

        // Build PointCloud2 message with standard xyz+rgb packing for Foxglove
        sensor_msgs::msg::PointCloud2 pc_msg;
        pc_msg.header.stamp    = get_clock()->now();
        pc_msg.header.frame_id = "camera_color_frame";
        pc_msg.height   = 1;
        pc_msg.width    = n;
        pc_msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(pc_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(n);

        sensor_msgs::PointCloud2Iterator<float>   iter_x(pc_msg, "x");
        sensor_msgs::PointCloud2Iterator<float>   iter_y(pc_msg, "y");
        sensor_msgs::PointCloud2Iterator<float>   iter_z(pc_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pc_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pc_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pc_msg, "b");

        const uint8_t* color_data = (const uint8_t*)aligned_color.get_data();
        int width  = aligned_color.get_width();
        int height = aligned_color.get_height();

        for (int i = 0; i < n; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
            *iter_x = vertices[i].x;
            *iter_y = vertices[i].y;
            *iter_z = vertices[i].z;

            int u   = std::min(std::max(int(tex_coords[i].u * width),  0), width  - 1);
            int v   = std::min(std::max(int(tex_coords[i].v * height), 0), height - 1);
            int idx = (v * width + u) * 3;
            *iter_r = color_data[idx + 2]; // BGR -> RGB
            *iter_g = color_data[idx + 1];
            *iter_b = color_data[idx + 0];
        }

        pc_pub_->publish(pc_msg);
    }

    void publish_imu()
    {
        rs2::frameset frames;
        if (!imu_pipeline_.poll_for_frames(&frames)) return;

        rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
        rs2::motion_frame gyro_frame  = frames.first_or_default(RS2_STREAM_GYRO);
        if (!accel_frame && !gyro_frame) return;

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp    = get_clock()->now();
        imu_msg.header.frame_id = "camera_imu_frame";

        if (accel_frame) {
            auto accel = accel_frame.get_motion_data();
            imu_msg.linear_acceleration.x = accel.x;
            imu_msg.linear_acceleration.y = accel.y;
            imu_msg.linear_acceleration.z = accel.z;
        }
        if (gyro_frame) {
            auto gyro = gyro_frame.get_motion_data();
            imu_msg.angular_velocity.x = gyro.x;
            imu_msg.angular_velocity.y = gyro.y;
            imu_msg.angular_velocity.z = gyro.z;
        }
        imu_pub_->publish(imu_msg);
    }

    rs2::pipeline color_pipeline_;
    rs2::config   color_config_;
    rs2::pipeline imu_pipeline_;
    rs2::config   imu_config_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::TimerBase::SharedPtr color_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    bool running_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}