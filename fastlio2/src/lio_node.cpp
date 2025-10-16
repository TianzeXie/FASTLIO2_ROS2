#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
// #include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "base_link";
    std::string world_frame = "lidar_odom";
    bool print_time_cost = false;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 10, std::bind(&LIONode::imuCB, this, std::placeholders::_1));
        m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(m_node_config.lidar_topic, 10, std::bind(&LIONode::lidarCB, this, std::placeholders::_1));

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        double roll = 0.0, pitch = -0.75, yaw = 0.0;
        Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
        m_base_to_livox_r = (rz * ry * rx).toRotationMatrix(); // ZYX order
        m_base_to_livox_t = Eigen::Vector3d::Zero();
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 10.0,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    bool syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
        size_t num_points = cloud->size();
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {

        // nav_msgs::msg::Odometry odom;
        // odom.header.frame_id = frame_id;
        // odom.header.stamp = Utils::getTime(time);
        // odom.child_frame_id = child_frame;
        // odom.pose.pose.position.x = m_kf->x().t_wi.x();
        // odom.pose.pose.position.y = m_kf->x().t_wi.y();
        // odom.pose.pose.position.z = m_kf->x().t_wi.z();
        // Eigen::Quaterniond q(m_kf->x().r_wi);
        // odom.pose.pose.orientation.x = q.x();
        // odom.pose.pose.orientation.y = q.y();
        // odom.pose.pose.orientation.z = q.z();
        // odom.pose.pose.orientation.w = q.w();

        // V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        // odom.twist.twist.linear.x = vel.x();
        // odom.twist.twist.linear.y = vel.y();
        // odom.twist.twist.linear.z = vel.z();
        // odom_pub->publish(odom);

        
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        // broad_caster->sendTransform(transformStamped);

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = transformStamped.header.stamp;
        transform_stamped.header.frame_id = "lidar_odom";
        transform_stamped.child_frame_id = "base_link";
        static geometry_msgs::msg::TransformStamped livox_to_base_link_transform;
        static bool transform_acquired = false; // Check if the transform has already been acquired
        if (!transform_acquired) {
            // Get the transform from base_link to livox_frame
            try {
                livox_to_base_link_transform = tf_buffer_->lookupTransform("livox_frame", "base_link", transformStamped.header.stamp, tf2::durationFromSec(0.001));
                transform_acquired = true; // Set the flag to true indicating that the transform has been acquired
            } catch (tf2::TransformException &ex) {
                return;
            }
        }
        tf2::Transform tf_livox_frame_to_base_link;
        tf2::fromMsg(livox_to_base_link_transform.transform, tf_livox_frame_to_base_link);
        tf2::Transform tf_lidar_odom_to_livox_frame;
        tf2::fromMsg(transformStamped.transform, tf_lidar_odom_to_livox_frame);
        tf2::Transform tf_lidar_odom_to_base_link = tf_lidar_odom_to_livox_frame * tf_livox_frame_to_base_link;
        transform_stamped.transform = tf2::toMsg(tf_lidar_odom_to_base_link);
        
        broad_caster->sendTransform(transform_stamped);

        nav_msgs::msg::Odometry base_link_odom;
        base_link_odom.header.frame_id = "lidar_odom";
        base_link_odom.child_frame_id = "base_link";
        base_link_odom.header.stamp = transformStamped.header.stamp;
        base_link_odom.pose.pose.position.x = tf_lidar_odom_to_base_link.getOrigin().x();
        base_link_odom.pose.pose.position.y = tf_lidar_odom_to_base_link.getOrigin().y();
        base_link_odom.pose.pose.position.z = tf_lidar_odom_to_base_link.getOrigin().z();

        tf2::Quaternion base_quat = tf_lidar_odom_to_base_link.getRotation();
        base_link_odom.pose.pose.orientation.x = base_quat.x();
        base_link_odom.pose.pose.orientation.y = base_quat.y();
        base_link_odom.pose.pose.orientation.z = base_quat.z();
        base_link_odom.pose.pose.orientation.w = base_quat.w();
        odom_pub->publish(base_link_odom);
        geometry_msgs::msg::TransformStamped heading_to_livox;
        heading_to_livox.header.stamp = transformStamped.header.stamp;
        heading_to_livox.header.frame_id = "livox_frame";   // ← 你要求的“航向系”
        heading_to_livox.child_frame_id = "heading_frame";
        heading_to_livox.transform.translation.x = 0.0;
        heading_to_livox.transform.translation.y = 0.0;
        heading_to_livox.transform.translation.z = 0.0;
        double roll, pitch, yaw;
        tf2::Matrix3x3 mat(tf2::Quaternion(
            q.x(),
            q.y(),
            q.z(),
            q.w()
        ));
        mat.getRPY(roll, pitch, yaw);  // 获取当前姿态的欧拉角

        // 构造新四元数：只保留 yaw，roll=0, pitch=0
        tf2::Quaternion quat_heading;
        quat_heading.setRPY(-roll, -pitch, 0.0);  // ← 关键！水平化！

        heading_to_livox.transform.rotation.x = quat_heading.getX();
        heading_to_livox.transform.rotation.y = quat_heading.getY();
        heading_to_livox.transform.rotation.z = quat_heading.getZ();
        heading_to_livox.transform.rotation.w = quat_heading.getW();

        // 发布这个“水平航向系”
        broad_caster->sendTransform(heading_to_livox);
    }

    void timerCB()
    {
        if (!syncPackage())
            return;
        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        broadCastTF(m_odom_pub,m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        // publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        Eigen::Matrix3d r_livox_to_base = m_base_to_livox_r.transpose(); // inverse rotation
        Eigen::Vector3d t_livox_to_base = -r_livox_to_base * m_base_to_livox_t; // = 0 in your case

        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(
            m_package.cloud, 
            m_base_to_livox_r.transpose(), 
            Eigen::Vector3d::Zero()
        );

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    Eigen::Matrix3d m_base_to_livox_r;
    Eigen::Vector3d m_base_to_livox_t;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIONode>());
    rclcpp::shutdown();
    return 0;
}