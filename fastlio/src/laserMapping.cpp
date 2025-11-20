// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <vector>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
std::mutex imu_buffer_mutex;
std::mutex lidar_buffer_mutex;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool    is_first_lidar = true;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    std::lock_guard<std::mutex> lock(lidar_buffer_mutex);
    scan_count ++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    std::lock_guard<std::mutex> lock(lidar_buffer_mutex);
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if(is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);


    {
        std::lock_guard<std::mutex> lock(imu_buffer_mutex);    
        if (timestamp < last_timestamp_imu)
        {
            std::cerr << "lidar loop back, clear buffer" << std::endl;
            imu_buffer.clear();
        }

        last_timestamp_imu = timestamp;

        imu_buffer.push_back(msg);
    }
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas, rclcpp::Node::SharedPtr node)
{
    std::lock_guard<std::mutex> lock1(imu_buffer_mutex);
    std::lock_guard<std::mutex> lock2(lidar_buffer_mutex);
    RCLCPP_INFO(node->get_logger(), "[PERF] Buffer Empty: Lidar size: %d IMU size: %d" , 
                           (int)lidar_buffer.size(), (int)imu_buffer.size());
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        
        return false;
    }
    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        RCLCPP_INFO(node->get_logger(), "[PERF] Newest Lidar time: %.3f Newest IMU time: %.3f" , 
                           lidar_end_time, last_timestamp_imu);
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "lidar_odom";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    /*
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
    */
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudBaseLink(new PointCloudXYZI(size, 1));
    double roll = 0.0, pitch = -0.75, yaw = 0.0; // 根据实际安装调整
    Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d base_to_livox_r_ = (rz * ry * rx).toRotationMatrix(); // ZYX 顺序
    Eigen::Vector3d base_to_livox_t_ = Eigen::Vector3d::Zero();
// 预计算变换矩阵的转置（从livox_frame到base_link的旋转）
    Eigen::Matrix3d livox_to_base_r = base_to_livox_r_.transpose();
    
    for (int i = 0; i < size; i++)
    {
        // 1. 首先将点从lidar坐标系转换到livox_frame坐标系
        PointType point_livox;
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], &point_livox);
        
        // 2. 然后将点从livox_frame转换到base_link坐标系
        V3D p_livox(point_livox.x, point_livox.y, point_livox.z);
        
        // 由于base_to_livox_t_是零向量，简化计算: p_base = R^T * p_livox
        V3D p_base = livox_to_base_r * p_livox;
        
        // 3. 设置点云数据
        PointType& point_base = laserCloudBaseLink->points[i];
        point_base.x = p_base(0);
        point_base.y = p_base(1);
        point_base.z = p_base(2);
        point_base.intensity = point_livox.intensity;
        point_base.curvature = point_livox.curvature;
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudBaseLink, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "base_link";  // 设置为base_link坐标系
    
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "lidar_odom";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "lidar_odom";
    pubLaserCloudMap->publish(laserCloudmsg);
}

void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}
template<typename T>
void set_twiststamp(T & out)
{

    out.twist.linear.x = state_point.vel(0);
    out.twist.linear.y = state_point.vel(1);
    out.twist.linear.z = state_point.vel(2);
    
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
                                std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br,
                                std::unique_ptr<tf2_ros::Buffer> & tf_buffer_,
                                rclcpp::Logger logger_)
{
    odomAftMapped.header.frame_id = "lidar_odom";
    odomAftMapped.child_frame_id = "base_link";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);


  static geometry_msgs::msg::TransformStamped livox_to_base_link_transform;
  static bool transform_acquired = false; // Check if the transform has already been acquired
  if (!transform_acquired) {

      try {
          livox_to_base_link_transform = tf_buffer_->lookupTransform("livox_frame", "base_link", odomAftMapped.header.stamp);
          transform_acquired = true; // Set the flag to true indicating that the transform has been acquired
      } catch (tf2::TransformException &ex) {
          RCLCPP_ERROR(logger_, "Failed to lookup transform from base_link to livox_frame: %s", ex.what());
          return;
      }
  }
  
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = odomAftMapped.header.stamp;
  transform_stamped.header.frame_id = "lidar_odom";  // Source frame
  transform_stamped.child_frame_id = "base_link";    // Target frame
  
  tf2::Transform tf_lidar_odom_to_livox_frame;
  tf2::fromMsg(odomAftMapped.pose.pose, tf_lidar_odom_to_livox_frame);
  tf2::Transform tf_livox_frame_to_base_link;
  tf2::fromMsg(livox_to_base_link_transform.transform, tf_livox_frame_to_base_link);
  tf2::Transform tf_lidar_odom_to_base_link = tf_lidar_odom_to_livox_frame * tf_livox_frame_to_base_link;

  transform_stamped.transform = tf2::toMsg(tf_lidar_odom_to_base_link);

  tf_br->sendTransform(transform_stamped);

  geometry_msgs::msg::Transform transform_msg = tf2::toMsg(tf_lidar_odom_to_base_link);
    odomAftMapped.pose.pose.position.x = transform_msg.translation.x;
    odomAftMapped.pose.pose.position.y = transform_msg.translation.y;
    odomAftMapped.pose.pose.position.z = transform_msg.translation.z;

    odomAftMapped.pose.pose.orientation.x = transform_msg.rotation.x;
    odomAftMapped.pose.pose.orientation.y = transform_msg.rotation.y;
    odomAftMapped.pose.pose.orientation.z = transform_msg.rotation.z;
    odomAftMapped.pose.pose.orientation.w = transform_msg.rotation.w;

    // geometry_msgs::msg::Twist twist_livox;
    // set_twiststamp(twist_livox);

    // tf2::Vector3 vel_livox(twist_livox.linear.x, twist_livox.linear.y, twist_livox.linear.z);
    // tf2::Vector3 vel_base = tf_livox_frame_to_base_link.getBasis().transpose() * vel_livox;

    // odomAftMapped.twist.twist.linear.x = vel_base.x();
    // odomAftMapped.twist.twist.linear.y = vel_base.y();
    // odomAftMapped.twist.twist.linear.z = vel_base.z();

    // odomAftMapped.twist.twist.angular.x = twist_livox.angular.x;
    // odomAftMapped.twist.twist.angular.y = twist_livox.angular.y;
    // odomAftMapped.twist.twist.angular.z = twist_livox.angular.z;

    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped base_link_to_heading;
    base_link_to_heading.header.stamp = odomAftMapped.header.stamp;
    base_link_to_heading.header.frame_id = "base_link";   // ← 你要求的“航向系”
    base_link_to_heading.child_frame_id = "heading_frame";
    base_link_to_heading.transform.translation.x = 0.0;
    base_link_to_heading.transform.translation.y = 0.0;
    base_link_to_heading.transform.translation.z = 0.0;
    double roll, pitch, yaw;
    tf2::Matrix3x3 mat(tf2::Quaternion(
        transform_msg.rotation.x,
        transform_msg.rotation.y,
        transform_msg.rotation.z,
        transform_msg.rotation.w
    ));
    mat.getRPY(roll, pitch, yaw);  // 获取当前姿态的欧拉角

    // 构造新四元数：只保留 yaw，roll=0, pitch=0
    tf2::Quaternion quat_heading;
    quat_heading.setRPY(-roll, -pitch, 0.0);  // ← 关键！水平化！

    base_link_to_heading.transform.rotation.x = quat_heading.getX();
    base_link_to_heading.transform.rotation.y = quat_heading.getY();
    base_link_to_heading.transform.rotation.z = quat_heading.getZ();
    base_link_to_heading.transform.rotation.w = quat_heading.getW();

    // 发布这个“水平航向系”
    tf_br->sendTransform(base_link_to_heading);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "lidar_odom";

    /*** if path is too large, the rvis will crash ***/
    // static int jjj = 0;
    // jjj++;
    // if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<bool>("high_frequency_odom_en", false);  // 新增：是否启用高频率里程计
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        
        // 获取高频率里程计参数
        this->get_parameter_or<bool>("high_frequency_odom_en", high_frequency_odom_en_, true);
        
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

        

        // 初始化IMU积分相关变量
        imu_integration_enabled_ = false;
        last_imu_integration_time_ = 0.0;
        high_frequency_odom_en_ = false;  // 默认启用高频率模式

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="lidar_odom";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;
        imu_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        lidar_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // 配置订阅选项
        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_callback_group_;
        
        rclcpp::SubscriptionOptions lidar_options;
        lidar_options.callback_group = lidar_callback_group_;
        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk,lidar_options);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk,lidar_options);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);
        // pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fastlio2/body_cloud", 20);
        // pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        // pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubBaseLinkOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("/robotpos", 20);
        // pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        //------------------------------------------------------------------------------------------------------
        // 主处理循环保持100Hz，等待雷达数据
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 50.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));
        // 根据参数决定是否创建高频率IMU积分定时器
        if (high_frequency_odom_en_)
        {
            auto imu_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 60.0));
            imu_timer_ = rclcpp::create_timer(this, this->get_clock(), imu_period_ms, std::bind(&LaserMappingNode::imu_integration_callback, this));
            RCLCPP_INFO(this->get_logger(), "High frequency IMU integration enabled at 60Hz");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "High frequency IMU integration disabled, using lidar rate odometry");
        }

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        // map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
        fclose(fp);
    }

private:
    void timer_callback()
{
    // 总体计时开始
    double total_start_time = omp_get_wtime();
    double stage_times[7] = {0}; // 各阶段耗时
    const char* stage_names[] = {
        "Data Sync & Init",    // 阶段0：数据同步和初始化
        "IMU & Preprocess",    // 阶段1：IMU处理和预处理
        "Map Operations",      // 阶段2：地图相关操作
        "State Update",        // 阶段3：状态更新（EKF）
        "Map Incremental",     // 阶段4：地图增量更新
        "Publish Results",     // 阶段5：结果发布
        "IMU Integration"      // 阶段6：IMU积分更新
    };

    if(sync_packages(Measures,this->shared_from_this()))
    {
        double stage_start = omp_get_wtime();
        if (flg_first_scan)
        {
            first_lidar_time = Measures.lidar_beg_time;
            p_imu->first_lidar_time = first_lidar_time;
            flg_first_scan = false;
            // 初始化IMU积分
            lock_guard<mutex> lock(imu_integration_mutex_);
            last_state_point_ = kf.get_x();
            last_imu_integration_time_ = Measures.lidar_beg_time;
            imu_integration_enabled_ = true;
            // 记录阶段0耗时
            stage_times[0] = omp_get_wtime() - stage_start;
            RCLCPP_INFO(this->get_logger(), "[PERF] First scan initialization completed. Time: %.3f ms", stage_times[0] * 1000);
            return;
        }
        stage_times[0] = omp_get_wtime() - stage_start;

        double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;
        double imu_start_time = omp_get_wtime();

        match_time = 0;
        kdtree_search_time = 0.0;
        solve_time = 0;
        solve_const_H_time = 0;
        svd_time   = 0;
        t0 = omp_get_wtime();

        // 阶段1：IMU处理
        stage_start = omp_get_wtime();
        p_imu->Process(Measures, kf, feats_undistort);
        stage_times[1] = omp_get_wtime() - stage_start;
        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

        if (feats_undistort->empty() || (feats_undistort == NULL))
        {
            RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
            return;
        }

        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                        false : true;
        /*** Segment the map in lidar FOV ***/
        stage_start = omp_get_wtime();
        lasermap_fov_segment();
        double map_segment_time = omp_get_wtime() - stage_start;

        /*** downsample the feature points in a scan ***/
        stage_start = omp_get_wtime();
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        t1 = omp_get_wtime();
        double downsampling_time = t1 - stage_start;
        feats_down_size = feats_down_body->points.size();
        /*** initialize the map kdtree ***/
        if(ikdtree.Root_Node == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
            if(feats_down_size > 5)
            {
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);
                for(int i = 0; i < feats_down_size; i++)
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                stage_start = omp_get_wtime();
                ikdtree.Build(feats_down_world->points);
                double kdtree_build_time = omp_get_wtime() - stage_start;
                // 记录阶段2耗时
                stage_times[2] = map_segment_time + downsampling_time + kdtree_build_time;
                RCLCPP_INFO(this->get_logger(), "[PERF] Map initialization: Segment(%.3fms) + Downsample(%.3fms) + KDTreeBuild(%.3fms) = %.3fms", 
                           map_segment_time * 1000, downsampling_time * 1000, kdtree_build_time * 1000, stage_times[2] * 1000);
                return;
            }
            return;
        }
        int featsFromMapNum = ikdtree.validnum();
        kdtree_size_st = ikdtree.size();
        
        if (feats_down_size < 5)
        {
            RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
            return;
        }
        
        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);

        V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
        fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
        <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

        /*** iterated state estimation ***/
        stage_start = omp_get_wtime();
        double t_update_start = omp_get_wtime();
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        state_point = kf.get_x();
        euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        geoQuat.x = state_point.rot.coeffs()[0];
        geoQuat.y = state_point.rot.coeffs()[1];
        geoQuat.z = state_point.rot.coeffs()[2];
        geoQuat.w = state_point.rot.coeffs()[3];
        double t_update_end = omp_get_wtime();
        stage_times[3] = t_update_end - stage_start; // 状态更新耗时

        /******* 根据模式决定是否发布里程计 *******/
        stage_start = omp_get_wtime();
        if (!high_frequency_odom_en_)
        {
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_, tf_buffer_, this->get_logger());   
        }
        double odom_publish_time = omp_get_wtime() - stage_start;
        /*** add the feature points to map kdtree ***/
        stage_start = omp_get_wtime();
        t3 = omp_get_wtime();
        map_incremental();
        t5 = omp_get_wtime();
        stage_times[4] = t5 - stage_start; // 地图增量更新耗时
        
        /******* Publish points *******/
        stage_start = omp_get_wtime();
        // if (path_en)                         publish_path(pubPath_);
        // if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
        if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
        if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
        // if (map_pub_en) publish_map(pubLaserCloudMap_);
        stage_times[5] = omp_get_wtime() - stage_start; // 发布结果耗时

        /*** Debug variables ***/
        if (runtime_pos_log)
        {
            frame_num ++;
            kdtree_size_end = ikdtree.size();
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
            aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
            aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
            aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
            aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
            aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
            T1[time_log_counter] = Measures.lidar_beg_time;
            s_plot[time_log_counter] = t5 - t0;
            s_plot2[time_log_counter] = feats_undistort->points.size();
            s_plot3[time_log_counter] = kdtree_incremental_time;
            s_plot4[time_log_counter] = kdtree_search_time;
            s_plot5[time_log_counter] = kdtree_delete_counter;
            s_plot6[time_log_counter] = kdtree_delete_time;
            s_plot7[time_log_counter] = kdtree_size_st;
            s_plot8[time_log_counter] = kdtree_size_end;
            s_plot9[time_log_counter] = aver_time_consu;
            s_plot10[time_log_counter] = add_point_size;
            time_log_counter ++;
            
            // 计算阶段2总耗时（地图操作）
            stage_times[2] = map_segment_time + downsampling_time + (t3 - t1 - stage_times[3] - stage_times[4]); // 减去状态更新和地图增量的时间
            
            // 打印详细的性能分析
            double total_time = omp_get_wtime() - total_start_time;
            double imu_time = stage_times[1];
            double map_ops_time = stage_times[2];
            double state_update_time = stage_times[3];
            double map_incremental_time = stage_times[4];
            double publish_time = stage_times[5];
            
            RCLCPP_INFO(this->get_logger(), "\n[PERF] ====== Frame Performance Analysis ======");
            RCLCPP_INFO(this->get_logger(), "[PERF] Total processing time: %.3f ms", total_time * 1000);
            RCLCPP_INFO(this->get_logger(), "[PERF] Stage breakdown:");
            RCLCPP_INFO(this->get_logger(), "[PERF]   %s: %.3f ms (%.1f%%)", stage_names[0], stage_times[0] * 1000, stage_times[0] / total_time * 100);
            RCLCPP_INFO(this->get_logger(), "[PERF]   %s: %.3f ms (%.1f%%)", stage_names[1], imu_time * 1000, imu_time / total_time * 100);
            RCLCPP_INFO(this->get_logger(), "[PERF]   %s: %.3f ms (%.1f%%)", stage_names[2], map_ops_time * 1000, map_ops_time / total_time * 100);
            RCLCPP_INFO(this->get_logger(), "[PERF]   %s: %.3f ms (%.1f%%)", stage_names[3], state_update_time * 1000, state_update_time / total_time * 100);
            RCLCPP_INFO(this->get_logger(), "[PERF]   %s: %.3f ms (%.1f%%)", stage_names[4], map_incremental_time * 1000, map_incremental_time / total_time * 100);
            RCLCPP_INFO(this->get_logger(), "[PERF]   %s: %.3f ms (%.1f%%)", stage_names[5], publish_time * 1000, publish_time / total_time * 100);
            
            // 关键性能指标
            RCLCPP_INFO(this->get_logger(), "[PERF] Key metrics:");
            RCLCPP_INFO(this->get_logger(), "[PERF]   Input points: %d, Downsampled: %d", 
                       (int)feats_undistort->points.size(), feats_down_size);
            RCLCPP_INFO(this->get_logger(), "[PERF]   Map points: %d -> %d", kdtree_size_st, kdtree_size_end);
            RCLCPP_INFO(this->get_logger(), "[PERF]   ICP iterations: %d, Match time: %.3f ms", 
                       (int)(t_update_end - t_update_start > 0 ? 1 : 0), match_time * 1000);
            RCLCPP_INFO(this->get_logger(), "[PERF]   KDTree search time: %.3f ms, Incremental time: %.3f ms", 
                       kdtree_search_time * 1000, kdtree_incremental_time * 1000);
            
            // 长期平均性能
            RCLCPP_INFO(this->get_logger(), "[PERF] Running averages (over %d frames):", frame_num);
            RCLCPP_INFO(this->get_logger(), "[PERF]   Total: %.3f ms, ICP: %.3f ms, Match: %.3f ms", 
                       aver_time_consu * 1000, aver_time_icp * 1000, aver_time_match * 1000);
            RCLCPP_INFO(this->get_logger(), "[PERF]   Solve: %.3f ms, Const H: %.3f ms, Incre: %.3f ms", 
                       aver_time_solve * 1000, aver_time_const_H_time * 1000, aver_time_incre * 1000);
            RCLCPP_INFO(this->get_logger(), "[PERF] ========================================\n");
            
            ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
            dump_lio_state_to_log(fp);
        }
        
        // 更新IMU积分状态 - 在雷达更新后同步状态
        stage_start = omp_get_wtime();
        {
            lock_guard<mutex> lock(imu_integration_mutex_);
            last_state_point_ = state_point;
            last_imu_integration_time_ = lidar_end_time;
        }
        stage_times[6] = omp_get_wtime() - stage_start;
        
        // 如果没有启用runtime_pos_log，仍然打印简要性能信息
        double total_time = omp_get_wtime() - total_start_time;
        RCLCPP_INFO(this->get_logger(), "[PERF] [TIME] %.3f: Frame %d: Total %.3fms | IMU: %.3fms | StateUpdate: %.3fms | MapIncre: %.3fms | Points: %d->%d", 
                    lidar_end_time, frame_num, total_time * 1000, stage_times[1] * 1000, stage_times[3] * 1000, 
                    stage_times[4] * 1000, (int)feats_undistort->points.size(), feats_down_size);
        
    }
}

    // 新增：高频率IMU积分回调函数，60Hz
    void imu_integration_callback()
    {
        if (!imu_integration_enabled_ || !flg_EKF_inited)
        {
            return;
        }
        
        double current_time = this->get_clock()->now().seconds();
        
        // 使用IMU数据进行状态预测
        state_ikfom predicted_state;
        bool prediction_success = false;
        vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_for_integration;
        {
            lock_guard<mutex> lock_integration(imu_integration_mutex_);
            lock_guard<mutex> lock_buffer(mtx_buffer);
            
            predicted_state = last_state_point_;
            double integration_start_time = last_imu_integration_time_;
            
            // 查找从上次积分时间到当前时间的IMU数据
            
            for (auto imu_msg : imu_buffer)
            {
                double imu_time = get_time_sec(imu_msg->header.stamp);
                if (imu_time > integration_start_time && imu_time <= current_time)
                {
                    imu_for_integration.push_back(imu_msg);
                }
            }
            
            // 如果有IMU数据则进行积分
            if (!imu_for_integration.empty())
            {
                prediction_success = perform_imu_integration(predicted_state, imu_for_integration, integration_start_time, current_time);
                if (prediction_success)
                {
                    last_state_point_ = predicted_state;
                    last_imu_integration_time_ = current_time;
                }
            }
        }
         
        // 如果预测成功，发布高频率的里程计
        if (prediction_success)
        {
            publish_high_frequency_odometry(predicted_state, current_time,imu_for_integration[imu_for_integration.size()-1]);
        }
    }
    
    // IMU积分预测函数
    bool perform_imu_integration(state_ikfom& state, const vector<sensor_msgs::msg::Imu::ConstSharedPtr>& imu_data, double start_time, double end_time)
    {
        if (imu_data.empty()) return false;
        
        for (size_t i = 0; i < imu_data.size() - 1; i++)
        {
            auto imu_curr = imu_data[i];
            auto imu_next = imu_data[i + 1];
            
            double dt = get_time_sec(imu_next->header.stamp) - get_time_sec(imu_curr->header.stamp);
            if (dt <= 0 || dt > 0.1) continue;  // 跳过无效的时间间隔
            
            // 获取IMU测量值
            V3D acc_measurement(imu_curr->linear_acceleration.x, 
                              imu_curr->linear_acceleration.y, 
                              imu_curr->linear_acceleration.z);
            V3D gyr_measurement(imu_curr->angular_velocity.x, 
                              imu_curr->angular_velocity.y, 
                              imu_curr->angular_velocity.z);
            
            // 补偿bias
            V3D acc_unbias = acc_measurement - state.ba;
            V3D gyr_unbias = gyr_measurement - state.bg;
            
            // 姿态更新 (使用SO3指数映射)
            state.rot = state.rot * SO3::exp(gyr_unbias * dt);
            
            // 速度更新 (在世界坐标系下)
            V3D grav_vec = state.grav.get_vect();  // Convert S2 to V3D
            V3D acc_world = state.rot * acc_unbias + grav_vec;
            state.vel = state.vel + acc_world * dt;
            
            // 位置更新
            state.pos = state.pos + state.vel * dt + 0.5 * acc_world * dt * dt;
        }
        
        return true;
    }
    
    // 发布高频率里程计
    void publish_high_frequency_odometry(const state_ikfom& state, double timestamp,sensor_msgs::msg::Imu::ConstSharedPtr &imudata)
    {
        // 更新全局状态用于发布
        euler_cur = SO3ToEuler(state.rot);
        geoQuat.x = state.rot.coeffs()[0];
        geoQuat.y = state.rot.coeffs()[1];  
        geoQuat.z = state.rot.coeffs()[2];
        geoQuat.w = state.rot.coeffs()[3];
        
        // 计算lidar在world坐标系下的位置
        pos_lid = state.pos + state.rot * state.offset_T_L_I;
        
        // 发布里程计
        odomAftMapped.header.frame_id = "lidar_odom";
        odomAftMapped.child_frame_id = "livox_frame";
        odomAftMapped.header.stamp = imudata->header.stamp;
        
        odomAftMapped.pose.pose.position.x = state.pos(0);
        odomAftMapped.pose.pose.position.y = state.pos(1);
        odomAftMapped.pose.pose.position.z = state.pos(2);
        odomAftMapped.pose.pose.orientation.x = geoQuat.x;
        odomAftMapped.pose.pose.orientation.y = geoQuat.y;
        odomAftMapped.pose.pose.orientation.z = geoQuat.z;
        odomAftMapped.pose.pose.orientation.w = geoQuat.w;
        
        odomAftMapped.twist.twist.linear.x = state.vel(0);
        odomAftMapped.twist.twist.linear.y = state.vel(1);
        odomAftMapped.twist.twist.linear.z = state.vel(2);
        
        odomAftMapped.twist.twist.angular.x = imudata->angular_velocity.x;
        odomAftMapped.twist.twist.angular.y = imudata->angular_velocity.y;
        odomAftMapped.twist.twist.angular.z = imudata->angular_velocity.z;
        
        pubOdomAftMapped_->publish(odomAftMapped);
        
        // 发布TF
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = odomAftMapped.header.stamp;
        transform_stamped.header.frame_id = "lidar_odom";
        transform_stamped.child_frame_id = "base_link";
        static geometry_msgs::msg::TransformStamped livox_to_base_link_transform;
        static bool transform_acquired = false; // Check if the transform has already been acquired
        if (!transform_acquired) {
            // Get the transform from base_link to livox_frame
            try {
                livox_to_base_link_transform = tf_buffer_->lookupTransform("livox_frame", "base_link", odomAftMapped.header.stamp, tf2::durationFromSec(0.001));
                transform_acquired = true; // Set the flag to true indicating that the transform has been acquired
            } catch (tf2::TransformException &ex) {
                return;
            }
        }
        tf2::Transform tf_lidar_odom_to_livox_frame;
        tf2::fromMsg(odomAftMapped.pose.pose, tf_lidar_odom_to_livox_frame);
        
        // 假设有固定的livox_frame到base_link的变换，这里简化处理
        tf2::Transform tf_livox_frame_to_base_link;
        tf2::fromMsg(livox_to_base_link_transform.transform, tf_livox_frame_to_base_link);
        tf2::Transform tf_lidar_odom_to_base_link = tf_lidar_odom_to_livox_frame * tf_livox_frame_to_base_link;
        transform_stamped.transform = tf2::toMsg(tf_lidar_odom_to_base_link);
        
        tf_broadcaster_->sendTransform(transform_stamped);

        geometry_msgs::msg::TransformStamped heading_to_livox;
        heading_to_livox.header.stamp = odomAftMapped.header.stamp;
        heading_to_livox.header.frame_id = "livox_frame";   // ← 你要求的“航向系”
        heading_to_livox.child_frame_id = "heading_frame";
        heading_to_livox.transform.translation.x = 0.0;
        heading_to_livox.transform.translation.y = 0.0;
        heading_to_livox.transform.translation.z = 0.0;
        double roll, pitch, yaw;
        tf2::Matrix3x3 mat(tf2::Quaternion(
            state.rot.coeffs()[0],
            state.rot.coeffs()[1],
            state.rot.coeffs()[2],
            state.rot.coeffs()[3]
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
        tf_broadcaster_->sendTransform(heading_to_livox);

        //发布机器人位姿
        nav_msgs::msg::Odometry base_link_odom;
        base_link_odom.header.frame_id = "lidar_odom";
        base_link_odom.child_frame_id = "base_link";
        base_link_odom.header.stamp = odomAftMapped.header.stamp;
        base_link_odom.pose.pose.position.x = tf_lidar_odom_to_base_link.getOrigin().x();
        base_link_odom.pose.pose.position.y = tf_lidar_odom_to_base_link.getOrigin().y();
        base_link_odom.pose.pose.position.z = tf_lidar_odom_to_base_link.getOrigin().z();

        tf2::Quaternion base_quat = tf_lidar_odom_to_base_link.getRotation();
        base_link_odom.pose.pose.orientation.x = base_quat.x();
        base_link_odom.pose.pose.orientation.y = base_quat.y();
        base_link_odom.pose.pose.orientation.z = base_quat.z();
        base_link_odom.pose.pose.orientation.w = base_quat.w();
        pubBaseLinkOdom_->publish(base_link_odom);
    }

    void map_publish_callback()
    {
        if (map_pub_en) publish_map(pubLaserCloudMap_);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            save_to_pcd();
            res->success = true;
            res->message = "Map saved.";
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubBaseLinkOdom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;  // 新增：高频IMU积分定时器
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;
    

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    // IMU积分相关变量
    mutex imu_integration_mutex_;
    state_ikfom last_state_point_;  // 上一次状态
    double last_imu_integration_time_;  // 上一次IMU积分时间
    bool imu_integration_enabled_;  // IMU积分是否启用
    bool high_frequency_odom_en_;   // 是否启用高频率里程计模式

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);
    auto node = std::make_shared<LaserMappingNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // rclcpp::spin(std::make_shared<LaserMappingNode>());
    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    while (rclcpp::ok() && !flg_exit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}