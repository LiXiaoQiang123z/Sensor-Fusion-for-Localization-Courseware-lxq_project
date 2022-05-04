/*
 * @Description: 地图匹配任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/matching/matching_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
    // 定位：初始化-匹配：sub 去畸变点云&gnss【匹配这俩-获得精准位姿】
MatchingFlow::MatchingFlow(ros::NodeHandle& nh) {
    // subscriber:
    // a. undistorted Velodyne measurement: 
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    // b. lidar pose in map frame:
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

    // publisher:
    // a. global point cloud map:
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    // b. local point cloud map:
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    // c. current scan:
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
    // d. estimated lidar pose in map frame:
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/map", "/lidar", 100);
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/vehicle_link");

    matching_ptr_ = std::make_shared<Matching>();
}

// 主要过程
bool MatchingFlow::Run() {
    // 全局地图
    if (matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        matching_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);
    }

    // 局部地图
    if (matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        local_map_pub_ptr_->Publish(matching_ptr_->GetLocalMap());

    ReadData(); // 读取数据

    while(HasData()) {
        if (!ValidData()) {
            LOG(INFO) << "Invalid data. Skip matching" << std::endl;
            continue;
        }

        if (UpdateMatching()) {
            PublishData();
        }
    }

    return true;
}

bool MatchingFlow::ReadData() {
    // pipe lidar measurements and pose into buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    return true;
}

bool MatchingFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    
    if (matching_ptr_->HasInited())
        return true;
    
    if (gnss_data_buff_.size() == 0)
        return false;
        
    return true;
}

bool MatchingFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();

    if (matching_ptr_->HasInited()) { // 匹配是否初始化||若已经初始化，则不需【位姿初始化】过程了
        cloud_data_buff_.pop_front();
        gnss_data_buff_.clear();
        return true;
    }

    current_gnss_data_ = gnss_data_buff_.front();

    double diff_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

#define gnss_or_ScanContext 1
bool gnss_or_ScanContext_bool=false;
// 主要的过程： 
bool MatchingFlow::UpdateMatching() {
    if (!matching_ptr_->HasInited()) {
        //
        // TODO: implement global initialization here
        //
        // Hints: You can use SetGNSSPose & SetScanContextPose from matching.hpp
        //

        // // naive implementation:
        // Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();
        // matching_ptr_->SetInitPose(init_pose);
        // matching_ptr_->SetInited();

        /**
         * @brief Construct a new if object
         * 1.以粗略的gnss位姿作为初始值
         * 2.通过雷达点云和地图点云匹配，完成精度初始位姿
         *
         * current_cloud_data_
         * current_gnss_data_
         */
#if gnss_or_ScanContext == 0
        // 利用GNSS找到定位的初始位姿，需要在建图的时候保存点云的初始位姿，【建图的时候保存】
        //然后，定位的时候修改对应gnss 初始化位置函数，将地图的初始GNSS位姿作为定位gnss数据的的原点。
        matching_ptr_->SetGNSSPose(current_gnss_data_.pose);
        if(gnss_or_ScanContext_bool == false){
            LOG(INFO)<<"gnss:SetGNSSPose is ok "<<std::endl;
            gnss_or_ScanContext_bool=true;
        }
#elif gnss_or_ScanContext == 1
        // 如果利用了闭环检测，则以上不需要了。（闭环检测可以在任意位置定位-即精度初始位姿）
        matching_ptr_->SetGNSSPose(current_gnss_data_.pose);
        matching_ptr_->SetScanContextPose(current_cloud_data_);
        if (gnss_or_ScanContext_bool == false){
            LOG(INFO) << "loopclosure :SetScanContextPose is ok " << std::endl;
            gnss_or_ScanContext_bool = true;
        }
#endif
        matching_ptr_->SetInited();
    }

    return matching_ptr_->Update(current_cloud_data_, laser_odometry_); // 定位的主要流程：和局部地图匹配 + 更新局部地图
}

// pub
bool MatchingFlow::PublishData()
{
    laser_tf_pub_ptr_->SendTransform(laser_odometry_, current_cloud_data_.time);
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);
    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());

    return true;
}
}