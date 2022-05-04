/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

    // sub：畸变补偿后点云  ||  pub：/map/lidar odom_topic
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, odom_topic, "/map", "/lidar", 100);

    front_end_ptr_ = std::make_shared<FrontEnd>();
}

bool FrontEndFlow::Run() {
    if (!ReadData()) // 读取数据
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        if (UpdateLaserOdometry()) { // 主要：更新点云里程计
            PublishData();
        }
    }

    return true;
}
// 新的数据插入
bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    return true;
}

bool FrontEndFlow::HasData() {
    return cloud_data_buff_.size() > 0;
}

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool FrontEndFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;
    if (!odometry_inited) { // 1.里程计初始化：初始位姿为单位阵
        odometry_inited = true;
        // init lidar odometry:
        front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
    }

    // update lidar odometry using current undistorted measurement:
    /**
     * @brief 
     * in   current_cloud_data_ 当前点云数据
     * out  laser_odometry_ 里程计位姿
     * 1.预处理：去除nan值，滤波处理 2.ndt匹配求取第k帧位姿，预测k+1帧位姿 3.关键帧的更新（维护局部地图）
     */
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

// pub:里程计的位姿、时间戳
bool FrontEndFlow::PublishData() {
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);

    return true;
}
}