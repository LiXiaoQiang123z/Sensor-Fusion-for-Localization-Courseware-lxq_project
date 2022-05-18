/*
 * @Description: IMU integration activity
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#include <cmath>

#include "imu_integration/estimator/activity.hpp"
#include "glog/logging.h"

#include <iostream> // new
#include <fstream>

namespace imu_integration {

namespace estimator {

// 构造函数： w acc，g 赋值操作
Activity::Activity(void) 
    : private_nh_("~"), 
    initialized_(false),
    // gravity acceleration:
    G_(0, 0, -9.81),
    // angular velocity bias:
    angular_vel_bias_(0.0, 0.0, 0.0),
    // linear acceleration bias:
    linear_acc_bias_(0.0, 0.0, 0.0)
{}

// 初始化
void Activity::Init(void) {
    // param 参数获取
    // parse IMU config:
    private_nh_.param("imu/topic_name", imu_config_.topic_name, std::string("/sim/sensor/imu"));
    // IMUSubscriber 订阅，接受imu的数据 ：1.时间戳 2.角速度 3.加速度 存入imu_data_中
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(private_nh_, imu_config_.topic_name, 1000000);

    // a. gravity constant:
    private_nh_.param("imu/gravity/x", imu_config_.gravity.x,  0.0);
    private_nh_.param("imu/gravity/y", imu_config_.gravity.y,  0.0);
    private_nh_.param("imu/gravity/z", imu_config_.gravity.z, -9.81);
    G_.x() = imu_config_.gravity.x;
    G_.y() = imu_config_.gravity.y;
    G_.z() = imu_config_.gravity.z;

    // b. angular velocity bias:
    private_nh_.param("imu/bias/angular_velocity/x", imu_config_.bias.angular_velocity.x,  0.0);
    private_nh_.param("imu/bias/angular_velocity/y", imu_config_.bias.angular_velocity.y,  0.0);
    private_nh_.param("imu/bias/angular_velocity/z", imu_config_.bias.angular_velocity.z,  0.0);
    angular_vel_bias_.x() = imu_config_.bias.angular_velocity.x;
    angular_vel_bias_.y() = imu_config_.bias.angular_velocity.y;
    angular_vel_bias_.z() = imu_config_.bias.angular_velocity.z;

    // c. linear acceleration bias:
    private_nh_.param("imu/bias/linear_acceleration/x", imu_config_.bias.linear_acceleration.x,  0.0);
    private_nh_.param("imu/bias/linear_acceleration/y", imu_config_.bias.linear_acceleration.y,  0.0);
    private_nh_.param("imu/bias/linear_acceleration/z", imu_config_.bias.linear_acceleration.z,  0.0);
    linear_acc_bias_.x() = imu_config_.bias.linear_acceleration.x;
    linear_acc_bias_.y() = imu_config_.bias.linear_acceleration.y;
    linear_acc_bias_.z() = imu_config_.bias.linear_acceleration.z;

    // parse odom config:
    private_nh_.param("pose/frame_id", odom_config_.frame_id, std::string("inertial"));
    private_nh_.param("pose/topic_name/ground_truth", odom_config_.topic_name.ground_truth, std::string("/pose/ground_truth"));
    private_nh_.param("pose/topic_name/estimation", odom_config_.topic_name.estimation, std::string("/pose/estimation"));

    odom_ground_truth_sub_ptr = std::make_shared<OdomSubscriber>(private_nh_, odom_config_.topic_name.ground_truth, 1000000);
    odom_estimation_pub_ = private_nh_.advertise<nav_msgs::Odometry>(odom_config_.topic_name.estimation, 500);
}

// 执行函数
bool Activity::Run(void) {
    // LOG(INFO)<<"1"<<std::endl;
    if (!ReadData())
        return false;
// LOG(INFO)<<"2"<<std::endl;
    while(HasData()) {
        if (UpdatePose()) { // 执行更新位姿
        // LOG(INFO)<<"3"<<std::endl;
            PublishPose();
        }
    }

    return true;
}

bool Activity::ReadData(void) {
    // fetch IMU measurements into buffer:
    imu_sub_ptr_->ParseData(imu_data_buff_); //数据存入buff deque变量

    if (static_cast<size_t>(0) == imu_data_buff_.size())
        return false;
// LOG(INFO)<<"22"<<std::endl;
    if (!initialized_) { // 第一次 or 其他
        odom_ground_truth_sub_ptr->ParseData(odom_data_buff_); // 里程计：存真值
// LOG(INFO)<<"3"<<std::endl;
        if (static_cast<size_t>(0) == odom_data_buff_.size())
            return false;
// LOG(INFO)<<"4"<<std::endl;
    }

    return true;
}

bool Activity::HasData(void) {
    if (imu_data_buff_.size() < static_cast<size_t>(3)) // imu buff中存入了值
        return false;

    if (
        !initialized_ && 
        static_cast<size_t>(0) == odom_data_buff_.size() // readdata中里程计buff 存入了值
    ) {
        return false;
    }

    return true;
}

template <typename T>
Eigen::Matrix<T, 3, 3> skew(Eigen::Matrix<T, 3, 1> &mat_in)
{
	Eigen::Matrix<T, 3, 3> skew_mat;
	skew_mat.setZero();
	skew_mat(0, 1) = -mat_in(2);
	skew_mat(0, 2) = mat_in(1);
	skew_mat(1, 2) = -mat_in(0);
	skew_mat(1, 0) = mat_in(2);
	skew_mat(2, 0) = -mat_in(1);
	skew_mat(2, 1) = mat_in(0);
	return skew_mat;
};

#define median_or_euler 0
#define R_or_q 0
/**
 * @brief 【导航姿态结算 误差分析】
 * 
 * @return true 
 * @return false 
 */
bool Activity::UpdatePose(void) {
    if (!initialized_) { // 初始化：
        // use the latest measurement for initialization:
        OdomData &odom_data = odom_data_buff_.back(); // last/new data
        IMUData imu_data = imu_data_buff_.back();

        pose_ = odom_data.pose;
        vel_ = odom_data.vel;

        initialized_ = true;

        odom_data_buff_.clear();
        imu_data_buff_.clear();

        // keep the latest IMU measurement for mid-value integration:
        imu_data_buff_.push_back(imu_data); // 为了中值积分
    } else {
        //
        // TODO: implement your estimation here
#if R_or_q == 1
    #if median_or_euler== 0
        LOG(INFO) << "[median - 法]" << std::endl;
        // get deltas:
        /* code */ // 【没有考虑bias等误差，需要再加上】【一些防御性判断条件等】
        Eigen::Matrix3d pose_pre,pose_cur; 
        pose_pre = pose_.block<3,3>(0,0);
        IMUData imu0 = imu_data_buff_.at(0);
        IMUData imu1 = imu_data_buff_.at(1);
        double delta_time = imu1.time - imu0.time;
        
        // update orientation:
        Eigen::Vector3d theta = (imu0.angular_velocity + imu1.angular_velocity) / 2 * delta_time;
        Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity() + sin(theta.norm()) / theta.norm() * skew(theta) + (1 - cos(theta.norm())) / (theta.norm() * theta.norm()) * skew(theta) * skew(theta);
        pose_cur = pose_pre * delta_R;
        pose_.block<3,3>(0,0) = pose_cur;
        // get velocity delta:
        Eigen::Vector3d vel_pre,delta_v;
        vel_pre = vel_;
        delta_v =  (pose_cur * imu1.linear_acceleration +  pose_pre*imu0.linear_acceleration)/2 - G_ ;
        vel_ = vel_pre + delta_v * delta_time;
        // update position:
        Eigen::Vector3d tran_pre,tran_cur;
        tran_pre = pose_.block<3,1>(0,3);
        tran_cur = tran_pre + vel_pre*delta_time + delta_v * delta_time * delta_time / 2; 
        pose_.block<3,1>(0,3) = tran_cur;
        // move forward --
    #elif median_or_euler==1
        LOG(INFO) << "[euler - 法]" << std::endl;
        // get deltas:
        Eigen::Matrix3d pose_pre,pose_cur;
        pose_pre = pose_.block<3,3>(0,0);
        IMUData imu0 = imu_data_buff_.at(0);
        IMUData imu1 = imu_data_buff_.at(1);
        double delta_time = imu1.time - imu0.time;
        // update orientation:
        Eigen::Vector3d theta = imu0.angular_velocity  * delta_time;
        Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity() + sin(theta.norm()) / theta.norm() * skew(theta) + (1 - cos(theta.norm())) / (theta.norm() * theta.norm()) * skew(theta) * skew(theta);
        pose_cur = pose_pre * delta_R;
        pose_.block<3,3>(0,0) = pose_cur;
        // get velocity delta:
        Eigen::Vector3d vel_pre,delta_v;
        vel_pre = vel_;
        delta_v = pose_pre*imu0.linear_acceleration - G_ ;
        vel_ = vel_pre + delta_v * delta_time;
        // update position:
        Eigen::Vector3d tran_pre,tran_cur;
        tran_pre = pose_.block<3,1>(0,3);
        tran_cur = tran_pre + vel_pre*delta_time + delta_v * delta_time * delta_time / 2; 
        pose_.block<3,1>(0,3) = tran_cur;
        // move forward -- 
    #endif
#elif R_or_q == 0
        // 每太注意，写完，才发现后面还有函数调用：【函数版本：简便很多了】
        Eigen::Matrix3d pose_pre,pose_cur;
        Eigen::Vector3d delta_q;

        if( !GetAngularDelta(1,0,delta_q) ){
            LOG(INFO)<< "GetAngularDelta  ERROR! "<<std::endl;
            return false;
        }
        UpdateOrientation(delta_q,pose_cur,pose_pre);

        Eigen::Vector3d delta_v;
        double delta_time;
        if( !GetVelocityDelta(1,0,pose_cur,pose_pre,delta_time,delta_v) ){
            LOG(INFO)<< "delta_v  ERROR! "<<std::endl;
            return false;
        }
        UpdatePosition( delta_time, delta_v);

#endif
        // NOTE: this is NOT fixed. you should update your buffer according to the method of your choice:
        imu_data_buff_.pop_front();
    }
    
    return true;
}


bool Activity::PublishPose() {
    // a. set header:
    message_odom_.header.stamp = ros::Time::now();
    message_odom_.header.frame_id = odom_config_.frame_id;
    
    // b. set child frame id:
    message_odom_.child_frame_id = odom_config_.frame_id;

    // b. set orientation:
    Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));
    message_odom_.pose.pose.orientation.x = q.x();
    message_odom_.pose.pose.orientation.y = q.y();
    message_odom_.pose.pose.orientation.z = q.z();
    message_odom_.pose.pose.orientation.w = q.w();

    // c. set position:
    Eigen::Vector3d t = pose_.block<3, 1>(0, 3);
    message_odom_.pose.pose.position.x = t.x();
    message_odom_.pose.pose.position.y = t.y();
    message_odom_.pose.pose.position.z = t.z();  

    // d. set velocity:
    message_odom_.twist.twist.linear.x = vel_.x();
    message_odom_.twist.twist.linear.y = vel_.y();
    message_odom_.twist.twist.linear.z = vel_.z(); 

    odom_estimation_pub_.publish(message_odom_);

    return true;
}

/**
 * @brief  get unbiased angular velocity in body frame
 * @param  angular_vel, angular velocity measurement
 * @return unbiased angular velocity in body frame
 */
inline Eigen::Vector3d Activity::GetUnbiasedAngularVel(const Eigen::Vector3d &angular_vel) {
    return angular_vel - angular_vel_bias_;
}

/**
 * @brief  get unbiased linear acceleration in navigation frame
 * @param  linear_acc, linear acceleration measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased linear acceleration in navigation frame
 */
inline Eigen::Vector3d Activity::GetUnbiasedLinearAcc(
    const Eigen::Vector3d &linear_acc,
    const Eigen::Matrix3d &R
) {
    return R*(linear_acc - linear_acc_bias_) - G_;
}

/**
 * @brief  get angular delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  angular_delta, angular delta output
 * @return true if success false otherwise
 */
bool Activity::GetAngularDelta(
    const size_t index_curr, const size_t index_prev,
    Eigen::Vector3d &angular_delta
) {
    //
    // TODO: this could be a helper routine for your own implementation
    //
    if (
        index_curr <= index_prev ||
        imu_data_buff_.size() <= index_curr
    ) {
        return false;
    }

    const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

    double delta_t = imu_data_curr.time - imu_data_prev.time;

    Eigen::Vector3d angular_vel_curr = GetUnbiasedAngularVel(imu_data_curr.angular_velocity);
    Eigen::Vector3d angular_vel_prev = GetUnbiasedAngularVel(imu_data_prev.angular_velocity);

#if median_or_euler == 0
    angular_delta = 0.5*delta_t*(angular_vel_curr + angular_vel_prev);
#elif median_or_euler == 1
    angular_delta = delta_t * angular_vel_prev;
#endif
    return true;
}

/**
 * @brief  get velocity delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  R_curr, corresponding orientation of current imu measurement
 * @param  R_prev, corresponding orientation of previous imu measurement
 * @param  velocity_delta, velocity delta output
 * @return true if success false otherwise
 */
bool Activity::GetVelocityDelta(
    const size_t index_curr, const size_t index_prev,
    const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, 
    double &delta_t, Eigen::Vector3d &velocity_delta
) {
    //
    // TODO: this could be a helper routine for your own implementation
    //
    if (
        index_curr <= index_prev ||
        imu_data_buff_.size() <= index_curr
    ) {
        return false;
    }

    const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

    delta_t = imu_data_curr.time - imu_data_prev.time;

    Eigen::Vector3d linear_acc_curr = GetUnbiasedLinearAcc(imu_data_curr.linear_acceleration, R_curr);
    Eigen::Vector3d linear_acc_prev = GetUnbiasedLinearAcc(imu_data_prev.linear_acceleration, R_prev);

#if median_or_euler == 0
    velocity_delta = 0.5 * delta_t * (linear_acc_curr + linear_acc_prev); // 速度更新量
#elif median_or_euler == 1
    velocity_delta = delta_t * linear_acc_prev;
#endif

    return true;
}

/**
 * @brief  update orientation with effective rotation angular_delta
 * @param  angular_delta, effective rotation
 * @param  R_curr, current orientation
 * @param  R_prev, previous orientation
 * @return void
 */
void Activity::UpdateOrientation(
    const Eigen::Vector3d &angular_delta,
    Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev
) {
    //
    // TODO: this could be a helper routine for your own implementation
    //
    // magnitude:
    double angular_delta_mag = angular_delta.norm(); //大小
    // direction:
    Eigen::Vector3d angular_delta_dir = angular_delta.normalized(); //归一化

    // build delta q: || 旋转向量转换成四元数
    double angular_delta_cos = cos(angular_delta_mag/2.0);
    double angular_delta_sin = sin(angular_delta_mag/2.0);
    Eigen::Quaterniond dq(
        angular_delta_cos, 
        angular_delta_sin*angular_delta_dir.x(), 
        angular_delta_sin*angular_delta_dir.y(), 
        angular_delta_sin*angular_delta_dir.z()
    );
    Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));
    
    // update: || 基于四元数的更新
    q = q*dq;
    
    // write back:
    R_prev = pose_.block<3, 3>(0, 0); // 上一次的旋转 R
    pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    R_curr = pose_.block<3, 3>(0, 0); // 更新之后的旋转 R
}

/**
 * @brief  update orientation with effective velocity change velocity_delta
 * @param  delta_t, timestamp delta 
 * @param  velocity_delta, effective velocity change
 * @return void
 */
void Activity::UpdatePosition(const double &delta_t, const Eigen::Vector3d &velocity_delta) {
    //
    // TODO: this could be a helper routine for your own implementation
    //
    pose_.block<3, 1>(0, 3) += delta_t*vel_ + 0.5*delta_t*velocity_delta; // 位置
    vel_ += velocity_delta; // 速度
}


} // namespace estimator

} // namespace imu_integration