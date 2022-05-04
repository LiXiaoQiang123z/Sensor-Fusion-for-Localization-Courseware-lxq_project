/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

using namespace lidar_localization;
// 没用到(⊙o⊙)…
int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "test_frame_node");
    ros::NodeHandle nh;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", "/map", 100);
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "/map", "/lidar", 100);

    std::deque<CloudData> cloud_data_buff;
    std::deque<IMUData> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;
    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
    bool transform_received = false;
    bool gnss_origin_position_inited = false;

    ros::Rate rate(100); // 100HZ
    while (ros::ok()) {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff); // 1. 新的数据插入到data_buff中
        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        if (!transform_received) { //2.获取lidar to imu 的位姿变换
            if (lidar_to_imu_ptr->LookupData(lidar_to_imu)) {
                transform_received = true;
                // LOG(INFO) << "lidar to imu transform matrix is:" << std::endl << lidar_to_imu;
            }
        } else {
            while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0) {
                CloudData cloud_data = cloud_data_buff.front();
                IMUData imu_data = imu_data_buff.front();
                GNSSData gnss_data = gnss_data_buff.front();

                double d_time = cloud_data.time - imu_data.time; // 时间戳对齐
                if (d_time < -0.05) {
                    cloud_data_buff.pop_front();
                } else if (d_time > 0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                } else {
                    cloud_data_buff.pop_front();
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();

                    Eigen::Matrix4f odometry_matrix;

                    if (!gnss_origin_position_inited) { // gnss数据初始化：获取第一帧的经纬度、海拔
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
                    }
                    gnss_data.UpdateXYZ(); // 经纬度、海拔坐标系转换->ENU局部笛卡尔坐标系：平移位置
                    odometry_matrix(0,3) = gnss_data.local_E;
                    odometry_matrix(1,3) = gnss_data.local_N;
                    odometry_matrix(2,3) = gnss_data.local_U;
                    odometry_matrix.block<3,3>(0,0) = imu_data.GetOrientationMatrix();// 四元数转换为旋转矩阵float类型：旋转位姿
                    odometry_matrix *= lidar_to_imu; // 将里程计位姿矩阵从imu旋转到lader坐标系下。【位姿的观测值】

                    pcl::transformPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, odometry_matrix); // 点云旋转：lader系下相邻帧直接旋转

                    cloud_pub_ptr->Publish(cloud_data.cloud_ptr); // 发布
                    odom_pub_ptr->Publish(odometry_matrix);
                }
            }
        }

        rate.sleep();
    }

    return 0;
}