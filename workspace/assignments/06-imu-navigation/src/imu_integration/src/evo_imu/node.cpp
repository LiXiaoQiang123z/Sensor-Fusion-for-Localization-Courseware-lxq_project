/*
 * @Author: LiXiaoQiang123 1724946681@qq.com
 * @Date: 2022-05-13 16:47:51
 * @LastEditors: LiXiaoQiang123 1724946681@qq.com
 * @LastEditTime: 2022-05-15 16:54:44
 * @FilePath: /workspace-ch6/src/imu_integration/src/estimator/node.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <fstream>
#include <iostream> // new

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "imu_integration/config/config.hpp"

using namespace std;

struct pose{
    double time = 0.0;
    Eigen::Quaterniond q;
    Eigen::Vector3d trans;
};

pose estim_pose;
pose gt_pose;

std::ofstream esti_data;
std::ofstream gt_data;

// callback init
double  stamp_gt_init = 0;
double  stamp_ins_init = 0;

int  flag_gt = 1;
int  flag_ins = 1;

bool CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path, std::ios::out); //  使用std::ios::out 可实现覆盖
    if(!ofs)
    {
        std::cout << "open csv file error " << std::endl;
        return  false;
    }
    return true;
}

void WriteText(std::ofstream& ofs, pose data){
    ofs << std::fixed << data.time << " " << data.trans.x() << " " << data.trans.y() << " " << data.trans.z() << " "
        << data.q.x() << " " << data.q.y() << " " << data.q.z() << " " << data.q.w() << std::endl;
}

void esti_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (flag_ins)
    {
        stamp_ins_init = msg->header.stamp.toSec();
        flag_ins = 0;
    }
    estim_pose.time = msg->header.stamp.toSec() - stamp_ins_init; //把时间戳转化成浮点型格式

    estim_pose.trans.x() = msg->pose.pose.position.x;
    estim_pose.trans.y() = msg->pose.pose.position.y;
    estim_pose.trans.z() = msg->pose.pose.position.z;

    estim_pose.q.w() = msg->pose.pose.orientation.w;
    estim_pose.q.x() = msg->pose.pose.orientation.x;
    estim_pose.q.y() = msg->pose.pose.orientation.y;
    estim_pose.q.z() = msg->pose.pose.orientation.z;

    WriteText(esti_data, estim_pose);
}

void gt_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (flag_gt)
    {
        stamp_gt_init = msg->header.stamp.toSec();
        flag_gt = 0;
    }
    gt_pose.time = msg->header.stamp.toSec() - stamp_gt_init; //把时间戳转化成浮点型格式

    gt_pose.trans.x() = msg->pose.pose.position.x;
    gt_pose.trans.y() = msg->pose.pose.position.y;
    gt_pose.trans.z() = msg->pose.pose.position.z;

    gt_pose.q.w() = msg->pose.pose.orientation.w;
    gt_pose.q.x() = msg->pose.pose.orientation.x;
    gt_pose.q.y() = msg->pose.pose.orientation.y;
    gt_pose.q.z() = msg->pose.pose.orientation.z;

    WriteText(gt_data, gt_pose);
}

int main(int argc, char** argv) {

    char *esti_path = "/home/lxq/dataset/catkin_multi/shnelan-multi-sensor-fusion/Sensor_multi_fusion_DEBUG/workspace-ch6/src/9_data/imu_integration/data_esti.txt";
    char *gt_path = "/home/lxq/dataset/catkin_multi/shnelan-multi-sensor-fusion/Sensor_multi_fusion_DEBUG/workspace-ch6/src/9_data/imu_integration/data_gt.txt";

    CreateFile(esti_data, esti_path);
    CreateFile(gt_data, gt_path);

    std::string node_name{"evo_imu_node"};
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Subscriber sub_esti = nh.subscribe("/pose/estimation",10,esti_callback);
    ros::Subscriber sub_gt = nh.subscribe("/pose/ground_truth",10,gt_callback);
    // 100 Hz:
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    } 
    esti_data.close();
    gt_data.close();
    return EXIT_SUCCESS;
}
