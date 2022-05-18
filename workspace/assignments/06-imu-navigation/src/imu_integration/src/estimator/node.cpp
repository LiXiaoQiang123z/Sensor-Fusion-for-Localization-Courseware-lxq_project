/*
 * @Author: LiXiaoQiang123 1724946681@qq.com
 * @Date: 2022-05-13 16:47:51
 * @LastEditors: LiXiaoQiang123 1724946681@qq.com
 * @LastEditTime: 2022-05-14 17:19:19
 * @FilePath: /workspace-ch6/src/imu_integration/src/estimator/node.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include "imu_integration/estimator/activity.hpp"

int main(int argc, char** argv) {

    std::string node_name{"imu_integration_estimator_node"};
    ros::init(argc, argv, node_name);
    
    imu_integration::estimator::Activity activity;

    activity.Init();
    
    // 100 Hz:
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        activity.Run();

        loop_rate.sleep();
    } 

    return EXIT_SUCCESS;
}