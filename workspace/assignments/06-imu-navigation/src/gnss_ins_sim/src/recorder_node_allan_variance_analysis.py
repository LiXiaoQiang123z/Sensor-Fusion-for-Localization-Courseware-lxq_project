#!/usr/bin/python 
# -*- coding: utf-8 -*-
import os

import rospkg
import rospy
import rosbag

import math
import numpy as np

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# 步骤
# １．定义ＩＭＵ模型
    # 1.1 定义IMU误差模型：中等精度的imu模型【低、中、高 + 手动】
    # 1.2 创建IMU对象：9轴（6/9轴），gnss=true（false/true）
# 2.创建运行配置文件： 2.1：修改yaml文件中output文件路径 2.2修改motion_def 下的运动配置文件参数： 熟练使用类型1~5
    # ch6要求：静止、匀速、加减速、快速转弯 || 
    # 类型1： 类型2：绝对姿态和绝对速度 类型3：姿态变化和速度变化  类型4：绝对姿态和速度变化 类型5：姿态变化和绝对速度
    # 【对应方法】静止：类型1-变量为零即可 ； 匀速：类型5/2：绝对速度 ； 加减速：类型4/3:：速度变化 ； 快速转弯：类型3/5：姿态变化
# 3.创建算法
    # algo = allan_analysis.Allan() # an Allan analysis demo algorithm
    # def run(self, set_of_input):
    # def get_results(self):
# 4.运行模拟过程：
    # 4.1 创建模拟对象
    # 4.2 运行模拟
# 5.显示结果
    # sim.results('./data/')  保存文件到指定路径 || or sim.results() 不保存结果
    # sim.plot(['ref_pos', 'gyro'], opt={'ref_pos': '3d'}) 绘制数据

def get_gnss_ins_sim(motion_def_file, fs_imu, fs_gps):
    '''
    Generate simulated GNSS/IMU data using specified trajectory.
    '''
    # set IMU model:
    D2R = math.pi/180.0
    # imu_err = 'low-accuracy'
    # 1.定义IMU模型
    imu_err = { #1.1 定义imu误差模型
        # 1. gyro:
        # a. random noise:
        # gyro angle random walk, deg/rt-hr
        'gyro_arw': np.array([0.75, 0.75, 0.75]),
        # gyro bias instability, deg/hr 
        'gyro_b_stability': np.array([10.0, 10.0, 10.0]),
        # gyro bias isntability correlation time, sec
        'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error: || 确定性误差
        'gyro_b': np.array([0.0, 0.0, 0.0]),
        'gyro_k': np.array([1.0, 1.0, 1.0]),
        'gyro_s': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        # 2. accel:
        # a. random noise: || 噪声
        # accel velocity random walk, m/s/rt-hr
        'accel_vrw': np.array([0.05, 0.05, 0.05]),
        # accel bias instability, m/s2
        'accel_b_stability': np.array([2.0e-4, 2.0e-4, 2.0e-4]),
        # accel bias isntability correlation time, sec
        'accel_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
        'accel_k': np.array([1.0, 1.0, 1.0]),
        'accel_s': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        # 3. mag: ||磁力计
        'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0, 
        'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
        'mag_std': np.array([0.1, 0.1, 0.1])
    }
    # generate GPS and magnetometer data:　｜｜　1.2 创建imu对象
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    # 4.运行模拟过程：
    # init simulation: || 4.1 创建模拟对象
    sim = ins_sim.Sim(
        [fs_imu, fs_gps, fs_imu],
        motion_def_file,
        ref_frame=1,
        imu=imu,
        mode=None,
        env=None,
        algorithm=None
    )
    
    # run: // 4.2运行模拟
    sim.run(1)

    # get simulated data:
    rospy.logwarn(
        "Simulated data size {}".format(
            len(sim.dmgr.get_data_all('gyro').data[0])
        )
    )

    # imu measurements:
    step_size = 1.0 / fs_imu
    for i, (gyro, accel) in enumerate(
        zip(
            # a. gyro
            sim.dmgr.get_data_all('gyro').data[0], 
            # b. accel
            sim.dmgr.get_data_all('accel').data[0]
        )
    ):
        yield {
            'stamp': i * step_size,
            'data': {
                # a. gyro:
                'gyro_x': gyro[0],
                'gyro_y': gyro[1],
                'gyro_z': gyro[2],
                # b. accel:
                'accel_x': accel[0],
                'accel_y': accel[1],
                'accel_z': accel[2]
            }
        }
    # sim.results("/home/lxq/dataset/catkin_multi/shnelan-multi-sensor-fusion/Sensor_multi_fusion_DEBUG/workspace-ch6/src/9_data/allan_variance_analysis")
    sim.results()
    sim.plot(['pos', 'vel'], opt={'pos': '3d'})

def gnss_ins_sim_recorder():
    """
    Record simulated GNSS/IMU data as ROS bag
    """
    # ensure gnss_ins_sim_node is unique:
    rospy.init_node('gnss_ins_sim_recorder_node')

    # parse params:
    motion_def_name = rospy.get_param('/gnss_ins_sim_recorder_node/motion_file')
    sample_freq_imu = rospy.get_param('/gnss_ins_sim_recorder_node/sample_frequency/imu')
    sample_freq_gps = rospy.get_param('/gnss_ins_sim_recorder_node/sample_frequency/gps')
    topic_name_imu = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name')
    rosbag_output_path = rospy.get_param('/gnss_ins_sim_recorder_node/output_path')
    rosbag_output_name = rospy.get_param('/gnss_ins_sim_recorder_node/output_name')

    # generate simulated data:
    motion_def_path = os.path.join(
        rospkg.RosPack().get_path('gnss_ins_sim'), 'config', 'motion_def', motion_def_name
    )
    imu_simulator = get_gnss_ins_sim(
        # motion def file:
        motion_def_path,
        # gyro-accel/gyro-accel-mag sample rate:
        sample_freq_imu,
        # GPS sample rate:
        sample_freq_gps
    )

    with rosbag.Bag(
        os.path.join(rosbag_output_path, rosbag_output_name), 'w'
    ) as bag:
        # get timestamp base:
        timestamp_start = rospy.Time.now()

        for measurement in imu_simulator:
            # init:
            msg = Imu()
            # a. set header:
            msg.header.frame_id = 'NED'
            msg.header.stamp = timestamp_start + rospy.Duration.from_sec(measurement['stamp'])
            # b. set orientation estimation:
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0
            # c. gyro:
            msg.angular_velocity.x = measurement['data']['gyro_x']
            msg.angular_velocity.y = measurement['data']['gyro_y']
            msg.angular_velocity.z = measurement['data']['gyro_z']
            msg.linear_acceleration.x = measurement['data']['accel_x']
            msg.linear_acceleration.y = measurement['data']['accel_y']
            msg.linear_acceleration.z = measurement['data']['accel_z']

            # write:
            bag.write(topic_name_imu, msg, msg.header.stamp)

if __name__ == '__main__':
    try:
        gnss_ins_sim_recorder()
    except rospy.ROSInterruptException:
        pass
