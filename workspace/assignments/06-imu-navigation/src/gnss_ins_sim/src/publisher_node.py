#!/usr/bin/python 
# -*- coding: utf-8 -*-

import os

import rospkg
import rospy

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
    #### choose a built-in IMU model, typical for IMU381 ||　１．定义ＩＭＵ模型
    imu_err = 'mid-accuracy'  # 1.1 定义IMU误差模型：中等精度的imu模型【低、中、高 + 手动】
    # generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True) # 1.2 创建IMU对象：9轴（6/9轴），gnss=true（false/true）

    # 2.创建运行配置文件： 2.1：修改yaml文件中output文件路径 2.2修改motion_def 下的运动配置文件参数： 熟练使用类型1~5
    # ch6要求：静止、匀速、加减速、快速转弯 || 
    # 类型1： 类型2：绝对姿态和绝对速度 类型3：姿态变化和速度变化  类型4：绝对姿态和速度变化 类型5：姿态变化和绝对速度
    # 【对应方法】静止：类型1-变量为零即可 ； 匀速：类型5/2：绝对速度 ； 加减速：类型4/3:：速度变化 ； 快速转弯：类型3/5：姿态变化
    
    # 3.创建算法
    # algo = allan_analysis.Allan() # an Allan analysis demo algorithm
    # def run(self, set_of_input):
    # def get_results(self):

    # 4.运行模拟过程：
    # init simulation: || 4.1 创建模拟对象
    sim = ins_sim.Sim(
        [fs_imu, fs_gps, fs_imu], #imu 陀螺仪 加速度计算 gps 磁力计等采样频率
        motion_def_file, # 初始条件和运动定义=配置文件
        ref_frame=1, # 参考帧
        imu=imu,# 创建imu对象
        mode=None, # 车辆激动能力
        env=None, # imu振动模型
        algorithm=None # 创建算法对象，这里false
    )

    # run: || 4.2 运行模拟
    sim.run(1) #运行1次

    ###########################################
    # get simulated data: || 获取模拟数据
    rospy.logwarn(
        "Simulated data size {}".format(
            len(sim.dmgr.get_data_all('gyro').data[0])
        )
    )

    # imu measurements: || 
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

    # # plot interested data
    # sim.plot(['ref_pos', 'gyro'], opt={'ref_pos': '3d'})


def gnss_ins_sim_publisher():
    """
    Publish simulated GNSS/IMU data
    """
    
    # ensure gnss_ins_sim_node is unique:
    rospy.init_node('gnss_ins_sim_node')
    
    # parse params:
    motion_def_name = 'motion_def-3d.csv' #rospy.get_param('motion_file')
    sample_freq_imu = 100.0               #rospy.get_param('sample_frequency/imu')
    sample_freq_gps = 10.0                #rospy.get_param('sample_frequency/gps')
    topic_name_imu = '~sim/sensor/imu'    #rospy.get_param('topic_name')

    pub = rospy.Publisher(topic_name_imu, Imu, queue_size=1000000)
    
    # generate simulated data:
    motion_def_path = os.path.join(
        rospkg.RosPack().get_path('gnss_ins_sim'), 'config', 'motion_def', motion_def_name
    )
    imu_simulator = get_gnss_ins_sim( #执行gnss ins sim 算法过程
        # motion def file:
        motion_def_path,
        # gyro-accel/gyro-accel-mag sample rate:
        sample_freq_imu,
        # GPS sample rate:
        sample_freq_gps
    )

    rate = rospy.Rate( 
        int(sample_freq_imu)
    ) # 100 Hz || 频率获取
    while not rospy.is_shutdown():
        # get measurement:
        try:
            measurement = next(imu_simulator)
        except StopIteration:
            break

        # init:
        msg = Imu()
        # a. set header:
        msg.header.frame_id = 'NED'
        msg.header.stamp = rospy.Time.now()
        # b. set orientation estimation:
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        # c. gyro:
        msg.linear_acceleration.x = measurement['data']['gyro_x']
        msg.linear_acceleration.y = measurement['data']['gyro_y']
        msg.linear_acceleration.z = measurement['data']['gyro_z']
        msg.angular_velocity.x = measurement['data']['accel_x']
        msg.angular_velocity.y = measurement['data']['accel_y']
        msg.angular_velocity.z = measurement['data']['accel_z']
        # finally:
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        gnss_ins_sim_publisher()
    except rospy.ROSInterruptException:
        pass
