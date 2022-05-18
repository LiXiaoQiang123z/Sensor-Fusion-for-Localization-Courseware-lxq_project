#!/usr/bin/python 
# -*- coding: utf-8 -*-

import os

import rospkg
import rospy
import rosbag

import math
import numpy as np
import pandas as pd

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

def get_gnss_ins_sim(motion_def_file, fs_imu, fs_gps):
    '''
    Generate simulated GNSS/IMU data using specified trajectory.
    '''
    #  TODO:set  origin  x y z
    origin_x =  2849886.618251
    origin_y =  -4656214.272942
    origin_z =  -3287190.600463
    # set IMU model:
    D2R = math.pi/180.0
    # imu_err = 'low-accuracy'
    imu_err = {
        # 1. gyro:
        # a. random noise:
        # gyro angle random walk, deg/rt-hr
        # 'gyro_arw': np.array([0.75, 0.75, 0.75]),
        'gyro_arw': np.array([0.00, 0.00, 0.00]),
        # gyro bias instability, deg/hr
        # 'gyro_b_stability': np.array([10.0, 10.0, 10.0]),
        'gyro_b_stability': np.array([0.0, 0.0, 0.0]),
        # gyro bias isntability correlation time, sec
        # 'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        # 'gyro_b': np.array([36.00, 36.00, 36.00]),
        # 'gyro_k': np.array([0.98, 0.98, 0.98]),
        # 'gyro_s': np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]),
        'gyro_b': np.array([0.0, 0.0, 0.0]),
        'gyro_k': np.array([1.0, 1.0, 1.0]),
        'gyro_s': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        # 2. accel:
        # a. random noise:
        # accel velocity random walk, m/s/rt-hr
        # 'accel_vrw': np.array([0.05, 0.05, 0.05]),
        'accel_vrw': np.array([0.00, 0.00, 0.00]),
        # accel bias instability, m/s2
        # 'accel_b_stability': np.array([2.0e-4, 2.0e-4, 2.0e-4]),
        'accel_b_stability': np.array([0.0, 0.0, 0.0]),
        # accel bias isntability correlation time, sec
        # 'accel_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        # 'accel_b': np.array([0.01, 0.01, 0.01]),
        # 'accel_k': np.array([0.98, 0.98, 0.98]),
        # 'accel_s': np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]),
        'accel_b': np.array([0.0, 0.0, 0.0]),
        'accel_k': np.array([1.0, 1.0, 1.0]),
        'accel_s': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        # 3. mag:
        'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0, 
        'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
        'mag_std': np.array([0.1, 0.1, 0.1])
    }
    # generate GPS and magnetometer data:
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    # init simulation:
    sim = ins_sim.Sim(
        # here sync GPS with other measurements as marker:
        [fs_imu, fs_imu, fs_imu],
        motion_def_file,
        ref_frame=1,
        imu=imu,
        mode=None,
        env=None,
        algorithm=None
    )
    
    # run:
    sim.run(1)

    # odom 中对应的参数：   ref_att_quat   ref_pos  ref_vel

    # get simulated data:
    rospy.logwarn(
        'Simulated data size: Gyro-{}, Accel-{}, ref_att_quat-{},ref_pos-{},ref_vel-{}'.format(
            len(sim.dmgr.get_data_all('gyro').data[0]),
            len(sim.dmgr.get_data_all('accel').data[0]),
            # len(sim.dmgr.get_data_all('gps_visibility').data)
            len(sim.dmgr.get_data_all('ref_att_quat').data),
            len(sim.dmgr.get_data_all('ref_pos').data),
            len(sim.dmgr.get_data_all('ref_vel').data)
        )
    )

    # calibration stages:
    STEP_SIZE = 1.0 / fs_imu

    # STAGES = [
    #     'rotate_z_pos', 'rotate_z_neg', 'rotate_y_pos', 'rotate_y_neg', 'rotate_x_pos', 'rotate_x_neg',
    #     'static_z_pos', 'static_z_neg', 'static_y_pos', 'static_y_neg', 'static_x_pos', 'static_x_neg'
    # ]
    # stage_id = 0

    # last_gps_visibility = True 
    # curr_gps_visibility = True
    for i, (gyro, accel, ref_q, ref_t_gt,ref_v_gt) in enumerate(
        zip(
            # a. gyro:
            sim.dmgr.get_data_all('gyro').data[0], 
            # b. accel:
            sim.dmgr.get_data_all('accel').data[0],

            # odom:1.四元数 2.真实位置 3.真实速度 || PVQ
            sim.dmgr.get_data_all('ref_att_quat').data, 
            sim.dmgr.get_data_all('ref_pos').data, 
            sim.dmgr.get_data_all('ref_vel').data
            # e. gps visibility as marker:
            # sim.dmgr.get_data_all('gps_visibility').data
        )
    ):  
        # last_gps_visibility = curr_gps_visibility
        # curr_gps_visibility = gps_visibility

        # if (not last_gps_visibility) and curr_gps_visibility:
        #     stage_id += 1
        
        # if not curr_gps_visibility:
        #     continue

        yield {
            'stamp': i * STEP_SIZE,
            # a. gyro:
            'gyro_x': gyro[0],
            'gyro_y': gyro[1],
            'gyro_z': gyro[2],
            # b. accel:
            'accel_x': accel[0],
            'accel_y': accel[1],
            'accel_z': accel[2],

            # odom:1.四元数 2.真实位置 3.真实速度 || PVQ
            'ref_q_w': ref_q[0],
            'ref_q_x': ref_q[1],
            'ref_q_y': ref_q[2],
            'ref_q_z': ref_q[3],

            # 'ref_pos_x': ref_t_gt[0],
            # 'ref_pos_y': ref_t_gt[1],
            # 'ref_pos_z': ref_t_gt[2],
            'ref_pos_x': ref_t_gt[0] + origin_x,
            'ref_pos_y': ref_t_gt[1] + origin_y,
            'ref_pos_z': ref_t_gt[2] + origin_z,
            
            'ref_vel_x': ref_v_gt[0],
            'ref_vel_y': ref_v_gt[1],
            'ref_vel_z': ref_v_gt[2],
            # # e. stage:
            # 'stage': STAGES[stage_id]
        }
    sim.results()
    sim.plot(['ref_pos', 'ref_vel'], opt={'ref_pos': '3d'})


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
    topic_name_imu = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name_imu')
    topic_name_gt = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name_gt') #new

    # # scv save path
    # output_path = rospy.get_param('/gnss_ins_sim_recorder_node/output_path')
    # output_name = rospy.get_param('/gnss_ins_sim_recorder_node/output_name')

    # bag save path [new]
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

    # # write as csv:
    # data = pd.DataFrame(
    #     list(imu_simulator)
    # )
    # data.to_csv(
    #     os.path.join(output_path, output_name)
    # )

    # 参考allan 保存成rosbag
    with rosbag.Bag(
        os.path.join(rosbag_output_path, rosbag_output_name), 'w'
    ) as bag:
        # get timestamp base:
        timestamp_start = rospy.Time.now()

        for measurement in imu_simulator:
            # init:
            msg_imd = Imu()
            # a. set header:
            msg_imd.header.frame_id = 'inertial'
            msg_imd.header.stamp = timestamp_start + \
                rospy.Duration.from_sec(measurement['stamp'])
            # b. set orientation estimation:
            msg_imd.orientation.x = 0.0
            msg_imd.orientation.y = 0.0
            msg_imd.orientation.z = 0.0
            msg_imd.orientation.w = 1.0
            # c. gyro:
            msg_imd.angular_velocity.x = measurement['gyro_x']
            msg_imd.angular_velocity.y = measurement['gyro_y']
            msg_imd.angular_velocity.z = measurement['gyro_z']
            msg_imd.linear_acceleration.x = measurement['accel_x']
            msg_imd.linear_acceleration.y = measurement['accel_y']
            msg_imd.linear_acceleration.z = measurement['accel_z']

            # write:
            bag.write(topic_name_imu, msg_imd, msg_imd.header.stamp)


            # init: odom [new] 参考手动实现imu的程序
            msg_odom = Odometry()
            # a. set header:
            msg_odom.header.frame_id = 'inertial'
            msg_odom.header.stamp = msg_imd.header.stamp
            # b. set orientation estimation:
            msg_odom.pose.pose.orientation.x = measurement['ref_q_x']
            msg_odom.pose.pose.orientation.y = measurement['ref_q_y']
            msg_odom.pose.pose.orientation.z = measurement['ref_q_z']
            msg_odom.pose.pose.orientation.w = measurement['ref_q_w']
            # c. gyro:
            msg_odom.pose.pose.position.x = measurement['ref_pos_x']
            msg_odom.pose.pose.position.y = measurement['ref_pos_y']
            msg_odom.pose.pose.position.z = measurement['ref_pos_z']
            msg_odom.twist.twist.linear.x = measurement['ref_vel_x']
            msg_odom.twist.twist.linear.y = measurement['ref_vel_y']
            msg_odom.twist.twist.linear.z = measurement['ref_vel_z']

            # write:
            bag.write(topic_name_gt, msg_odom, msg_odom.header.stamp)

if __name__ == '__main__':
    try:
        gnss_ins_sim_recorder()
    except rospy.ROSInterruptException:
        pass
