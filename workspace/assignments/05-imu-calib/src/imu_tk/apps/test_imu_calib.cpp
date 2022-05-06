/*
 * @Author: LiXiaoQiang123 1724946681@qq.com
 * @Date: 2022-05-04 22:40:25
 * @LastEditors: LiXiaoQiang123 1724946681@qq.com
 * @LastEditTime: 2022-05-05 21:31:51
 * @FilePath: /Sensor-Fusion-for-Localization-Courseware-lxq_project/workspace/assignments/05-imu-calib/src/imu_tk/apps/test_imu_calib.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEim
 */
#include <iostream>

#include "imu_tk/io_utils.h"
#include "imu_tk/calibration.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"
#include "imu_tk/visualization.h"

using namespace std;
using namespace imu_tk;
using namespace Eigen;

/**
 * @brief 低成本IMU MEMS 内参误差标定
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
  if( argc < 3 )
    return -1;

  vector< TriadData > acc_data, gyro_data;
  
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[1]<<endl;  
  importAsciiData( argv[1], acc_data, imu_tk::TIMESTAMP_UNIT_SEC );
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[2]<<endl;  
  importAsciiData( argv[2], gyro_data, imu_tk::TIMESTAMP_UNIT_SEC  );
  
  
  CalibratedTriad init_acc_calib, init_gyro_calib; // 校准acc gyro类
  init_acc_calib.setBias(  // 初始化：bias数值，是静止时候的数据集
    Vector3d(32768, 32768, 32768) 
  );
  init_gyro_calib.setScale( 
    Vector3d(1.0/6258.0, 1.0/6258.0, 1.0/6258.0) 
  );
  
  MultiPosCalibration mp_calib; // 估计TKB的类
    
  mp_calib.setInitStaticIntervalDuration(50.0); // 初始静态时间间隔 Tinit吗
  mp_calib.setInitAccCalibration( init_acc_calib );
  mp_calib.setInitGyroCalibration( init_gyro_calib );  
  mp_calib.setGravityMagnitude(9.81744); // 重力的大小
  mp_calib.enableVerboseOutput(true); // 详细输出
  mp_calib.enableAccUseMeans(false); // 加速度均值-否
  //mp_calib.setGyroDataPeriod(0.01); // 角速度先验
  mp_calib.calibrateAccGyro(acc_data, gyro_data ); // 【关键】对齐加速度和角速度
  mp_calib.getAccCalib().save("test_imu_acc.calib"); // 保存结果
  mp_calib.getGyroCalib().save("test_imu_gyro.calib");
  
//   for( int i = 0; i < acc_data.size(); i++)
//   {
//     cout<<acc_data[i].timestamp()<<" "
  //         <<acc_data[i].x()<<" "<<acc_data[i].y()<<" "<<acc_data[i].z()<<" "
  //         <<gyro_data[i].x()<<" "<<gyro_data[i].y()<<" "<<gyro_data[i].z()<<endl;
//   }
//   cout<<"Read "<<acc_data.size()<<" tuples"<<endl;
  
  return 0;
}