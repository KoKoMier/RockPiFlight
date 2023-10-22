#ifndef QUATERNION_H
#define QUATERNION_H

#include <iostream>
#include <cmath>


#define sampleFreq	500.f			// sample frequency in Hz
/*采样周期的一半，用于求解四元数微分方程时计算角增量
请确定自己的姿态调用周期: 10ms,即上面的sampleFreq: 100Hz*/
#define halfT 0.0092f
 
//这里的Kp,Ki是用于控制加速度计修正陀螺仪积分姿态的速度
#define Kp 2.0f  //2.0f
#define Ki 0.002f  //0.002f
 
 

static float q0 = 1, q1 = 0, q2 = 0, q3 = 0; //最优估计四元数
//定义姿态解算误差的积分.     
//当前加计测得的重力加速度在三轴(x,y,z)上的分量,与当前姿态计算得来的重力在三轴上的分量的误差的积分
static float xErrorInt = 0, yErrorInt = 0, zErrorInt = 0;
 
/*
 * 6轴陀螺仪姿态融合算法:  Mahony的互补滤波算法 6轴版
 * 单位: m/s^2   rad/s
 * 由于加速度的噪音很大, 此处建议使用滤波后的数据
 * */
 
 
 
 
void MahonyImuUpdate(float gx, float gy, float gz, float ax, float ay, float az, float *roll, float *pitch, float *yaw)//g表陀螺仪，a表加计
{
    float q0temp,q1temp,q2temp,q3temp;
    float norm; //矢量的模或四元数的范数
    float posture_x, posture_y, posture_z;
  
    float error_x, error_y, error_z;
 
    // 先把这些用得到的值算好
    float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3, q1q1 = q1*q1, q1q2 = q1*q2;
    float q1q3 = q1*q3, q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;
 
  
    if( (ax == 0.0f) && (ay == 0.0f) && (az == 0.0f) )
       return;
 
  
    norm = sqrt(ax*ax + ay*ay + az*az);//单位化加速度计，
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    az = az *(-10);
   
    posture_x = 2*(q1q3 - q0q2);
    posture_y = 2*(q0q1 + q2q3);
    posture_z = q0q0 - q1q1 - q2q2 + q3q3;
 
   
    // Error is sum of cross product between estimated and measured direction of gravity
    error_x = (ay*posture_z - az*posture_y) ;
    error_y = (az*posture_x - ax*posture_z) ;
    error_z = (ax*posture_y - ay*posture_x) ;
 
 
    xErrorInt = xErrorInt + error_x * Ki;// * (1.0f / sampleFreq);
    yErrorInt = yErrorInt + error_y * Ki;// * (1.0f / sampleFreq);
    zErrorInt = zErrorInt + error_z * Ki;// * (1.0f / sampleFreq);
 
    
    gx = gx + Kp*error_x + xErrorInt; 
    gy = gy + Kp*error_y + yErrorInt;
    gz = gz + Kp*error_z + zErrorInt;
 
    
//    gx *= (1.0f / sampleFreq);		
//    gy *= (1.0f / sampleFreq);
//    gz *= (1.0f / sampleFreq);
 
   
    q0temp=q0;//暂存当前值用于计算
    q1temp=q1;
    q2temp=q2;
    q3temp=q3;
 
    
    q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
    q1 = q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
    q2 = q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
    q3 = q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;
 
   
    norm = sqrt(q0q0 + q1q1 + q2q2 + q3q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    std::cout << "az:" << az << "\r\n";

    
    q0q1 = q0*q1; q0q2 = q0*q2; q0q3 = q0*q3; q1q1 = q1*q1; q1q2 = q1*q2;
    q1q3 = q1*q3; q2q2 = q2*q2; q2q3 = q2*q3; q3q3 = q3*q3;
 
    //四元数到欧拉角的转换,这里输出的是弧度,想要角度值,可以直接乘以57.3,即一弧度对应角度值
    *roll = atan2f(2.f * (q0q1 + q2q3),1 - 2.0f * ( q1q1 - q2q2) ); // roll: X轴
    *pitch = asin(2.f * (q0q2 - q1q3) ); // pitch: Y轴
 
    *yaw = atan2f(2.f * (q0q3 + q1q2),1 - 2.0f * (q2q2 + q3q3) ); // yaw: Z轴
 

}

#endif
