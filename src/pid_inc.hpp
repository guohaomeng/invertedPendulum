#ifndef PIDINC_H
#define PIDINC_H
#include <SimpleFOC.h>

class PID_INC
{
public:
  float P, I, D;
  float Current_Error;  //当前误差
  float Last_Error;     //上一次误差
  float Previous_Error; //上上次误差
  PID_INC(float P, float I, float D);
  ~PID_INC() = default;
  float PID_Increase(float error);
};

PID_INC::PID_INC(float P, float I, float D)
    : P(P), I(I), D(D), Current_Error(0.0f) // e(k)
      ,
      Last_Error(0.0f) // e(k-1)
      ,
      Previous_Error(0.0f) // e(k-2)
{
}
/*!
 *  @brief      增量式PID
 *  @since      v1.0
 *  *sptr ：误差参数
 *  *pid:  PID参数
 *  NowPlace：实际值
 *  Point：   期望值
 */

// 增量式PID电机控制
float PID_INC::PID_Increase(float error)
{

  float Increase; //最后得出的实际增量

  Increase = P * (error - Last_Error)                            //比例P
             + I * error                                         //积分I
             + D * (error - 2.0f * Last_Error + Previous_Error); //微分D

  Previous_Error = Last_Error; // 更新前次误差
  Last_Error = error;          // 更新上次误差
  return Increase;             // 返回增量
}

#endif