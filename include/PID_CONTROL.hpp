/*
 * @Description: PID控制器
 * @Author: qingmeijiupiao
 * @Date: 2024-04-13 21:00:21
 * @LastEditTime: 2024-04-17 22:19:19
 * @LastEditors: Please set LastEditors
 */

#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP
#include <Arduino.h>
//PID参数
struct pid_param{
    float Kp;//比例系数
    float Ki;//积分系数
    float Kd;//微分系数
    float _dead_zone;//死区
    float _max_value;//最大值   
    pid_param(float P=0,float I=0,float D=0,float dead_zone=100,float max_value=10000){
        Kp = P;
        Ki = I;
        Kd = D;
        _dead_zone = dead_zone;
        _max_value = max_value;
    }
};

class PID_CONTROL{
    public:
    PID_CONTROL(){}
    PID_CONTROL(float P,float I,float D,float dead_zone=0,float max_value=0){
        Kp = P;
        Ki = I;
        Kd = D;
        _dead_zone = dead_zone;
        _max_value = max_value;
        integral = 0.0;
        prevError = 0.0;
        last_contrl_time = 0;
    }
    double control(double error){
        if(abs(error)<_dead_zone){
            error=0;
        }
        double time_p=micros()-last_contrl_time;
        if (last_contrl_time=0 || time_p<0){
            time_p=1;
        }
        // 计算时间,确保ki，kd在不同控制频率的一致性
        time_p*=1e-6;
        // 计算比例项
        double proportional = Kp * error;

        // 计算积分项
        integral += time_p*(Ki * error);
        double derivative=0;
        // 计算微分项
        if(time_p!=0)
            derivative =Kd * (error - prevError)/time_p;


        // 计算总的控制输出
        double output = proportional + integral + derivative;
        last_contrl_time=micros();
        if(abs(output)>_max_value){
            return output>0?_max_value:-1*_max_value;
        }
        return output;
    }
    // 重置控制器状态
    void reset() {
        integral = 0.0;
        prevError = 0.0;
    }
    // 修改控制器参数
    void setPram(float P=0,float I=0,float D=0,float dead_zone=0,float max_value=0){
        if(P!=0){
            Kp = P;
        }
        if(I!=0){
            Ki = I;
        }
        if(D!=0){
            Kd = D;
        }
        if(dead_zone!=0){
            _dead_zone = dead_zone;
        }
        if(max_value!=0){
            _max_value = max_value;
        }
        reset();
    }
    void setPram(pid_param pid){
        setPram(pid.Kp,pid.Ki,pid.Kd,pid._dead_zone,pid._max_value);
    }
    double Kp;  // 比例系数
    double Ki;  // 积分系数
    double Kd;  // 微分系数
    double _max_value;  // 输出限幅
    double _dead_zone;  // 死区
private:
    double setpoint;  // 设定值
    double integral;  // 积分项
    double prevError; // 上一次误差
    double last_contrl_time=0;
};


#endif