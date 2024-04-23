/*
 * @Description: 
 * @Author: qingmeijiupiao
 * @Date: 2024-04-13 21:00:21
 * @LastEditTime: 2024-04-18 09:23:54
 * @LastEditors: Please set LastEditors
 */

#ifndef CAR_HPP
#define CAR_HPP
#include "C600.hpp"

//坐标系示意图,车正前方是Y正，右侧是X正，正上方是Z正
//         Z
//         |
//         |    Y
//         |   /
//   ______|__/
//  /     /| /
// /_____/ |/
// |     | O--------> X
// |_____|/

template<typename T>
struct vec3{
    T X=0;
    T Y=0;
    T Z=0;
    vec3(T x,T y,T z){
        X=x;
        Y=y;
        Z=z;
    }
};

//用电机编码器解算位置任务函数
void get_position_func(void *car);


//麦克纳姆轮底盘小车
class McNampCar{
    friend void get_position_func(void *car);
    public:
    /**
     * @description: 默认构造函数
     * @return {*}
     * @Author: qingmeijiupiao
     * @Date: 2024-04-17 09:29:37
     * @param {MOTOR*} _LFM 左前轮
     * @param {MOTOR*} _LBM 左后轮
     * @param {MOTOR*} _RFM 右前轮
     * @param {MOTOR*} _RBM 右后轮
     * @param {int} width 车宽,单位mm
     * @param {int} Long  车长,单位mm
     * @param {int} wheel_diameter 轮子直径,单位mm
     */
    McNampCar(MOTOR* _LFM,MOTOR* _LBM,MOTOR* _RFM,MOTOR* _RBM,float width=286.1,float Long=394,float wheel_diameter=152.4,bool need_get_position=false){
        LFM=_LFM;
        LBM=_LBM;
        RFM=_RFM;
        RBM=_RBM;
        this->car_width=width;
        this->car_long=Long;
        this->car_wheel_diameter=wheel_diameter;
        if(need_get_position){
            xTaskCreate(get_position_func, "get_position_func", 1024, this, 5, &position_func_handle);
        }
    }

    /**
     * @description: 电机旋转方向设置
     * @return {*}
     * @Author: qingmeijiupiao
     * @Date: 2024-04-17 09:22:14
     * @param {int8_t} _LFM_dir -1反方向，1为正方向
     * @param {int8_t} _LBM_dir -1反方向，1为正方向
     * @param {int8_t} _RFM_dir -1反方向，1为正方向
     * @param {int8_t} _RBM_dir -1反方向，1为正方向
     */
    void set_motor_dir(int8_t _LFM_dir,int8_t _LBM_dir,int8_t _RFM_dir,int8_t _RBM_dir){
        LFM_dir=_LFM_dir;
        LBM_dir=_LBM_dir;
        RFM_dir=_RFM_dir;
        RBM_dir=_RBM_dir;
    }
    //初始化
    void setup(){
        LFM->setup();
        LBM->setup();
        RFM->setup();
        RBM->setup();
    }

    
    //设置速度,XY单位M/S，Z单位rad/s
    void set_speed(vec3<float> speed){
        //计算每个轮子的速度
        float LFM_speed=LFM_dir*MPS2RPM(speed.Y+speed.X+speed.Z*0.0005*(car_long+car_width));
        float LBM_speed=LBM_dir*MPS2RPM(speed.Y-speed.X+speed.Z*0.0005*(car_long+car_width));
        float RFM_speed=RFM_dir*MPS2RPM(speed.Y-speed.X-speed.Z*0.0005*(car_long+car_width));
        float RBM_speed=RBM_dir*MPS2RPM(speed.Y+speed.X-speed.Z*0.0005*(car_long+car_width));
        LFM->set_speed(LFM_speed);
        LBM->set_speed(LBM_speed);
        RFM->set_speed(RFM_speed);
        RBM->set_speed(RBM_speed);
    }
    void set_speed(float speed_x,float speed_y,float speed_z){
        set_speed(vec3<float>(speed_x,speed_y,speed_z));
        
    }
    //停止
    void stop(bool need_unload=false){
        LFM->stop(false);
        LBM->stop(false);
        RFM->stop(false);
        RBM->stop(false);
    }
    //使能
    void load(){
        LFM->load();
        LBM->load();
        RFM->load();
        RBM->load();
    }
    
    /**
     * @description: 设置位置
     * @return {*}
     * @Author: qingmeijiupiao
     * @Date: 2024-04-17 10:30:02
     * @LastEditTime: Do not edit
     * @LastEditors: qingmeijiupiao
     * @param {vec3<float>} position 位置,X,Y单位M,Z单位rad
     */
    void set_position(vec3<float> position){
        now_position=position;
    }
    void set_position(float x,float y,float z){
        set_position(vec3<float>(x,y,z));
    }
    vec3<float> get_position(){
        return now_position;
    }
    private:
        //转换RPM为米每秒
        float RPM2MPS(float rpm){
            return 0.001*car_wheel_diameter*PI*rpm/60;//0.152是轮子直径
        }
        //转换米每秒为RPM
        float MPS2RPM(float mps){
            if(mps==0)
                return 0;
            return 60*mps/(0.001*152.4*PI);
        }
        MOTOR* LFM;//左前轮电机对象
        MOTOR* LBM;//左后轮电机对象
        MOTOR* RFM;//右前轮电机对象
        MOTOR* RBM;//右后轮电机对象
        float car_wheel_diameter;//轮子直径,单位mm
        float car_width=0;//车宽,单位mm
        float car_long=0;//车长,单位mm
        int8_t LFM_dir=1;//左前轮电机旋转方向
        int8_t LBM_dir=1;//左后轮电机旋转方向
        int8_t RFM_dir=-1;//右前轮电机旋转方向
        int8_t RBM_dir=-1;//右后轮电机旋转方向
        //靠电机编码器获取定位的任务句柄
        TaskHandle_t position_func_handle = NULL;
        //靠电机编码器获取定位的频率
        int position_func_HZ=100;
        vec3<float> now_position={0,0,0};//当前位置,单位米
};
void get_position_func(void *car){
    McNampCar *Ncar=(McNampCar*)car;
    int64_t last_LFM_position=Ncar->LFM->get_location();
    int64_t last_LBM_position=Ncar->LBM->get_location();
    int64_t last_RFM_position=Ncar->RFM->get_location();
    int64_t last_RBM_position=Ncar->RBM->get_location();
    
    float LFM_delta_position = 0;
    float LBM_delta_position = 0;
    float RFM_delta_position = 0;
    float RBM_delta_position = 0;
    while(1){
        //计算每个轮子的位置增量,单位米
        LFM_delta_position = PI*Ncar->car_wheel_diameter*(Ncar->LFM->get_reduction_ratio() * Ncar->LFM_dir * (Ncar->LFM->get_location() - last_LFM_position))/8192;
        LBM_delta_position = PI*Ncar->car_wheel_diameter*(Ncar->LBM->get_reduction_ratio() * Ncar->LBM_dir * (Ncar->LBM->get_location() - last_LBM_position))/8192;
        RFM_delta_position = PI*Ncar->car_wheel_diameter*(Ncar->RFM->get_reduction_ratio() * Ncar->RFM_dir * (Ncar->RFM->get_location() - last_RFM_position))/8192;
        RBM_delta_position = PI*Ncar->car_wheel_diameter*(Ncar->RBM->get_reduction_ratio() * Ncar->RBM_dir * (Ncar->RBM->get_location() - last_RBM_position))/8192;
        //保存上一次位置
        last_LFM_position = Ncar->LFM->get_location();
        last_LBM_position = Ncar->LBM->get_location();
        last_RFM_position = Ncar->RFM->get_location();
        last_RBM_position = Ncar->RBM->get_location();
        //计算当前位置增量
        Ncar->now_position.Y+=(LFM_delta_position+LBM_delta_position+RFM_delta_position+RBM_delta_position)/4.0;
        Ncar->now_position.X+=(LFM_delta_position+RFM_delta_position-RBM_delta_position-LBM_delta_position)/4.0;
        Ncar->now_position.Z+=(LFM_delta_position-LBM_delta_position-RFM_delta_position+RBM_delta_position)/(0.002*(Ncar->car_width+Ncar->car_long));
        //延时,让出CPU
        delay(1000/Ncar->position_func_HZ);
    }
};
#endif