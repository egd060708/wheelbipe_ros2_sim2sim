/*! @file PIDmethod.h
*  @brief PID Algorithm adjusts manipulator robot
*
*  This file contains the PIDmethod class and PIDtimer class, which provide ways to run PID.
*
*/
#ifndef _PIDMETHOD_H_
#define _PIDMETHOD_H_

#include <iostream>
#include <stdint.h>
#include <cstdint>
#include <stddef.h>
#include <math.h>
#include <limits>//定义各种变量的储存最大值
#include "Upper_Public.h"
#include <deque>

using namespace std;


typedef long long(*SystemTick_Fun)(void);//函数指针，用于接入不同单片机获取时间栈的函数

typedef enum
{
    Common,
    Fit
}Params_Mode;

typedef enum class _PID_Mode
{
    IS_PI,
    IS_PD
}PID_Mode;

class PIDtimer
{
public:
    static uint8_t getMicroTick_regist(long long(*getTick_fun)(void));//获取当前时间函数接口
    static SystemTick_Fun Get_SystemTick;   //获取时间的函数
    double dt = 0;				                //时间微分
    long long last_time = 0; 	                //记录上次时间
    uint8_t UpdataTimeStamp(void);          //时间栈更新

};

/*PID方法类*/
class PIDmethod : public PIDtimer
{
public:
    PIDmethod() {}
    PIDmethod(Params_Mode mode, double _timeStep = 0);//必须输入参数模式，时间栈选择输入
    void PID_Init(Params_Mode mode, double _timeStep = 0);
    //设置参数
    //拟合线性函数方法
    void Params_Config(Fit_Params _fun_p, Fit_Params _fun_i, Fit_Params _fun_d, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());
    void Params_Config(Fit_Params _fun_p, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());//纯p
    void Params_Config(PID_Mode mode, Fit_Params _fun_p, Fit_Params _fun_id, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());//pi或pd控制
    //普通参数方法
    void Params_Config(double _kp, double _ki, double _kd, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());
    void Params_Config(double _kp, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());//纯p
    void Params_Config(PID_Mode mode, double _kp, double _kid, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());//pi或pd控制
    //结算
    double Adjust(double _x);//线性自变量，使用自己算的d项
    double Adjust(double _x, double extern_d);//线性自变量，使用外部计算的d项
    double Adjust_Incremental();// 增量式全部基于误差计算，暂不引入额外参数

    //清理
    void Clear();

    Fit_Params fun_p = { 0 }, fun_i = { 0 }, fun_d = { 0 };//三次拟合参数
    double kp = 0, ki = 0, kd = 0;//普通参数
    double fact_kp = 0, fact_ki = 0, fact_kd = 0;

    double Error_Max = numeric_limits<double>::max();//P项限幅
    double I_Term_Max = 0;//I项限幅
    double Output_Max = 0;//输出上限
    double Output_Min = 0;//输出下限

    double P_Term = 0;//比例项输出
    double I_Term = 0;//积分项输出
    double D_Term = 0;//微分项输出

    double I_SeparThresh = 400;   /*!< 积分分离阈值，需为正数。std::abs(error)大于该阈值取消积分作用。*/

    double target = 0;
    double current = 0;
    double error = 0;
    double out = 0;
    double integral = 0;

    std::deque<double> error_buf = std::deque<double>(3,0); // 用于增量式pid，初始化长度为3的队列

    double timeStep = 0;//如果被赋值，则以此为微分时间

    bool d_of_current = true;//是否使用微分先行

    int fit_degree = 3;
private:
    //线性拟合函数系数+自变量
    double fit_function(Fit_Params param, double x);
    Params_Mode params_mode = Common;
    double last_current = 0;
    double last_error = 0;
    double d_current = 0;
    double d_error = 0;
    

};

#endif