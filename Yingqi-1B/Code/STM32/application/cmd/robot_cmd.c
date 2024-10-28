// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include  "distance.h"
#include "ins_task.h"
// #include "master_process.h"--视觉
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "ins_task.h"
#ifdef ONE_BOARD
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
#endif                                 // ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Distance_data *dt_data;

static Publisher_t *paw_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *paw_feed_sub;         // 发射反馈信息订阅者
static Paw_Ctrl_Cmd_s paw_cmd_send;      // 传递给发射的控制信息
static Paw_Upload_Data_s paw_fetch_data; // 从发射获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
// static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    dt_data = dtInit(&huart1); //获得测距传感器数据
    // vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    // gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    // gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    paw_cmd_pub = PubRegister("paw_cmd", sizeof(Paw_Ctrl_Cmd_s));
    paw_feed_sub = SubRegister("paw_feed", sizeof(Paw_Upload_Data_s));
     shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
     

#ifdef ONE_BOARD //  双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    // gimbal_cmd_send.pitch = 0;--云台

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

static void EmergencyHandler()//----设置急停
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.SE < 0 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        // gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        paw_cmd_send.paw_mode = Paw_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
         LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if ( rc_data[TEMP].rc.SE>=0)
    {   
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        robot_state = ROBOT_READY;
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        paw_cmd_send.paw_mode = Paw_ON;
         LOGINFO("[CMD] reinstate, robot ready");
    }
}

/**
 * @brief 获得底盘的当前角度和距离
 *
 */
static void CalcOffsetAngleDistance(){
    // chassis_cmd_send.x=(dt_data[TEMP].l2+dt_data[TEMP].r1)/2;
    // chassis_cmd_send.lf0=dt_data[TEMP].lf0;
    // chassis_cmd_send.lf1=dt_data[TEMP].lf1;
    // chassis_cmd_send.lf2=dt_data[TEMP].lf2;
    // chassis_cmd_send.rf0=dt_data[TEMP].rf0;
    // chassis_cmd_send.rf1=dt_data[TEMP].rf1;
    // chassis_cmd_send.rf2=dt_data[TEMP].rf2;
    // chassis_cmd_send.lb0=dt_data[TEMP].lb0;
    // chassis_cmd_send.lb1=dt_data[TEMP].lb1;
    // chassis_cmd_send.lb2=dt_data[TEMP].lb2;
    // chassis_cmd_send.rb0=dt_data[TEMP].rb0;
    // chassis_cmd_send.rb1=dt_data[TEMP].rb1;
    // chassis_cmd_send.rb2=dt_data[TEMP].rb2;

}


/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    // if (rc_data[TEMP].rc.SE>=0) // 右侧开关状态[下],底盘跟随云台
    // {
        
        // gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    // }
    // else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    // {
        // chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        // gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    // }

    // 云台参数,确定云台控制数据
    // if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中],视觉模式
    // {
        // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
        // ...
    // }
    // // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    // if (switch_is_down(rc_data[TEMP].rc.switch_left) || vision_recv_data->target_state == NO_TARGET)
    // { // 按照摇杆的输出大小进行角度增量,增益系数需调整
    //     gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
    //     gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
    // }
    // paw 0
    // if(rc_data[TEMP].rc.SA<500){
    //     paw_cmd_send.paw0location=up;
    // }else
    // if(rc_data[TEMP].rc.SA>1500){
    //     paw_cmd_send.paw0location=down;
    // }else{
    //     paw_cmd_send.paw0location=stand;
    // } 
    // // paw 1
    // if(rc_data[TEMP].rc.SB<500){
    //     paw_cmd_send.paw1location=up;
    // }
    // else if
    // (rc_data[TEMP].rc.SB>1500){
    //     paw_cmd_send.paw1location=down;
    // }else {
    //     paw_cmd_send.paw1location=stand;
    // }
    // // paw 2    
    // if(rc_data[TEMP].rc.SC<500){
    //     paw_cmd_send.paw2location=up;
    // }
    // else if
    // (rc_data[TEMP].rc.SC>1500){
    //     paw_cmd_send.paw2location=down;
    // }else {
    //     paw_cmd_send.paw2location=stand;
    // }

    // if(rc_data[TEMP].rc.SD>600){
    //     shoot_cmd_send.load_mode=LOAD_1_BULLET;
    // }else{
    //     shoot_cmd_send.load_mode=LOAD_STOP;
    // }


    // // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    // chassis_cmd_send.vx =   (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
    // chassis_cmd_send.vy =  (float)rc_data[TEMP].rc.rocker_r1; // 1数值方向
    //  chassis_cmd_send.wz =   (float)rc_data[TEMP].rc.rocker_l_;
    //   // 旋转方向

    // if(rc_data[TEMP].rc.SG>800){
       
    //     shoot_cmd_send.shoot_rate1=60.0f*124;
        
    // }else{
    //     shoot_cmd_send.shoot_rate1=0;
        
    // }
    // if(rc_data[TEMP].rc.SF>800){
    //     shoot_cmd_send.shoot_rate2= -60.0f *924;
    // }else{
    //     shoot_cmd_send.shoot_rate2= 0;
    // }
    // 发射参数
    // if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
    //     ;                                            // 弹舱舵机控制,待添加servo_motor模块,开启
    // else
    //     ; // 弹舱舵机控制,待添加servo_motor模块,关闭

    //摩擦轮控制,拨轮向上打为负,向下为正
    // if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
    //     shoot_cmd_send.friction_mode = FRICTION_ON;
    // else
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    // // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    // if (rc_data[TEMP].rc.dial < -500)
    //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    // else
    //     shoot_cmd_send.load_mode = LOAD_STOP;
    // // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    // shoot_cmd_send.shoot_rate = 8;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(paw_feed_sub, &paw_fetch_data);//---云台和发射
    // SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
        CalcOffsetAngleDistance();
    // // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    // if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
        RemoteControlSet();
    // else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        // MouseKeySet();

        EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(paw_cmd_pub, (void *)&paw_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    
    // PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    // VisionSend(&vision_send_data);
}
