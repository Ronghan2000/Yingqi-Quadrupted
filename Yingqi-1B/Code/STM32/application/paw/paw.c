#include "paw.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *paw0, *paw1, *paw2; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *paw_pub;
static Paw_Ctrl_Cmd_s paw_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *paw_sub;
static Paw_Upload_Data_s paw_feedback_data; // 来自cmd的发射控制信息
// dwt定时,计算冷却用
// static float hibernate_time = 0, dead_time = 0;

void PawInit()
{
    // 左摩擦轮
    Motor_Init_Config_s paw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 15, // 10
                .Ki = 1,
                .Kd = 0,
                .MaxOut = 1300000,
            },
            .speed_PID = {
                .Kp = 2, // 20
                .Ki = 0.002, // 1
                .Kd = 0.00,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 1300000,
                .MaxOut = 1300000,
            },
            .current_PID = {
                .Kp = 0.3, // 0.7
                .Ki = 0.01, // 0.1
                .Kd = 0.000,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP      ,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006};
    paw_config.can_init_config.tx_id = 4,
    paw_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    paw0 = DJIMotorInit(&paw_config);

    paw_config.can_init_config.tx_id = 5; // 右摩擦轮,改txid和方向就行
    paw_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    paw1 = DJIMotorInit(&paw_config);

    // 夹爪电机
    Motor_Init_Config_s catch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 6,
        },
        .controller_param_init_config = {
            
            .speed_PID = {
                .Kp = 2, // 10
                .Ki = 0.002, // 1
                .Kd = 0.0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 2000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    catch_config.can_init_config.tx_id = 6;
    paw2 = DJIMotorInit(&catch_config);

    paw_pub = PubRegister("paw_feed", sizeof(Paw_Upload_Data_s));
    paw_sub = SubRegister("paw_cmd", sizeof(Paw_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void PawTask()
{
    // 从cmd获取控制数据
    SubGetMessage(paw_sub, &paw_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (paw_cmd_recv.paw_mode == Paw_OFF)
    {
        DJIMotorStop(paw0);
        DJIMotorStop(paw1);
        DJIMotorStop(paw2);
    }
    else // 恢复运行
    {
        DJIMotorEnable(paw0);
        DJIMotorEnable(paw1);
        DJIMotorEnable(paw2);
    }
    //夹爪
     
    if((paw2->measure.real_current>50000)||(paw2->measure.real_current<-50000)){
        if(paw_cmd_recv.paw2location==up){
            DJIMotorOuterLoop(paw2, CURRENT_LOOP);
            DJIMotorSetRef(paw2, 1500);
            }else if(paw_cmd_recv.paw2location==down){
                    DJIMotorOuterLoop(paw2, CURRENT_LOOP);
                    DJIMotorSetRef(paw2, -1500);
            }else{
                DJIMotorOuterLoop(paw2, CURRENT_LOOP);
                DJIMotorSetRef(paw2, 0);
            }
    }else{
        if(paw_cmd_recv.paw2location==up){
            DJIMotorOuterLoop(paw2,SPEED_LOOP);
            DJIMotorSetRef(paw2, 15000);
            }else if(paw_cmd_recv.paw2location==down){
                    DJIMotorOuterLoop(paw2,  SPEED_LOOP);
                    DJIMotorSetRef(paw2, -15000);
            }else{
                DJIMotorOuterLoop(paw2,  SPEED_LOOP);
                DJIMotorSetRef(paw2, 0);
            }
    }

    switch(paw_cmd_recv.paw0location) {
        case up:
            DJIMotorOuterLoop(paw0,  SPEED_LOOP);
            DJIMotorSetRef(paw0, -15000);
            break;
        case down:
            DJIMotorOuterLoop(paw0,  SPEED_LOOP);
            DJIMotorSetRef(paw0, 15000);
            break;
        case stand:
            DJIMotorOuterLoop(paw0,  SPEED_LOOP);
            DJIMotorSetRef(paw0, 0);
            break;
        default:
            // Handle other cases here if necessary
            break;
    }
    switch(paw_cmd_recv.paw1location) {
        case up:
            DJIMotorOuterLoop(paw1,  SPEED_LOOP);
            DJIMotorSetRef(paw1, -15000);
            break;
        case down:
            DJIMotorOuterLoop(paw1,  SPEED_LOOP);
            DJIMotorSetRef(paw1, 15000);
            break;
        default:
            DJIMotorOuterLoop(paw1,  SPEED_LOOP);
            DJIMotorSetRef(paw1, 0);
            break;
    }
    
    // if(paw_cmd_recv.paw0location==up && paw0->measure.total_angle < (paw0->measure.first_angle + 360.0f*36.0f*36.0f /1.85f)){
    //     DJIMotorSetRef(paw0, paw0->measure.total_angle + 360.0f*36.0f*36.0f/1000/1.85f *1.5);//speed 20°/s
    // }else if(paw_cmd_recv.paw0location==down && paw0->measure.total_angle>(paw0->measure.first_angle  + (360.0f*36.0f*36.0f/1.85f) *0.1)){//比底部搞大概0.1圈
    //     DJIMotorSetRef(paw0, paw0->measure.total_angle - 360.0f*36.0f*36.0f/1000.0f/1.85f *1.5);//speed -20°/s
    // }else{
    //     // DJIMotorSetRef(paw0, paw0->measure.first_angle+360.0f*36.0f*36.0f/1000.0f/1.85f * 0.1);
    // }
    // if(paw_cmd_recv.paw0location==up){
    //     DJIMotorSetRef(paw0, paw0->measure.first_angle+360.0f*36.0f*36.0f/1000.0f/1.85f * 0.1);//speed 20°/s
    // }else if(paw_cmd_recv.paw0location==down ){//比底部搞大概0.1圈
    //     DJIMotorSetRef(paw0, paw0->measure. first_angle  );//speed -20°/s
    // }
        

    //  if(paw_cmd_recv.paw1location==up && paw1->measure.total_angle < (paw1->measure.first_angle + 360.0f*36.0f*36.0f/1000.0f/1.85f)){
    //     DJIMotorSetRef(paw1, paw1->measure.total_angle + 360.0f*36.0f*36.0f/1000.0f/1.85f * 20);//speed 20°/s
    // }else if(paw_cmd_recv.paw1location==down && paw1->measure.first_angle < (paw1->measure.first_angle  + 360.0f*36.0f*36.0f/1000.0f/1.85f * 0.1)){//比底部搞大概0.1圈
    //     DJIMotorSetRef(paw1, paw1->measure.total_angle - 360.0f*36.0f*36.0f/1000.0f/1.85f * 20);//speed -20°/s
    // }else{
    //     DJIMotorSetRef(paw1, paw1->measure.total_angle);
    // }
    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    // switch (paw_cmd_recv.load_mode)
    // {
    // // 停止拨盘
    // case LOAD_STOP:
    //     DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
    //     DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
    //     break;
    // // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    // case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
    //     DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
    //     DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
    //     hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
    //     dead_time = 150;                                                                    // 完成1发弹丸发射的时间
    //     break;
    // // 三连发,如果不需要后续可能删除
    // case LOAD_3_BULLET:
    //     DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到速度环
    //     DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
    //     hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
    //     dead_time = 300;                                                                        // 完成3发弹丸发射的时间
    //     break;
    // // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    // case LOAD_BURSTFIRE:
    //     DJIMotorOuterLoop(loader, SPEED_LOOP);
    //     DJIMotorSetRef(loader, paw_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
    //     // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
    //     break;
    // // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // // 也有可能需要从switch-case中独立出来
    // case LOAD_REVERSE:
    //     DJIMotorOuterLoop(loader, SPEED_LOOP);
    //     // ...
    //     break;
    // default:
    //     while (1)
    //         ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    // }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    // if (paw_cmd_recv.friction_mode == FRICTION_ON)
    // {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        // switch (shoot_cmd_recv.bullet_speed)
        // {
        // case SMALL_AMU_15:
        //     DJIMotorSetRef(friction_l, 0);
        //     DJIMotorSetRef(friction_r, 0);
        //     break;
        // case SMALL_AMU_18:
            //   DJIMotorSetRef(friction_l, 0);
        //     DJIMotorSetRef(friction_r, 0);
        //     break;
        // case SMALL_AMU_30:
        //     DJIMotorSetRef(friction_l, 0);
        //     DJIMotorSetRef(friction_r, 0);
        //     break;
        // default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
        //     DJIMotorSetRef(friction_l, 30000);
        //     DJIMotorSetRef(friction_r, 30000);
        //     break;
        // }
    // }
    // else // 关闭摩擦轮
    // {
        // DJIMotorSetRef(friction_l, 0);
        // DJIMotorSetRef(friction_r, 0);
    // }

    // 开关弹舱盖
    // if (paw_cmd_recv.lid_mode == LID_CLOSE)
    // {
        //...
    // }
    // else if (paw_cmd_recv.lid_mode == LID_OPEN)
    // {
        //...
    // }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(paw_pub, (void *)&paw_feedback_data);
}