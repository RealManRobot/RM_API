#ifndef RM_DEFINE_H
#define RM_DEFINE_H
//////////////////////////////////////////////////////////////////////////////////
//睿尔曼智能科技有限公司        Author:Dong Qinpeng
//创建日期:2022/08/23
//版本：V4.0
//版权所有，盗版必究。
//Copyright(C) 睿尔曼智能科技有限公司
//All rights reserved
//文档说明：该文档定义了机械臂接口函数中使用到的结构体和错误代码类型
//////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "cJSON.h"


#include "robot_define.h"

#ifdef _WIN32
#define MSG_DONTWAIT 0
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>
typedef SOCKET  SOCKHANDLE;
#endif

#ifdef __linux
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

typedef int SOCKHANDLE;
#endif

#define  SDK_VERSION (char*)"4.3.6"

typedef unsigned char byte;
typedef unsigned short u16;

////位姿结构体
//typedef struct
//{
//    //位置
//    float px;
//    float py;
//    float pz;
//    //欧拉角
//    float rx;
//    float ry;
//    float rz;
//}POSE;

// 回调函数结构体
typedef struct
{
    int sockhand;       ///< 返回调用时句柄
    int codeKey;        ///< 调用接口类型
    int errCode;        ///< 接口错误码
    Pose pose;          ///< 位姿信息
    float joint[7];     ///< 角度信息
    float direction_force[7]; //所有方向的力或力矩
    int nforce;         ///< 返回力大小
    uint16_t sys_err;   ///< 返回系统错误
} CallbackData;

//坐标系
typedef struct
{
    char name[12];    //坐标系名称,不超过10个字符
}FRAME_NAME;

//坐标系名称列表-兼容MATLAB API
typedef struct
{
    FRAME_NAME name[10];    //名称列表
}NAMES;

//坐标系
typedef struct
{
    FRAME_NAME frame_name;  //坐标系名称
    Pose pose;              //坐标系位姿
    float payload;     //坐标系末端负载重量
    float x;           //坐标系末端负载位置
    float y;           //坐标系末端负载位置
    float z;           //坐标系末端负载位置
}FRAME;

/**
 * @brief 机械臂当前规划类型
 *
 */
typedef enum
{
    None_Mode = 0,     //无规划
    Joint_Mode = 1,    //关节空间规划
    Line_Mode = 2,     //笛卡尔空间直线规划
    Circle_Mode = 3,   //笛卡尔空间圆弧规划
    Replay_Mode = 4,    //拖动示教轨迹复现
    Moves_Mode = 5,      //样条曲线运动
    Blend_Mode = 6      // 交融轨迹规划
}ARM_CTRL_MODES;

/**
 * @brief 机械臂位置示教模式
 *
 */
typedef enum
{
    X_Dir = 0,       //X轴方向
    Y_Dir = 1,       //Y轴方向
    Z_Dir = 2,       //Z轴方向
}POS_TEACH_MODES;

//机械臂姿态示教模式
typedef enum
{
    RX_Rotate = 0,       //RX轴方向
    RY_Rotate = 1,       //RY轴方向
    RZ_Rotate = 2,       //RZ轴方向
}ORT_TEACH_MODES;

//控制器通讯方式选择
typedef enum
{
    WIFI_AP = 0,       //WIFI AP模式
    WIFI_STA = 1,      //WIFI STA模式
    BlueTeeth = 2,     //蓝牙模式
    USB       = 3,     //通过控制器UART-USB接口通信
    Ethernet  = 4      //以太网口
} ARM_COMM_TYPE;

//机械臂自由度
#define   ARM_DOF               7              //机械臂自由度
//机械臂状态参数
typedef struct
{
    //float joint[ARM_DOF];       ///< 关节角度
    float temperature[ARM_DOF];   ///< 关节温度
    float voltage[ARM_DOF];       ///< 关节电压
    float current[ARM_DOF];       ///< 关节电流
    byte en_state[ARM_DOF];       ///< 使能状态
    uint16_t err_flag[ARM_DOF];   ///< 关节错误代码
    uint16_t sys_err;             ///< 机械臂系统错误代码
}JOINT_STATE;

//位置
typedef struct
{
    //position
    float px;
    float py;
    float pz;
    //orientation
    float w;
    float x;
    float y;
    float z;
}POSE_QUAT;

//姿态
typedef struct
{
    float rx;
    float ry;
    float rz;
}ORT;
//typedef struct
//{
//    POSE2 pose;
//    ORT ort;
//}KINEMATIC;
//旋转矩阵
//typedef struct
//{
//    int irow;
//    int iline;
//    float data[4][4];
//}_Matrix;

//位置
//typedef struct
//{
//    float x;
//    float y;
//    float z;
//}pos;

////四元数姿态
//typedef struct
//{
//    float w;
//    float x;
//    float y;
//    float z;
//}ort;

////欧拉角姿态
//typedef struct
//{
//    float Phi;
//    float Theta;
//    float Psi;
//}eul;

//机械臂位姿
//typedef struct
//{
//    pos position;    // px
//    ort orientation; // rx
//    eul euler;
//}Pose;

// 无线网络信息结构体
typedef struct{
    int channel;               ///< 信道 AP模式时存在此字段
    char ip[16];               ///< IP 地址
    char mac[18];              ///< MAC 地址
    char mask[16];             ///< 子网掩码
    char mode[5];              ///< 模式
    char password[16];         ///< 密码
    char ssid[32];             ///< 网络名称 (SSID)
}WiFi_Info;

// 在线编程存储信息
typedef struct  {
    int id;
    int size;
    int speed;
    char trajectory_name[32];
}TrajectoryData;

// 在线编程程序列表
typedef struct{
    int page_num;       ///< 页码（全部查询时此参数传NULL）
    int page_size;      ///< 每页大小（全部查询时此参数传NULL）
    int total_size;
    char vague_search[32];  ///< 模糊搜索 （传递此参数可进行模糊查询）
    TrajectoryData list[100];   ///< 符合的在线编程列表
}ProgramTrajectoryData;

/**
 * 在线编程运行状态结构体
 */
typedef struct{
    int run_state;  ///< 0 未开始 1运行中 2暂停中
    int id;         ///< 运行轨迹编号，已存储轨迹 的id，没有存储则为0 ，未运行则不返回
    int edit_id;    ///< 上次编辑的在线编程编号 ID，未运行时返回，没有存储则为0
    int plan_num;   ///< 运行到的行数，未运行则不返回
    int loop_num[10];   ///< 存在循环指令的行数，未运行则不返回
    int loop_cont[10];  ///< 循环指令行数对应的运行次数，未运行则不返回
    int step_mode;  ///< 单步模式，1为单步模式，0为非单步模式，未运行则不返回
    int plan_speed; ///< 全局规划速度比例 1-100，未运行则不返回
}ProgramRunState;

/**
 * 扩展关节配置参数
 */
typedef struct{
    int32_t rpm_max;        ///<  关节的最大速度
    int32_t rpm_acc;        ///< 最大加速度
    int32_t conversin_coe;  ///< 减速比,该字段只针对升降关节（指直线运动的关节）有效；如果是旋转关节（指做旋转运动的关节），则不发送该字段，注意参数的设置一定跟电机匹配，避免发生意外
    int32_t limit_min;      ///< 最小限位，如果是旋转关节，单位为°，精度0.001，如果是升降关节，则单位为mm
    int32_t limit_max;      ///< 最大限位，如果是旋转关节，单位为°，精度0.001，如果是升降关节，则单位为mm
}ExpandConfig;


//实时机械臂状态上报
/**
 * 机械臂关节状态结构体
 */
typedef struct {
    float joint_current[ARM_DOF];
    byte joint_en_flag[ARM_DOF];
    uint16_t joint_err_code[ARM_DOF];
    float joint_position[ARM_DOF];
    float joint_temperature[ARM_DOF];
    float joint_voltage[ARM_DOF];
    float joint_speed[ARM_DOF];
} JointStatus;

/**
 * 力传感器结构体
 */
typedef struct {
    float force[6];
    float zero_force[6];
    int coordinate;         ///< 系统外受力数据的坐标系，0为传感器坐标系 1为当前工作坐标系 2为当前工具坐标系
} ForceData;

/***
 * udp推送扩展关节数据
 *
 */
typedef struct {
    float pos;            ///< 当前角度  精度 0.001°，单位：°
    int current;        ///< 当前驱动电流，单位：mA，精度：1mA
    int err_flag;       ///< 驱动错误代码，错误代码类型参考关节错误代码
    int en_flag;        ///< 当前关节使能状态 ，1 为上使能，0 为掉使能
    int joint_id;       ///< 关节id号
    int mode;           ///< 当前升降状态，0-空闲，1-正方向速度运动，2-正方向位置运动，3-负方向速度运动，4-负方向位置运动
} ExpandState;

/***
 * udp推送升降机构状态
 *
 */
typedef struct {
    int height;         ///< 当前升降机构高度，单位：mm，精度：1mm
    float pos;            ///< 当前角度  精度 0.001°，单位：°
    int current;        ///< 当前驱动电流，单位：mA，精度：1mA
    int err_flag;       ///< 驱动错误代码，错误代码类型参考关节错误代码
    int en_flag;        ///< 当前关节使能状态 ，1 为上使能，0 为掉使能
} LiftState;

/***
 * udp推送灵巧手状态
 *
 */
typedef struct {
    int hand_pos[6];         ///< 表示灵巧手位置
    int hand_angle[6];         ///< 表示灵巧手角度
    int hand_force[6];            ///< 表示灵巧手自由度电流，单位mN
    int hand_state[6];        ///< 表示灵巧手自由度状态
    int hand_err;       ///< 表示灵巧手系统错误，由灵巧手厂商定义错误含义，例如因时状态码如下：1表示有错误，0表示无错误
} HandState;


/**
 * @brief udp推送机械臂当前状态
 *
 */
typedef enum {
    RM_IDLE_E,                     // 使能但空闲状态
    RM_MOVE_L_E,                   // move L运动中状态
    RM_MOVE_J_E,                   // move J运动中状态
    RM_MOVE_C_E,                   // move C运动中状态
    RM_MOVE_S_E,                   // move S运动中状态
    RM_MOVE_THROUGH_JOINT_E,       // 角度透传状态
    RM_MOVE_THROUGH_POSE_E,        // 位姿透传状态
    RM_MOVE_THROUGH_FORCE_POSE_E,  // 力控透传状态
    RM_MOVE_THROUGH_CURRENT_E,     // 电流环透传状态
    RM_STOP_E,                     // 急停状态
    RM_SLOW_STOP_E,                // 缓停状态
    RM_PAUSE_E,                    // 暂停状态
    RM_CURRENT_DRAG_E,             // 电流环拖动状态
    RM_SENSOR_DRAG_E,              // 六维力拖动状态
    RM_TECH_DEMONSTRATION_E        // 示教状态
} ArmCurrentStatus;
/***
 * aloha主臂状态
 *
 */
typedef struct {
    int io1_state;         ///<  IO1状态（手柄光电检测），0为按键未触发，1为按键触发。
    int io2_state;        ///<  IO2状态（手柄光电检测），0为按键未触发，1为按键触发。
} Alohastate;
/**
 * UDP接口实时机械臂状态上报
 */
typedef struct {
    int errCode;        ///< 接口错误码
    char *arm_ip;       ///< 上报数据的机械臂IP
    uint16_t arm_err;
    JointStatus joint_status;   ///< 关节状态
    ForceData force_sensor;     ///< 力数据
    uint16_t sys_err;       ///< 系统错误码
    Pose waypoint;      ///< 当前路点
    LiftState liftState;      ///< 升降关节数据
    ExpandState expandState;      ///< 扩展关节数据
    HandState handState;         ///< 灵巧手数据
    ArmCurrentStatus arm_current_status;     ///< 机械臂状态
    Alohastate aloha_state;     ///< aloha主臂状态
} RobotStatus;


//几何模型名称列表
typedef struct
{
    char name[12];    ///< 几何模型名称,不超过10个字符
}ElectronicFenceNames;

//几何模型参数
typedef struct
{
    int32_t form;       ///< 形状，1 表示立方体，2 表示点面矢量平面，3 表示球体
    char name[12];      ///< 几何模型名称，不超过10个字节，支持字母、数字、下划线
    // 立方体
    float x_min_limit;    ///< 立方体基于世界坐标系 X 方向最小位置，单位 m
    float x_max_limit;    ///< 立方体基于世界坐标系 X 方向最大位置，单位 m
    float y_min_limit;    ///< 立方体基于世界坐标系 Y 方向最小位置，单位 m
    float y_max_limit;    ///< 立方体基于世界坐标系 Y 方向最大位置，单位 m
    float z_min_limit;    ///< 立方体基于世界坐标系 Z 方向最小位置，单位 m
    float z_max_limit;    ///< 立方体基于世界坐标系 Z 方向最大位置，单位 m
    // 点线矢量平面
    float x1, y1, z1;     ///< 表示点面矢量平面三点法中的第一个点坐标，单位 m
    float x2, y2, z2;     ///< 表示点面矢量平面三点法中的第二个点坐标，单位 m
    float x3, y3, z3;     ///< 表示点面矢量平面三点法中的第三个点坐标，单位 m
    // 球体
    float radius;     ///< 表示半径，单位 0.001m
    float x, y, z;    ///< 表示球心在世界坐标系 X 轴、Y轴、Z轴的坐标，单位 m
}ElectronicFenceConfig;

// 几何模型参数列表-适配matlab
typedef struct
{
    ElectronicFenceConfig config[10];
}ElectronicFenceConfigList;

//夹爪状态
typedef struct
{
    bool enable_state;  ///< 夹爪使能标志，0 表示未使能，1 表示使能
    bool status;         ///< 夹爪在线状态，0 表示离线， 1表示在线
    int32_t error;          ///< 夹爪错误信息，低8位表示夹爪内部的错误信息bit5-7 保留bit4 内部通bit3 驱动器bit2 过流 bit1 过温bit0 堵转
    int32_t mode;           ///< 当前工作状态：1 夹爪张开到最大且空闲，2 夹爪闭合到最小且空闲，3 夹爪停止且空闲，4 夹爪正在闭合，5 夹爪正在张开，6 夹爪闭合过程中遇到力控停止
    int32_t current_force;  ///< 夹爪当前的压力，单位g
    int32_t temperature;    ///< 当前温度，单位℃
    int32_t actpos;         ///< 夹爪开口度
}GripperState;

typedef struct{
    char build_time[20];
    char version[10];
}CtrlInfo;

typedef struct{
    char model_version[5];
}DynamicInfo;

typedef struct{
    char build_time[20];
    char version[10];
}PlanInfo;

typedef struct {
    char version[20];
}AlgorithmInfo;

// 机械臂软件信息
typedef struct
{
    char product_version[10];
    AlgorithmInfo algorithm_info;
    CtrlInfo ctrl_info;
    DynamicInfo dynamic_info;
    PlanInfo plan_info;
}ArmSoftwareInfo;

// 定义包络参数结构体
typedef struct
{
    char name[12];      ///< 工具包络球体的名称，1-10 个字节，支持字母数字下划线
    float radius;     ///< 工具包络球体的半径，单位 0.001m
    float x;      ///< 工具包络球体球心基于末端法兰坐标系的 X 轴坐标，单位 m
    float y;      ///< 工具包络球体球心基于末端法兰坐标系的 Y 轴坐标，单位 m
    float z;      ///< 工具包络球体球心基于末端法兰坐标系的 Z 轴坐标，单位 m
}ToolEnvelope;

// 定义包络参数列表结构体
typedef struct{
    char tool_name[12];     ///< 控制器中已存在的工具坐标系名称，如果不存在该字段，则为临时设置当前包络参数
    ToolEnvelope list[5];       ///< 包络参数列表，每个工具最多支持 5 个包络球，可以没有包络
    int count;      ///< 包络球数量
}ToolEnvelopeList;

// 全局路点结构体
typedef struct
{
    char point_name[16];    ///< 全局路点的名称
    float joint[ARM_DOF];   ///< 全局路点的关节角度
    Pose pose;              ///< 全局路点的位置姿态
    char work_frame[12];    ///< 工作坐标系名称
    char tool_frame[12];    ///< 工具坐标系名称
    char time[20];          ///< 路点新增或修改时间
}Waypoint;

// 全局路点列表
typedef struct{
    int page_num;       ///< 页码（全部查询时此参数传NULL）
    int page_size;      ///< 每页大小（全部查询时此参数传NULL）
    int total_size;     ///< 列表长度
    char vague_search[32];  ///< 模糊搜索 （传递此参数可进行模糊查询）
    Waypoint points_list[100];   ///< 返回符合的全局路点列表
}WaypointsList;

typedef struct {
    const float* q_in;        ///< 上一时刻关节角度 单位°
    const Pose* q_pose;       ///< 目标位姿
    float q_out[7];       ///< 输出的关节角度 单位°
    uint8_t flag;       ///< 姿态参数类别：0-四元数；1-欧拉角
} IK_Params;

typedef struct {
    char project_path[300];      ///< 下发文件路径文件名
    int project_path_len;   ///< 名称长度
    int plan_speed;     ///< 规划速度比例系数
    int only_save;      ///< 0-保存并运行文件，1-仅保存文件，不运行
    int save_id;        ///< 保存到控制器中的编号
    int step_flag;      ///< 设置单步运行方式模式，1-设置单步模式 0-设置正常运动模式
    int auto_start;     ///< 设置默认在线编程文件，1-设置默认  0-设置非默认
    int project_type;   ///< 下发文件类型。0-在线编程文件，1-拖动示教轨迹文件
} Send_Project_Params;

typedef struct
{
    int joint_speed;   ///< 关节速度。1：上报；0：关闭上报；-1：不设置，保持之前的状态
    int lift_state;    ///< 升降关节信息。1：上报；0：关闭上报；-1：不设置，保持之前的状态
    int expand_state;  ///< 扩展关节信息（升降关节和扩展关节为二选一，优先显示升降关节）1：上报；0：关闭上报；-1：不设置，保持之前的状态
    int hand_state;    ///< 灵巧手状态。1：上报；0：关闭上报；-1：不设置，保持之前的状态
    int arm_current_status;    ///< 机械臂状态。1：上报；0：关闭上报；-1：不设置，保持之前的状态
    int aloha_state;    ///< aloha主臂状态是否上报。1：上报；0：关闭上报；-1：不设置，保持之前的状态
}UDP_Custom_Config;

/**
 * @brief 机械臂主动上报接口配置
 * @ingroup UdpConfig
 */
typedef struct {
    int cycle;      ///< 广播周期，5ms的倍数，-1：不设置，保持之前的状态
    bool enable;     ///< 使能，是否主动上报
    int port;       ///< 广播的端口号，-1：不设置，保持之前的状态
    int force_coordinate; ///< 系统外受力数据的坐标系，0为传感器坐标系 1为当前工作坐标系 2为当前工具坐标系（力传感器版本支持）-1代表不支持传感器
    char ip[28];       ///< 自定义的上报目标IP地址，空字符串代表不设置，保持之前的状态
    UDP_Custom_Config custom;       ///< 自定义项内容
} Realtime_Push_Config;


/**
 * @brief 复合模式拖动示教参数
 *
 */
typedef struct{
    int free_axes[6];       ///< 自由驱动方向[x,y,z,rx,ry,rz]，0-在参考坐标系对应方向轴上不可拖动，1-在参考坐标系对应方向轴上可拖动
    int frame;              ///< 参考坐标系，0-工作坐标系 1-工具坐标系。
    int singular_wall;      ///< 仅在六维力模式拖动示教中生效，用于指定是否开启拖动奇异墙，0表示关闭拖动奇异墙，1表示开启拖动奇异墙，若无配置参数，默认启动拖动奇异墙
}MultiDragTeach;

/**
 * @brief 力位混合控制参数
 *
 */
typedef struct
{
    int sensor;            ///< 传感器，0-一维力；1-六维力
    int mode;              ///< 0-基坐标系力控；1-工具坐标系力控；
    int control_mode[6];       ///< 6个力控方向的模式 0-固定模式 1-浮动模式 2-弹簧模式 3-运动模式 4-力跟踪模式 5-浮动+运动模式 6-弹簧+运动模式 7-力跟踪+运动模式 8-姿态自适应模式
    float desired_force[6];     ///< 力控轴维持的期望力/力矩，力控轴的力控模式为力跟踪模式时，期望力/力矩设置才会生效 ，单位N。
    float limit_vel[6];     ///< 力控轴的最大线速度和最大角速度限制，只对开启力控方向生效。
}ForcePosition;
/**
 * @brief 透传力位混合补偿参数
 *
 */
typedef struct
{
    int flag;          ///< 0-下发目标角度，1-下发目标位姿
    Pose pose;         ///< 当前坐标系下的目标位姿，支持四元数/欧拉角表示姿态。位置精度：0.001mm，欧拉角表示姿态，姿态精度：0.001rad，四元数方式表示姿态，姿态精度：0.000001
    float joint[ARM_DOF];       ///< 目标关节角度，单位：°，精度：0.001°
    int sensor;            ///< 传感器，0-一维力；1-六维力
    int mode;              ///< 0-基坐标系力控；1-工具坐标系力控；
    bool follow;            ///< 表示驱动器的运动跟随效果，true 为高跟随，false 为低跟随。
    int control_mode[6];       ///< 6个力控方向的模式 0-固定模式 1-浮动模式 2-弹簧模式 3-运动模式 4-力跟踪模式 5-浮动+运动模式 6-弹簧+运动模式 7-力跟踪+运动模式 8-姿态自适应模式
    float desired_force[6];     ///< 力控轴维持的期望力/力矩，力控轴的力控模式为力跟踪模式时，期望力/力矩设置才会生效 ，精度0.1N。
    float limit_vel[6];     ///< 力控轴的最大线速度和最大角速度限制，只对开启力控方向生效。
}ForcePositionMove;

typedef struct{
    float alpha;    //unit: deg
    float a;        //unit: m
    float d;        //unit: m
    float offset;   //unit: deg
}DHData;

typedef void (*RobotStatusListener)(RobotStatus data);
typedef void (*RM_Callback)(CallbackData data);

#define  M_PI_RAD    0.0174533f
#define  MI_PI_ANG   57.2957805f
#define  PI          3.14159f

#define  M_PI		 3.14159265358979323846
#define  DELTA       0.26f   //关节判断角度差
#define  DELTA2      2*PI    //关节运动到该处

// 机械臂型号
#define ARM_65      65
#define ARM_63_1    631
#define ARM_63_2    632
#define ARM_ECO65   651
#define ARM_75      75
#define ARM_ECO62   62
#define ARM_GEN72   72
#define ARM_ECO63   634


// 是否打印日志
#define RM_DISPLAY_LOG 0  // 0 不打印, 1打印
#define RM_NONBLOCK 0   // 非阻塞
#define RM_BLOCK 1      // 阻塞

#define RM_INPUT 0      // 输入
#define RM_OUTPUT 1     // 输出

#define RM_LOW 0        // 低
#define RM_TALL 1       // 高

#define PORT_CONTROLLER  0 // 控制器
#define PORT_ENDMODEL    1 // 末端接口板

#define NAVIGATION_MAGNETIC 0 // 磁条导航
#define OPING_CONTROLLER 1    // 开环控制模式

#define TRAJECTORY_FILE_NAME_MAX_LENGTH 300
//系统初始化错误代码
#define SYS_NORMAL                          0x0000          // 系统运行正常
#define CONTROLLER_DATE_RETURN_FALSE        0x0001          // 消息请求返回FALSE
#define INIT_MODE_ERR                       0x0002          // 机械臂未初始化或输入型号非法
#define INIT_TIME_ERR                       0x0003          // 非法超时时间
#define INIT_SOCKET_ERR                     0x0004          // Socket 初始化失败
#define SOCKET_CONNECT_ERR                  0x0005          // Socket 连接失败
#define SOCKET_SEND_ERR                     0x0006          // Socket 发送失败
#define SOCKET_TIME_OUT                     0x0007          // Socket 通讯超时
#define UNKNOWN_ERR                         0x0008          // 未知错误
#define CONTROLLER_DATA_LOSE_ERR            0x0009          // 数据不完整
#define CONTROLLER_DATE_ARR_NUM_ERR         0x000A          // 数组长度错误
#define WRONG_DATA_TYPE                     0x000B          // 数据类型错误
#define MODEL_TYPE_ERR                      0x000C          // 型号错误
#define CALLBACK_NOT_FIND                   0x000D          // 缺少回调函数
#define ARM_ABNORMAL_STOP                   0x000E          // 机械臂异常停止
#define TRAJECTORY_FILE_LENGTH_ERR          0x000F          // 轨迹文件名称过长
#define TRAJECTORY_FILE_CHECK_ERR           0x0010          // 轨迹文件校验失败
#define TRAJECTORY_FILE_READ_ERR            0x0011          // 轨迹文件读取失败
#define CONTROLLER_BUSY                     0x0012          // 控制器忙,请稍后再试
#define ILLEGAL_INPUT                       0x0013          // 非法输入
#define QUEUE_LENGTH_FULL                   0x0014          // 数据队列已满
#define CALCULATION_FAILED                  0x0015          // 计算失败
#define FILE_OPEN_ERR                       0x0016          // 文件打开失败
#define FORCE_AUTO_STOP                     0x0017          // 力控标定手动停止
#define DRAG_TEACH_FLAG_FALSE               0x0018          // 没有可保存轨迹
#define LISTENER_RUNNING_ERR                0x0019          // UDP监听接口运行报错

// 回调函数对应Code
#define MOVEJ_CANFD_CB                      0x0001          // 角度透传非阻塞标识码
#define MOVEP_CANFD_CB                      0x0002          // 位姿透传非阻塞标识码
#define FORCE_POSITION_MOVE_CB              0x0003          // 力位混合透传

#ifdef __cplusplus
}
#endif
#endif // RM_DEFINE_H
