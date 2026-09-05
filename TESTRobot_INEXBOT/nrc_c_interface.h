#ifndef INTERFACE_C_API_NRC_C_INTERFACE_H_
#define INTERFACE_C_API_NRC_C_INTERFACE_H_

#include "parameter/nrc_define.h"
#include "parameter/nrc_interface_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 连接控制器
 * @param ip 控制器ip,"192.168.1.13"
 * @param port 端口号,"6001"
 * @use 此函数是同步方式连接，因此函数会阻塞，直到返回连接结果。
 * @return -1-失败
 */
EXPORT_API SOCKETFD connect_robot_c(const char* ip,const char* port);

/**
* @brief 伺服清错
* @note 出错前如果时伺服运行状态，清错后需要手动进行下电操作，释放控制器的占用状态才可以继续上电（清错后不能直接上电，先下电再上电）
*/
EXPORT_API int clear_error_c(SOCKETFD socketFd);

/**
 * @brief 设置伺服状态
 * @param 0 停止 1 就绪
 * @warning 设置伺服就绪应该先确保系统没有错误 clear_servo_error(SOCKETFD socketFd)
 * 该函数只有伺服状态为0（停止状态）或1（就绪状态）时调用生效，伺服状态为2（报警状态）或3（运行状态）时不能直接设置伺服状态
 */
EXPORT_API int set_servo_state_c(SOCKETFD socketFd,int state);

/**
 * @brief 获取伺服状态
 * @param status 接收获取结果 0：停止状态 1：就绪状态 2：报警状态 3：运行状态
 */
EXPORT_API int get_servo_state_c(SOCKETFD socketFd, int& status);

/**
 * @brief 机器人上电
 * @attention 调用该函数之前需要先调用set_servo_state(SOCKETFD socketFd,1)将伺服设置为1（就绪状态）
 * 			,机器人上电成功后调用 get_servo_state(SOCKETFD socketFd)为3伺服运行状态
 * @return 机器人当前伺服状态servoStatus
 * 该函数只有伺服状态为1（就绪状态）时调用生效
 */
EXPORT_API int set_servo_poweron_c(SOCKETFD socketFd);

/**
 * @brief 机器人下电
 * @attention 机器人下电成功后调用 get_servo_state(SOCKETFD socketFd)为1伺服就绪状态
 * @return 机器人当前伺服状态servoStatus
 * 该函数只有伺服状态为3（运行状态）时调用生效
 */
EXPORT_API int set_servo_poweroff_c(SOCKETFD socketFd);

/**
 * @brief 获取机器人当前位置
 * @param[in] coord 指定需要查询的坐标的坐标系
 * @param[out] pos 存储返回结果点位的容器，长度7
 * @return result 函数调用结果
 */
EXPORT_API int get_current_position_c(SOCKETFD socketFd, int coord, double* pos);

/**
 * @brief 获取机器人当前所有关节点的直角坐标
 * @param axisNum 指定需要查询的关节
 * @param pos 存储返回结果点位的容器，长度7
 */
EXPORT_API int get_joint_position_c(SOCKETFD socketFd, int axisNum, double* pos);
/**
 * @brief 获取机器人外部轴当前位置
 * @param pos 点位数组，长度5
 */
EXPORT_API int get_current_extra_position_c(SOCKETFD socketFd, double* pos);

/**
 * @brief 设置当前模式的速度 有三种模式 示教模式，运行模式，远程模式
 * @param speed 速度，参数范围：0<speed≤100
 */
EXPORT_API int set_speed_c(SOCKETFD socketFd, int speed);

/**
 * @brief 设置机器人当前坐标系
 * @param coord 坐标系 0：关节 1：直角 2：工具 3：用户
 */
EXPORT_API int set_current_coord_c(SOCKETFD socketFd, int coord);

/**
 * @brief 获取机器人当前坐标系
 * @param coord 坐标系 0：关节 1：直角 2：工具 3：用户
 */
EXPORT_API int get_current_coord_c(SOCKETFD socketFd, int& coord);

/**
 * @brief 获取机器人运行状态
 * @param status 机器人运行状态
 *  - 0 停止
 *  - 1 暂停
 *  - 2 运行
 */
EXPORT_API int get_robot_running_state_c(SOCKETFD socketFd, int& status);

/**
 * @brief 原坐标值转换为其他坐标值
 * @param originCoord 原坐标系    0 1 2 3 关节 直角 工具 用户
 * @param originPos 要进行转换的坐标值 [0,1,2,3,4,5,6]
 *        关节取值范围    0-6[-10000,10000]
 *        直角取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 *        工具取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 *        用户取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 * @param targetCoord 目标坐标系  0 1 2 3 关节 直角 工具 用户
 * @param targetPos 转换后的坐标系
 */
EXPORT_API int get_origin_coord_to_target_coord_c(SOCKETFD socketFd, int originCoord, double* originPos, int targetCoord, double* targetPos);

/**
 * @brief 获取当前机器人DH参数
 * @param param 结构体参数
*/
EXPORT_API int get_robot_dh_param_c(SOCKETFD socketFd, RobotDHParam& param);

/**
 * @brief 获取当前机器人关节参数
 * @param id 关节参数[1,6]
 * @param param 结构体参数
 */
EXPORT_API int get_robot_joint_param_c(SOCKETFD socketFd, int id, RobotJointParam& param);

/**
 * @brief 关节运动
 * @param targetPosValue 点位数组，n个轴就赋值前n位数组,其余置0
 * @param vel 速度，参数范围：0<vel≤100 单位 %
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 * @param isSync 是否同步模式 true同步 false不同步
 */
EXPORT_API int robot_movej_c(SOCKETFD socketFd, int coord, double vel, double acc, double dec, double* targetPos);

/**
 * @brief 直线运动
 * @param targetPosValue 点位数组，n个轴就赋值前n位数组,其余置0
 * @param vel 速度，参数范围：0<vel≤1000 单位mm/s
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 * @param isSync 是否同步模式 true同步 false不同步
 */
EXPORT_API int robot_movel_c(SOCKETFD socketFd, int coord, double vel, double acc, double dec, double* targetPos);

#ifdef __cplusplus
}
#endif
#endif /* INTERFACE_C_API_NRC_C_INTERFACE_H_ */
