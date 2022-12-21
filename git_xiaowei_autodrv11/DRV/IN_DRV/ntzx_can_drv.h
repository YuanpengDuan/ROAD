/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_can_drv.h
 * 作者：任家豪
 * 日期：2021年2月1日
 * 描述：底层CAN的通信接口
 * ******************************************************************/

#ifndef _NTZX_CAN_DRV_H_
#define _NTZX_CAN_DRV_H_

#include <linux/can.h> //提供struct can_frame

#define NTZX_CAN_PARA_ERR           (-1) // 传入参数错误
#define NTZX_CREAT_CAN_SOCKET_ERR   (-2) // 创建CAN套接字失败
#define NTZX_IOCTL_CAN_SOCKET_ERR   (-3) // 指定CAN设备物理接口失败
#define NTZX_BIND_CAN_SOCKET_ERR    (-4) // 绑定CNA套接字失败
#define NTZX_READ_CAN_ERR           (-5) // 未读取到CAN数据

int ntzx_can_init(const char *ifname); // 初始化CAN套接字
int ntzx_can_send(int sfd, struct can_frame *frame); // 发送CAN报文
int ntzx_can_recv(int sfd, struct can_frame *frame); // 阻塞式接收CAN报文

#endif