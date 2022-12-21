/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_can_drv.c
 * 作者：任家豪
 * 日期：2021年2月1日
 * 描述：底层CAN的通信调用，包括初始化，接收与发送
 * ******************************************************************/
#include "ntzx_can_drv.h"

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>         //socket(), setsockopt()
#include <linux/can.h>          //struct can_frame
#include <linux/can/raw.h>      //SOL_CAN_RAW, CAN_RAW_FD_FRAMES
#include <sys/ioctl.h>          //ioctl()
#include <sys/time.h>           //fd_set
#include <pthread.h>            //提供线程函数
#include <fcntl.h>
#include <unistd.h>
#include <net/if.h>

/* 创建套接字 */
int ntzx_can_init(const char *ifname)
{
    int sfd;
    struct sockaddr_can addr;
    struct ifreq ifr;
    if (ifname == NULL) {
        return NTZX_CAN_PARA_ERR;
    }
    if((sfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) { //创建can套接字
        return NTZX_CREAT_CAN_SOCKET_ERR;
    }
    strcpy(ifr.ifr_name, ifname);
    if (-1 == ioctl(sfd, SIOCGIFINDEX, &ifr)) { //指定can设备
        return NTZX_IOCTL_CAN_SOCKET_ERR;
    }
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(sfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { //将套接字与can绑定
        close(sfd);
        return NTZX_BIND_CAN_SOCKET_ERR;
    }
    return sfd;
}

/* can发送 */
int ntzx_can_send(int sfd, struct can_frame *frame)
{
    int nbytes;
    if ((sfd < 0) || (frame == NULL)) {
        return NTZX_CAN_PARA_ERR;
    }
    nbytes = write(sfd, frame, sizeof(struct can_frame));
    return nbytes;
}

/* can接收 */
int ntzx_can_recv(int sfd, struct can_frame *frame)
{
    fd_set readSet;
    int nbytes;
    FD_ZERO(&readSet);
    FD_SET(sfd, &readSet);
    if (select((sfd + 1), &readSet, NULL, NULL,NULL) > 0) {
        if (FD_ISSET(sfd, &readSet)) {
            nbytes = read(sfd, frame, sizeof(struct can_frame));
            return nbytes;
        }
    }
    return NTZX_READ_CAN_ERR;
}