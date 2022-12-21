#include "ntzx_conf_app.h"
#include "ntzx_log_app.h"
#include "ntzx_qianxun_inv.h"
#include "ntzx_lidar_leishen.h"
#include "ntzx_fuse_data.h"
#include "ntzx_mc.h"
#include "ntzx_download_info.h"
#include "ntzx_planning_lane.h"
#include"ntzx_tools.h"

#include <stdio.h>
#include <pthread.h>

int main(int argc, char *argv[])
{
    int rt=0;
  printf("start\n");
    /* 初始化配置信息 */
    rt = ntzx_conf_app_init();
    if (rt < 0) {
        printf("初始化配置信息失败:%d\n",rt);
        return -1;
    }
    /* 初始化日志文件 */
    rt = ntzx_log_app_init();
    if (rt < 0) {
        printf("初始化日志文件失败！%d\n",rt);
        return -1;
    }
    /*下载路径点*/
    rt = ntzx_download_info_test();
    if (rt<0)
    {
        printf("读取路径文件文件失败！%d\n",rt);
        return -1;
    }
    
    str_ntzx_vehicle_waypoint road_test;
    str_ntzx_vehicle_waypoint road_temp;
    ntzx_get_vehicle_waypoint(&road_test);
    printf("\n**********%d\n",road_test.num);
    int pa,ch=0 ;
    road_temp.waypoint_info[0]=road_test.waypoint_info[0];
    for ( pa= 0; pa < (road_test.num -1); pa++)
    {   
        // printf("road_test.num=%d",road_test.num);
        double length = (double)ntzx_GPS_length(road_test.waypoint_info[pa].Longitude_degree,road_test.waypoint_info[pa].Latitude_degree,road_temp.waypoint_info[ch].Longitude_degree,road_temp.waypoint_info[ch].Latitude_degree);
        // printf("length=%f\n",length);
        if (length <0.9 )
        {
            continue;
        }
        else
        {   
            ch++;
            double proportion = 0.1/length; 
            road_temp.waypoint_info[ch].Longitude_degree = (road_test.waypoint_info[pa].Longitude_degree - road_temp.waypoint_info[ch-1].Longitude_degree)*proportion+road_temp.waypoint_info[ch-1].Longitude_degree;
            road_temp.waypoint_info[ch].Latitude_degree = (road_test.waypoint_info[pa].Latitude_degree - road_temp.waypoint_info[ch-1].Latitude_degree)*proportion+road_temp.waypoint_info[ch-1].Latitude_degree;   
            road_temp.waypoint_info[ch].courseAngle = road_test.waypoint_info[pa].courseAngle;
            road_temp.waypoint_info[ch].speed_cm =  road_test.waypoint_info[pa].speed_cm;
            pa--;//报段错误
        }
    }
    road_temp.num = ch;
    printf("sum = %d\n",ch);

    for (int i = 0; i < ch-1; i++)
    {    
        printf("!!!!!!!!!!!!");       
        //printf("Length = %.8f \t",ntzx_GPS_length(road_temp.waypoint_info[i].Longitude_degree,road_temp.waypoint_info[i].Latitude_degree,
        //road_temp.waypoint_info[i+1].Longitude_degree,road_temp.waypoint_info[i+1].Latitude_degree));
        // printf("Latitude_degree=%.8f，Longitude_degree=%.8f，courseAngle=%f，speedmm=%f\n",road_temp.waypoint_info[i].Latitude_degree,road_temp.waypoint_info[i].Longitude_degree,road_temp.waypoint_info[i].courseAngle,road_temp.waypoint_info[i].speed_cm);
    }

    // /* 创建各模块处理线程 */
    pthread_t thread_inertial_nv; // 导航（或惯导信息）处理线程
    // pthread_t thread_3d_lidar; // 激光雷达信息处理线程
    // pthread_t thread_fuse;  // 融合数据处理线程
    pthread_t thread_plan; // 规划数据处理线程
    pthread_t thread_mc;   // 底层数据处理线程

    /* 创建千寻惯导线程主程序 */
    rt = pthread_create(&thread_inertial_nv, NULL, ntzx_qianxun_inv_main, NULL);
    if (rt < 0) {
        ntzx_stereotypes_log_write_buf("main creat inv thread err");
        return -1;
    }
    // /* 创建镭神激光雷达的主程序 */
    // rt = pthread_create(&thread_3d_lidar, NULL, ntzx_lidar_leishen_main, NULL);
    // if (rt < 0) {
    //     ntzx_stereotypes_log_write_buf("main creat 3dlidar thread err");
    //     return -1;
    //  }
  
    // /* 创建fuse的线程主程序 */
    // rt = pthread_create(&thread_fuse, NULL, ntzx_fuse_data_main, NULL);
    // if (rt < 0) {
    //     ntzx_stereotypes_log_write_buf("main creat fuse thread err");
    //     return -1;
    // }
    /* 创建planning线程主程序 */
    rt = pthread_create(&thread_plan, NULL, ntzx_plan_lane_main, NULL);
    if (rt < 0) {
        ntzx_stereotypes_log_write_buf("main creat plan thread err");
        return -1;
    }
    /* 创建MC线程主程序 */
    rt = pthread_create(&thread_mc, NULL, ntzx_mc_main, NULL);
    if (rt < 0) {
        ntzx_stereotypes_log_write_buf("main creat mc thread err");
        return -1;
    }
    
    // /* 等待千寻惯导线程结束 */
    pthread_join(thread_inertial_nv, NULL);
    // pthread_join(thread_3d_lidar, NULL);
    // pthread_join(thread_fuse, NULL);
    pthread_join(thread_plan, NULL);
    pthread_join(thread_mc, NULL);
}




// pthread_t thread_download; // 激光雷达信息处理线程
  /* 从平台下发数据 */
    // rt = pthread_create(&thread_download, NULL, ntzx_download_info_main, NULL);
    // if (rt < 0) {
    //     ntzx_stereotypes_log_write_buf("main creat download thread err");
    //     return -1;
    // }
    // pthread_join(thread_download, NULL);

    

   

    
