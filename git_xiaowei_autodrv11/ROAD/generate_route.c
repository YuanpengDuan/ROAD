#include "generate_route.h"
#include "ntzx_qianxun_inv.h"
#include "ntzx_mc.h"
#include "ntzx_timer_drv.h"
#include "ntzx_file_drv.h"
#include "ntzx_conf_app.h"

#include<string.h>
#include<stdio.h>
#include<pthread.h>
#include<math.h>
#include<unistd.h>

#define ROUTEFILE "gps_load.txt"

static FILE *g_route = NULL;

#define deg_rad  (0.01745329252 )    // Transfer from angle degree to rad
#define R_LATI   (6378137)
#define R_LONT ( 5407872)  //这个需要根据所在区域的纬度进行换算：R_LONT = R_LATI*cos(所在地的纬度转化为弧度)

double route_x,route_y;
double lonti0=32.031678583;
double lati0=120.015370757;
double ntzx_GPSS_length(double lonti0,double lati0,double lonti2,double lati2)
{   
    double x,y,length;
    x = (R_LONT )*(lonti2-lonti0)*deg_rad;  //弧长公式94385.186690208  0.195 0.702
    y = (R_LATI)*(lati2-lati0)*deg_rad;//111319.49079363  0.00523  0.00322
    route_x=(R_LONT )*(lonti2-lonti0)*deg_rad;
    route_y=(R_LATI)*(lati2-lati0)*deg_rad;
    length = sqrt( x*x+y*y );
    return  length;

}


int main(int argc, char *argv[])
{ 
    int rt;  
    str_inv_to_pl nav_info = {0};
    str_inv_to_pl nav_info_last = {0};
    str_inv_to_pl nav_info_llast = {0};
    struct_mc_sta mc_sta = {0};
    ntzx_systimer route;
    Timer_Set(&route, 100000);

    /* 初始化配置信息 */
    rt = ntzx_conf_app_init();
    if (rt < 0) {
        printf("初始化配置信息failed\n");
        return -1;
    }
    /* 初始化千寻惯导 */
    rt = ntzx_qianxun_init();
    if (rt < 0) {
        printf("qianxun 初始化失败\n");
        return -1;
    }  
    /* 初始化MC底层 */
    rt = ntzx_mc_usart_init();
    if (rt < 0) {
        printf("MC底层初始话失败");
        return -1;
    }
    /* 初始化文件 */
    rt = ntzx_file_init(ROUTEFILE, &g_route, "w");
    if (g_route == NULL) {
        printf("打开创建文件失败\n");
    }
    pthread_t thread_mc; // 底层数据处理线程
    pthread_t thread_inertial_nv; // 导航（或惯导信息）处理线程

    pthread_create(&thread_inertial_nv, NULL, ntzx_qianxun_inv_main, NULL);
    pthread_create(&thread_mc, NULL, ntzx_mc_main, NULL);
    
    int i = 0;
    /* 采集数据 */
    char tmp[1000000];      //临时保存
    memset(tmp,0,sizeof(tmp));
    char *p = tmp;
    int start_to_break = 0;//当收到静止数据时退出保存
    while (1) {
        if (Timer_GetReached(&route)==0) 
        {
            Timer_Set(&route, 100000);  //采样频率，车速1m/s.
            if (NTZX_QIANXUN_SUCCESS == ntzx_get_inv_to_pl(&nav_info)) 
            {
                printf("千寻接受正确数据\n");
                // i++;
            }else{
                printf("等待千寻正确数据。\n");
                sleep(1);
                continue;
            }
            ntzx_get_mc_state(&mc_sta);//获取车辆底盘状态

            double distance = ntzx_GPSS_length(nav_info_llast.lon,nav_info_llast.lat,
            nav_info.lon,nav_info.lat);
            printf("distance = %f",distance);
            if(distance>=0.2)
            {
               //将路径点信息先保存到数据里面
            sprintf(p + strlen(tmp), "%.9f\n", nav_info.lat);
            sprintf(p + strlen(tmp), "%.9f\n", nav_info.lon);
            sprintf(p + strlen(tmp), "%.9f\n", nav_info.courseAngle);
            sprintf(p + strlen(tmp), "%.9f\n", nav_info.speedmm);
            sprintf(p + strlen(tmp), "%.9f\n", route_x);
            sprintf(p + strlen(tmp), "%.9f\n", route_y);

            //信息点打印查看
            printf("route_x:%.9f\t\n", route_x);
            printf("route_y:%.9f\t\n", route_y);
            printf("nav_info.Latitude_degree:%.9f\t\n", nav_info.lat);
            printf("nav_info.Longitude_degree:%.9f\t\n", nav_info.lon);
            printf("nav_info.courseAngle:%9f\t\n", nav_info.courseAngle);
            printf("mc_sta.speed_cm:%.9f\t\n", nav_info.speedmm);
            i++;
            printf("i = %d",i);
            nav_info_llast = nav_info_last;
            nav_info_last=nav_info;//保存当前数据到历史数据
            }
            
            //第一种情况：当车子静止时，准备计数并退出。
            // printf("nav_info.lat = %f，nav_info_llast.lat = %f，nav_info.lon = %f，nav_info_llast.lon = %f\n",
            // nav_info.lat,nav_info_llast.lat,nav_info.lon,nav_info_llast.lon);
            if( (fabs(nav_info.lat-nav_info_llast.lat)<=0.0000002)  && (fabs(nav_info.lon-nav_info_llast.lon)<=0.0000002) )
            {   
                start_to_break++;
                printf("开始停止计数：%d\t\n",start_to_break);
                if (start_to_break == 60){
                printf("确认停止，结束记录路径点！\n");
                break;  
                }         
            }else{
                start_to_break = 0;//清零
            }
            // nav_info_llast = nav_info_last;
            // nav_info_last=nav_info;//保存当前数据到历史数据           

        //第二种情况：当车子采集满点，退出。
             if (i ==1000)
            {
                printf("已经保存1000组数据，退出！\n");
                break;
            }
         }
    }
    printf("准备写入txt文档\n");
    fprintf(g_route, "%s",  tmp);
    fflush(g_route);
    fclose(g_route);
    printf("写入txt文档结束，按Ctrl +C退出！\n");


    pthread_join(thread_inertial_nv, NULL);
    pthread_join(thread_mc, NULL);
}

