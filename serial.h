#ifndef SERIAL_H
#define SERIAL_H

#include <iostream>
#include <stdio.h>  //标准输入输出定义
#include <stdlib.h>  //标准函数库定义
#include <unistd.h>  //Unix 标准函数定义
#include <sys/types.h>  //Unix/Linux系统的基本系统数据类型，含有size_t, pid_t等类型
#include <sys/stat.h>  //Unix/Linux系统定义文件状态所在的伪标准头文件
#include <fcntl.h>  //文件控制
#include <termios.h>  //PPSIX 终端控制定义
#include <errno.h>  //错误号定义
#include <pthread.h> //线程库   需要加LIB
#include <opencv2/opencv.hpp>

#define UART_DEVICE "/dev/ttyTHS1"
#define ANGLE_BUFF_NUM  50 //缓存角度的个数，读取的是之前的多少个的角度
#define ANGLE_BUFF_BASE 15 //缓存角度的基础个数，在这个数量上加上处理时间

using namespace std;
using namespace cv;

union float_char
{
    char ch[4];
    float fl;
};

struct serial_data
{
    char status;//机器人的状态  静默(0x00)和单目辅助瞄准(0x01) ID为0x01
    char color;//己方阵营颜色  红(0x01)和蓝(0x02) ID为0x02
    float recive_angle[2];//接受到的角度信息 ID为0x03
    float solve_angle[2];//解算出的角度，为相对值
    float send_angle[2];//要发送的角度信息 yaw,pitch
    int time_num;//图像处理的时间，单位为5ms
};
//互斥锁
extern bool serial_over;
extern serial_data serial;
extern pthread_mutex_t mutex;


void * thread_serial(void *arg);
/*@brief 线程回调函数
 * @arg 使用全局变量，不传入参数
 * @return void
 */

int init_uart();
/*@brief  打开并初始化串口
 *@return int 设备文件号，失败返回-1
 */

void updata_angle(float yaw, float pitch, int time_num);
/*@brief 将计算出的角度更新到结构体中
 * @yaw yaw轴的角度
 * @pitch pitch轴的角度
 */

void get_angle(float &yaw, float &pitch);
/*@brief 提取结构体中的yaw，pitch角度，也就是当前云台的角度
 * @yaw yaw轴的角度
 * @pitch pitch轴的角度
 */

char get_status();
/*@brief 提取结构体中的状态
 * @retrun char类型 状态的标志,0x01为启动辅助瞄准
 */

char get_color();
/*@brief 提取结构体中的颜色
 * @retrun char类型 状态的标志,0x01为红，0x02为蓝
 */

bool check_sum(char data[], int lenth);
/*@brief  对受到的字符数组进行校验
 * @param data[] 储存字符的数组
 * @param lenth 字符数组的长度，包括ID和校验位
 *@return bool 若校验成功返回true，失败返回false
 */

void send_angle(int fd, float s_angle[2]);
/*@brief 发送坐标位置，将三个float变为12个字符再加上1个ID输出
 * @param fd   类型 int  串口名称
 * @param XYZ   类型 float数组   要发送的一个坐标
*/

void send_reply(int fd, int id);
/*@brief 发送坐标位置，将三个float变为12个字符再加上1个ID输出
 * @param fd   类型 int  串口名称
 * @param id   类型 int  回复的类型，1为状态回复，2为颜色回复
*/

#endif // SERIAL_H
