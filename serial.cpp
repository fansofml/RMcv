#include "serial.h"
#include "graph.h"

float_char angle_transform;//transform hex and float

//互斥锁
serial_data serial;

pthread_mutex_t mutex;

bool serial_over;

void * thread_serial(void *arg)
{
    pthread_mutex_lock(&mutex);
    serial_over = 1;
    pthread_mutex_unlock(&mutex);

    int num = 25;
    Mat M_graph(750, 1200, CV_8UC3, Scalar(255, 255, 255));
    namedWindow("graph");

   serial_data temp_serial;
   temp_serial.status = 0x00;
   temp_serial.color = 0x00;
   temp_serial.solve_angle[0] = 0;
   temp_serial.solve_angle[1] = 0;
   temp_serial.recive_angle[0] = 0;
   temp_serial.recive_angle[1] = 0;
   temp_serial.send_angle[0] = 0;
   temp_serial.send_angle[1] = 0;
   temp_serial.time_num = 20;

   RNG rng;
   const int stateNum=4;
   const int measureNum=2;
   KalmanFilter KF(stateNum, measureNum, 0);
   KF.transitionMatrix = (Mat_<float>(4,4) << 1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);
   setIdentity(KF.measurementMatrix);
   setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
   setIdentity(KF.measurementNoiseCov, Scalar::all(1e-2));
   setIdentity(KF.errorCovPost, Scalar::all(1));
   rng.fill(KF.statePost, RNG::UNIFORM,-10,10);
   Mat measurement = Mat::zeros(measureNum, 1, CV_32F);

   float yaw1, pitch1;//要发送的角度（未滤波）

   int fd = init_uart();//初始化串口

   char deal_buff[35];//存放要处理的字节
   int deal_num = 0;//存放要处理的字节数组里有效的字节个数
   char read_buff[20];//存放读到的字节
   int read_num = 0;//存放读到的字节个数
   char cut_buff[11];//临时储存被截断的数据字节
   int cut_num = 0;//被截断的字节个数
   int temp_num = 0;//储存字节中0xA5后面的字节的数量，包括0xA5，用于判断字节是否完整
   int recive_angle_buff_num = ANGLE_BUFF_NUM*2;
   vector<float> recive_angle_buff(recive_angle_buff_num, 0);

   //储存提取的数据，用于校验
   char temp_status[4];
   char temp_color[4];
   char temp_angle[11];

   while(1)
   {
       pthread_mutex_lock(&mutex);
       if(serial_over == 0)
       {
           break;
       }
       pthread_mutex_unlock(&mutex);

       const int64 start = getTickCount();
       //检查角度更新，若更新则需要发送
       pthread_mutex_lock(&mutex);
       if(temp_serial.solve_angle[0] != serial.solve_angle[0] || abs(temp_serial.solve_angle[1] != serial.solve_angle[1]) > 0.1)
       {
           KF.predict();
           temp_serial.solve_angle[0] = serial.solve_angle[0];
           temp_serial.solve_angle[1] = serial.solve_angle[1];
           temp_serial.time_num = serial.time_num;
           pthread_mutex_unlock(&mutex);

           if(temp_serial.time_num + 1 > ANGLE_BUFF_NUM)
           {
               temp_serial.time_num = ANGLE_BUFF_NUM;
           }

           yaw1 = temp_serial.recive_angle[0]+recive_angle_buff[temp_serial.time_num*2];
           pitch1 = temp_serial.recive_angle[1]+recive_angle_buff[temp_serial.time_num*2+1];
           cout<<temp_serial.time_num*2+1<<endl;

           measurement.at<float>(0) = yaw1;
           measurement.at<float>(1) = pitch1;
           KF.correct(measurement);

           //发送状态值
           temp_serial.send_angle[0] = KF.statePost.at<float>(0);
           temp_serial.send_angle[1] = KF.statePost.at<float>(1);
           send_angle(fd, temp_serial.send_angle);

           /*
           //发送预测值
           int code = 7;
           temp_serial.send_angle[0] = KF.statePost.at<float>(0)+code*KF.statePost.at<float>(2);
           temp_serial.send_angle[1] = KF.statePost.at<float>(1)+code*KF.statePost.at<float>(3);
           send_angle(fd, temp_serial.send_angle);
           */

           //graph(M_graph, temp_serial.recive_angle[0], temp_serial.send_angle[0], KF.statePost.at<float>(0), num);
           //imshow("graph", M_graph);
           //waitKey(3);
       }
       else
       {
           pthread_mutex_unlock(&mutex);
       }


       //cout << "recive:  yaw: " << temp_serial.recive_angle[0] << " pitch: " << temp_serial.solve_angle[0] << endl;
       //cout << "solve:  yaw: " << temp_serial.solve_angle[0]<< " pitch: " << temp_serial.solve_angle[1] << endl;
       //cout << "send:  yaw: " << temp_serial.send_angle[0] << " pitch: " << temp_serial.send_angle[0] << endl;
       //cout << "send:  yaw: " << yaw1 << " pitch: " << pitch1 << endl;


       //将读到的字节存进缓冲区，如果缓冲区有遗留字节则直接在后面加上
       read_num = read(fd, read_buff, 15);
       for(int i = 0; i < read_num; i++)
       {
           deal_buff[deal_num + i] = read_buff[i];
       }
       deal_num += read_num;

       //当缓冲区的字节大于3时开始处理
       if(deal_num > 3)
       {
           for(int i = 0; i < deal_num; i++)
           {
               if(deal_buff[i] == 0xA6)//判断数据标志
               {
                   if((i+1) < deal_num)//如果0xA6不是最后一个字节，则开始判断，如果是最后一个字节则储存
                   {
                       temp_num = deal_num - i;//0xA5到数组末尾的字节数，包括0xA5
                       //开始判断数据类型ID
                       switch (deal_buff[i+1]) {
                       case 0x01:
                           //判断数据是否完整
                           if(temp_num > 3)
                           {
                               //将这4个字节提取出来
                               temp_status[0] = deal_buff[i];
                               temp_status[1] = deal_buff[i+1];
                               temp_status[2] = deal_buff[i+2];
                               temp_status[3] = deal_buff[i+3];
                               //开始校验
                               if(check_sum(temp_status, 4) == 1)
                               {
                                   temp_serial.status = temp_status[2];
                               }
                           }
                           else
                           {
                               //如果不足一帧数据放到cut_buff储存
                               for(int j = 0; j < temp_num; j++)
                               {
                                   cut_buff[j] = deal_buff[i+j];
                               }
                               cut_num = temp_num;
                           }
                           break;
                       case 0x02:
                           //判断数据是否完整
                           if(temp_num > 3)
                           {
                               //将这4个字节提取出来
                               temp_color[0] = deal_buff[i];
                               temp_color[1] = deal_buff[i+1];
                               temp_color[2] = deal_buff[i+2];
                               temp_color[3] = deal_buff[i+3];
                               //开始校验
                               if(check_sum(temp_color, 4) == 1)
                               {
                                   temp_serial.color = temp_color[2];
                               }
                           }
                           else
                           {
                               //如果不足一帧数据放到cut_buff储存
                               for(int j = 0; j < temp_num; j++)
                               {
                                   cut_buff[j] = deal_buff[i+j];
                               }
                               cut_num = temp_num;
                           }
                           break;
                       case 0x03:
                           //判断数据是否完整
                           if(temp_num > 10)
                           {
                               //将这11个字节提取出来
                               for(int j = 0; j < 11; j++)
                               {
                                   temp_angle[j] = deal_buff[i+j];
                               }

                               //开始校验
                               if(check_sum(temp_angle, 11) == 1)
                               {
                                   cout<<"recive data"<<endl;
                                   //将16进制转化为float
                                   angle_transform.ch[0] = temp_angle[2];
                                   angle_transform.ch[1] = temp_angle[3];
                                   angle_transform.ch[2] = temp_angle[4];
                                   angle_transform.ch[3] = temp_angle[5];
                                   temp_serial.recive_angle[0] = angle_transform.fl;
                                   recive_angle_buff.insert(recive_angle_buff.begin()+0, angle_transform.fl);
                                   angle_transform.ch[0] = temp_angle[6];
                                   angle_transform.ch[1] = temp_angle[7];
                                   angle_transform.ch[2] = temp_angle[8];
                                   angle_transform.ch[3] = temp_angle[9];
                                   temp_serial.recive_angle[1] = angle_transform.fl;
                                   recive_angle_buff.insert(recive_angle_buff.begin()+1, angle_transform.fl);
                                   recive_angle_buff.resize(recive_angle_buff_num);
                               }
                           }
                           else
                           {
                               //如果不足一帧数据放到cut_buff储存
                               for(int j = 0; j < temp_num; j++)
                               {
                                   cut_buff[j] = deal_buff[i+j];
                               }
                               cut_num = temp_num;
                           }
                           break;
                       default:
                           break;
                       }
                   }
                   else
                   {
                       cut_buff[0] = deal_buff[i];
                       cut_num = 1;
                   }
               }

           }

           //更新serial结构体，存入刚读取的信息
           pthread_mutex_lock(&mutex);
           serial.status = temp_serial.status;
           serial.color = temp_serial.color;
           serial.recive_angle[0] = temp_serial.recive_angle[0];
           serial.recive_angle[1] = temp_serial.recive_angle[1];
           pthread_mutex_unlock(&mutex);

           //将被截断的数据储存到deal_buff中下一次处理
           for(int i = 0; i < cut_num; i++)
           {
               deal_buff[i] = cut_buff[i];
           }
           deal_num = cut_num;
           cut_num = 0;
       }
       double duration =(getTickCount()-start)/getTickFrequency()*1000;
       //cout<<duration<<endl;
   }
   close(fd);
   return (void*)0;
}

int init_uart()
{
   int fd = open(UART_DEVICE, O_RDWR|O_NOCTTY|O_NONBLOCK );
   if(-1 == fd)
   {
       cout<<"Can't open serial"<<endl;
       return -1;
   }

   struct termios  options;//创建一个修改保留串口数据的结构体
   tcgetattr(fd, &options);//用于获取与终端相关的参数，返回的结果保留在 termios 中
   tcflush(fd, TCIOFLUSH);//清空输入输出缓存区，TCIFLUSH输入缓存区   TCOFLUSH输出缓存区    TCIFLUSH输入输出缓存

   //set speed
   cfsetispeed(&options, B115200);//设置输入速度
   cfsetospeed(&options, B115200);//设置输出速度

   //set 数据位数
   options.c_cflag &= ~CSIZE;
   options.c_cflag |= CS8;

   //set parity
   options.c_cflag &= ~PARENB;   //禁止校验
   options.c_iflag &= ~INPCK;     //禁止输入校验

   //set 停止位
   options.c_cflag &= ~CSTOPB;//1

   //阻塞模式读操作下起作用
   options.c_cc[VTIME] =0; //  seconds 单位为百毫秒
   options.c_cc[VMIN] = 0;//等待读取字符的最小数量

   //modern
   options.c_lflag &=~ (ICANON|ECHO|ECHOE|ISIG);//取消规范模式 取消本地回显 收到信号字符不会处理
   options.c_cflag &=~(INLCR|ICRNL);//不转换回车和换行
   options.c_cflag &=~(IXON);
   options.c_oflag &=~OPOST;//不处理直接输出
   options.c_oflag &=~(ONLCR|OCRNL);//不转换回车和换行
   options.c_iflag &=~(ICRNL|INLCR);//不转换回车和换行
   options.c_iflag &=~(IXON|IXOFF|IXANY);
   options.c_cflag |=(CLOCAL|CREAD);

   tcsetattr(fd, TCSANOW, &options);

   return fd;
}

void updata_angle(float yaw, float pitch, int time_num)
{
    pthread_mutex_lock(&mutex);
    serial.solve_angle[0] = yaw;
    serial.solve_angle[1] = pitch;
    serial.time_num = time_num;
    pthread_mutex_unlock(&mutex);
 }

void get_angle(float &yaw, float &pitch)
{
   pthread_mutex_lock(&mutex);
   yaw = serial.recive_angle[0];
   pitch = serial.recive_angle[1] ;
   pthread_mutex_unlock(&mutex);
}

char get_status()
{
   char status;
   pthread_mutex_lock(&mutex);
   status = serial.status;
   pthread_mutex_unlock(&mutex);
   return status;
}

char get_color()
{
   char color;
   pthread_mutex_lock(&mutex);
   color = serial.color;
   pthread_mutex_unlock(&mutex);
   return color;
}

bool check_sum(char data[], int lenth)
{
   int sum = 0;
   for(int i = 0; i < lenth - 1; i++)
   {
       sum +=data[i];
   }
   char check = sum&0x01FF;
   if(check == data[lenth - 1])
   {
       return true;
   }
   else
   {
       return false;
   }
}

void send_angle(int fd, float s_angle[2])
{
   char send_char[11];

   send_char[0] = 0xA5;
   send_char[1] = 0x03;//angle坐标的ID

   angle_transform.fl = s_angle[0];
   send_char[2] = angle_transform.ch[0];
   send_char[3] = angle_transform.ch[1];
   send_char[4] = angle_transform.ch[2];
   send_char[5] = angle_transform.ch[3];

   angle_transform.fl = s_angle[1];
   send_char[6] = angle_transform.ch[0];
   send_char[7] = angle_transform.ch[1];
   send_char[8] = angle_transform.ch[2];
   send_char[9] = angle_transform.ch[3];

   //设置和校验位
   char sum = 0;
   for(int i = 0; i<10; i++)
   {
       sum +=send_char[i];
   }
   send_char[10] = sum&0xFF;
    write(fd, send_char, 11);
}

void send_reply(int fd, int id)
{
    char reply[4];
    reply[0] = 0XA5;
    if(id == 1)
    {
        reply[1]=0X01;
    }
    if(id==2)
    {
        reply[1]=0X02;
    }
    reply[2]=0X06;
    //设置和校验位
    int sum = 0;
    for(int i = 0; i<3; i++)
    {
        sum +=reply[i];
    }
    reply[3] = sum&0xFF;

    write(fd, reply, 4);

}
