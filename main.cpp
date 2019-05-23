#include"ArmorFind.h"
#include "ArmorSet.h"
#include "anglesolve.h"
#include<RMVideoCapture.h>
#include <chrono>
#include "serial.h"
#include "graph.h"
#include "time.h"
#include <iostream>

using namespace std;
int main()
{
    //graph put
    //int num = 25;
    //Mat M_graph(750, 1200, CV_8UC3, Scalar(255, 255, 255));

    pthread_mutex_init(&mutex, NULL);
    pthread_t serial_tid;

    int err = pthread_create(&serial_tid, NULL, thread_serial, NULL);
    if(err!=0)
    {
      cout<<"failed"<<endl;
    }

    float yaw, pitch;
    int time_num;//储存图像处理的时间，单位为5ms
    Point2f center_point;
    AST::Mode = 1;  /*Mode =0;在红方时打开自瞄       Mode =1;在蓝方时打开自瞄*/
    AST::armormode = 1; /*mode =1;小装甲板：步兵，工程         mode=11;大装甲板：英雄，哨兵 */

    RMVideoCapture cap("/dev/video0",3);
    cap.setVideoFormat(640,480,1);
    cap.setVideoFPS(120);
    cap.setExposureTime(0,24);
    cap.startStream();
    cap.info();

    while (1)
    {
        const int64 start = getTickCount();
        Mat frame;
        Mat img_hsv, img_gray;
        Armorfind armor;
        vector<Point2f>  points;
        bool label=0;
        cap >> frame;

        points =armor.Armorfinds(frame, img_hsv, img_gray,label);
        double deal_time = (getTickCount()-start)/getTickFrequency();
        time_num = (int) (deal_time/0.005);
        time_num += ANGLE_BUFF_BASE;
        double duration =1/deal_time;
        cout<<time_num<<endl;
        cout<<"FPS: "<<duration<<endl;

        if (!points.empty())
        {
            center_point = angle_solve(points, yaw, pitch,AST::armormode);
            updata_angle(yaw, pitch, time_num);
        }


        if((char)waitKey(5)=='q')
       {
            cap.closeStream();
            pthread_mutex_lock(&mutex);
            serial_over = 0;
            pthread_mutex_unlock(&mutex);
            break;
       }
    }
    return 0;

}
