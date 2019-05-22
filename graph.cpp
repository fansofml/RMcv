#include "graph.h"

void graph(Mat src, float yaw0, float yaw1, float yaw_pre, int &num)
{
    if(num > 1199)
    {
        num = 25;
    }

    if(num == 25)
    {
        flash_graph(src);
    }

    Point2f target_yaw0;
    target_yaw0.x = num;
    target_yaw0.y = 360 - yaw0 * 4;
    circle(src, target_yaw0, 1, Scalar(255, 0, 0), -1, CV_AA, 0);

    Point2f target_yaw1;
    target_yaw1.x = num;
    target_yaw1.y = 360 - yaw1 * 4;
    circle(src, target_yaw1, 1, Scalar(0, 255, 0), -1, CV_AA, 0);

    Point2f target_yaw_pre;
    target_yaw_pre.x = num;
    target_yaw_pre.y = 360 - yaw_pre * 4;
    circle(src, target_yaw_pre, 1, Scalar(0, 0, 255), -1, CV_AA, 0);

    num +=2;
}

void flash_graph(Mat src)
{
    src = Scalar(255, 255, 255);
    //XY
    line(src, Point(0, 360), Point(1199, 360), Scalar(0, 0, 0), 1);
    line(src, Point(25, 0), Point(25, 699), Scalar(0, 0, 0), 1);


}
