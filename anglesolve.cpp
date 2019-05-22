#include"anglesolve.h"

using namespace cv;
using namespace std;

//相机参数车上的

/*
double M_[3][3] =
{
    581.298989, 0, 322.9587759,
0, 579.0489285, 245.139366726,
0, 0, 1
};
*/
double M_[3][3] =
{
    582.8877, 0, 322.1580,
0, 581.8807, 241.1092,
0, 0, 1
};
double D_[1][5] =
{
    0.0440941876, -0.06114637, 0.0054152447, -0.00042233131, -0.0460863
};//不准

double fx = M_[0][0];
double fy = M_[1][1];
double cx = M_[0][2];
double cy = M_[1][2];

Point2f angle_solve(vector<Point2f> amor_4points, float &yaw, float &pitch, int armor_mode)
{
    double max_pixel_distance = point_distance(amor_4points[3], amor_4points[0]);
    double temp_distance;
    for(int i = 0; i < 3; i++)
    {
        temp_distance = point_distance(amor_4points[i], amor_4points[i+1]);
        if(temp_distance>max_pixel_distance)
        {
            max_pixel_distance = temp_distance;
        }
    }
    double distance = distance_solve(max_pixel_distance, armor_mode);
    //double distance = 1000;
    Point2f target_point = target_solve(distance);


    Point2f amor_point;//装甲板中心点
    amor_point.x = (amor_4points[0].x + amor_4points[1].x + amor_4points[2].x + amor_4points[3].x) / 4;
    amor_point.y = (amor_4points[0].y + amor_4points[1].y + amor_4points[2].y + amor_4points[3].y) / 4;

    yaw = atan((target_point.x - cx) / fx) - atan((amor_point.x - cx) / fx);
    pitch = atan((target_point.y - cy) / fy) - atan((amor_point.y - cy) / fy);

    yaw*=180/3.141592;
    pitch*=180/3.141592;
    return amor_point;
}

double distance_solve(double pixel_distance, int armor_mode)
{
    double object_distance;
    if(armor_mode == 1)
    {
        object_distance = SMALL_ARMOR_WIDTH;
    }
    else
    {
        object_distance = BIG_ARMOR_WIDTH;
    }
    //小孔成像原理测距
    double distance = (fx * object_distance) / pixel_distance;
    cout <<"distance:"<< distance << "mm" << endl;
    return distance;
}

Point2f target_solve(double distance)
{
    Point2f target_point;
    double camera_height = CAMERA_HEIGHT;
    double camera_fix_x = FIX_X;
    target_point.x = cx + camera_fix_x;
    target_point.y = cy +camera_height*fy/distance-5;

    return target_point;
}

double point_distance(Point2f p1, Point2f p2)
{
    double distance;
    distance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    return distance;
}
