#ifndef ArmorFind_H
#define ArmorFind_H

#include <iostream>
#include <opencv.hpp>
#include <vector>
#include <math.h>
#include<chrono>
#include <string>
#include<algorithm>
#include"ArmorSet.h"
using namespace std;
using namespace cv;



class Armorfind
{

public:
    Point2f armorpoint[4];
    vector<Point3f> xyzpoint;

private:


    vector<vector<Point> >ArmorcoutourRGB;
    vector<vector<Point> >Armorcoutour;//轮廓
    vector<RotatedRect> ArmorRect;//旋转矩阵

    vector< RotatedRect> ArmorRect_real;//装甲板灯条旋转矩阵对
    vector<pair<RotatedRect, RotatedRect>> RectChoose;//打击的装甲板矩阵
    vector<Rect> armor;//装甲板矩形
    vector<Point2f> armor_points;
    vector<vector<Point2f>> all_points;
    vector<Point2f> last_armor;
    const double PI = 3.142857142857;
public:
    vector<Point2f> Armorfinds(Mat src, Mat img_hsv, Mat img_gray,bool &labels);//	RGB二值化
    void armorclear();
    vector<Point2f>  high_choose(vector<RotatedRect>  real,Mat& src);	//获得装甲板的bestchoose
    Armorfind();
    ~Armorfind();
private:

    void hsv_threold(Mat& src, Mat& dst);
    Mat p_img(Mat& src);
    bool colorchoose(Mat src, Point it);//根据颜色进行颜色决策
    void RectArmor(Mat src,vector<Point2f>points);//画矩形
    bool Armorstrict(vector<RotatedRect>  realarmor, vector<RotatedRect> ArmorRect,Mat& src);//得到装甲板灯条旋转矩阵对
    bool Armorloose(vector<RotatedRect>  realarmor, vector<RotatedRect> ArmorRect);
    vector<RotatedRect> ArmorRects(Mat src, vector<vector<Point>> Armorcoutours);//窗口改变
    void RectArmorss(vector< RotatedRect> Rect, Mat src);
    vector<vector<Point2f>> ALLpoint(vector<pair<RotatedRect, RotatedRect>> Rect,Mat &src);
    float sqrt_fast(float x);//快速开方算法
    float distances(Point2f pt1, Point2f pt2);
    vector<Point2f>  high_choose(vector<vector<Point2f>> ptss);	//获得装甲板的bestchoose
    vector<Point2f> high_chooses(vector<vector<Point2f>> ptss,bool &armor_label);
    bool Armorstrict_rect(Mat src, vector<RotatedRect> ArmorRect);
};


#endif
