#include "ArmorFind.h"
#include "ArmorSet.h"
#include <math.h>
#define hsv_threshold 70
#define big_armor_highest 4.85
#define big_armor_lowest 3
Armorfind::Armorfind()
{
}

Armorfind::~Armorfind()
{
}

/*
* @brief set the caps
*
*/


vector<Point2f> Armorfind::Armorfinds(Mat src, Mat img_hsv, Mat img_gray,bool &labels)
{

    namedWindow("picutre_text1");
    namedWindow("picutre_text2");
    //namedWindow("picutre_text3");
    //namedWindow("picutre_text4");
        Mat text(480, 640,CV_8UC1,Scalar(0));
        Mat text1(480, 640, CV_8UC1, Scalar(0));
        Mat text2(480, 640, CV_8UC1, Scalar(0));
        /*准备工作*/
        img_gray = p_img(src);
    findContours(img_gray, Armorcoutour, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    ArmorRects(src, Armorcoutour);//11ms
   RectArmorss(ArmorRect, text1);
   //Armorstrict(ArmorRect_real, ArmorRect,src);
    if (!Armorstrict(ArmorRect_real, ArmorRect,img_hsv))
    {
       Armorloose(ArmorRect_real, ArmorRect);
      cout << "loose" << endl;
    }
    //RectArmorss( RecChoose,text2);
    /*画出可能的装甲对（可注释）*/

    all_points =ALLpoint(RectChoose, text2);
  for (size_t i = 0; i < all_points.size(); i++)
    {
        Point2f vertices[4];
        vertices[0] = all_points[i][0];
        vertices[1] = all_points[i][1];
        vertices[2] = all_points[i][2];
        vertices[3] = all_points[i][3];
        for (int i = 0; i < 4; i++)
       line(text2, vertices[i], vertices[(i + 1) % 4], Scalar(255));
    }

    armor_points = high_chooses(all_points,labels);
    RectArmor(src, armor_points);
  // resize(src, src, Size(1280, 720));
  imshow("picutre_text1",src);
  imshow("picutre_text2", img_gray);
  //imshow("picutre_text3", text1);
  //imshow("picutre_text4", text2);
    armorclear();
    return armor_points;

    //return 0;
}




/***************************计算函数****************************************/
/*
* @brief 判断两点间的距离
*/
float Armorfind::distances(Point2f pt1, Point2f pt2)
{
    float distances = sqrt_fast(pow(abs(pt1.x - pt2.x), 2) + pow(abs(pt1.y - pt2.y), 2));
    return distances;
}
/*
* @brief 快速开方算法
*/
float Armorfind::sqrt_fast(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)& x; // get bits for floating VALUE
    i = 0x5f375a86 - (i >> 1); // gives initial guess y0
    x = *(float*)& i; // convert bits BACK to float
    x = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy
    return 1/x;
}
/*
* @brief 寻找灯条矩形
*/
/***************************功能函数****************************************/
/*
* @brief 预处理函数
*/
Mat Armorfind::p_img(Mat& src)
{

    Mat out;
    cvtColor(src, out, COLOR_BGR2GRAY);
    Mat element1, element2;

    threshold(out, out,hsv_threshold , 255, THRESH_BINARY);
    element1 = getStructuringElement(MORPH_RECT, Size(1, 5));
    erode(out, out, element1);
    element2 = getStructuringElement(MORPH_RECT, Size(5, 9));

    dilate(out, out, element2);

    return out;
}
vector<RotatedRect> Armorfind::ArmorRects(Mat src, vector<vector<Point>> Armorcoutours)//获得椭圆
{
    Scalar armorcolor;//gray
    RotatedRect s/*旋转矩阵*/, s_fitEllipse, s_minAreaRect;
    if (Armorcoutour.size() > 0)
    {
        int cout1 =0;
        for (auto& contour : Armorcoutours)
        {
            cout1 ++;
          bool Armorflags ;
            if (contour.size() > 10 && contour.size() < 300)//面积约束
            {
                s_fitEllipse = fitEllipse(contour);//椭圆
                s_minAreaRect = minAreaRect(contour);//最小外接旋转矩形
            //    double  lightrectarea = contourArea(contour);//旋转矩阵面积
                s = s_minAreaRect;
                s.angle = s_minAreaRect.angle;
                s.center = s_fitEllipse.center;
                Armorflags = colorchoose(src, Point(s.center));
                s.size.height = s_minAreaRect.size.height;
                s.size.width = s_minAreaRect.size.width;


                if (s.size.width / s.size.height > 0.1)
                {
                   if (s_fitEllipse.size.area() / s.size.area() > AST::light_min_areas)//面积和长度比的约束
                    {
                        if (s_fitEllipse.size.height > 5&&s_fitEllipse.size.height<100 )
                        {

                            if (Armorflags)
                            {
                                ArmorRect.push_back(s);
                            }
                        }

                    }
                }
            }
            if (cout1 > 500)
                break;
        }

      //  cout << ArmorRect.size() << endl;

    }
    return ArmorRect_real;
}
/*
* @brief 找到预设装甲对//严格匹配
* @param
* @return
*/
bool Armorfind::Armorstrict(vector<RotatedRect>  realarmor, vector<RotatedRect> ArmorRect,Mat& src)
{
//	float Aangle;
   // Point center_armor;

    size_t size = ArmorRect.size();
    if (size < 2)
    {
        return 0;
    }
    for (unsigned int NI = 0; NI < size; NI++)
        {
            for (unsigned int NL = NI; NL < size; NL++)
                 {
                    if (ArmorRect[NI].size.area() != ArmorRect[NL].size.area() || ArmorRect[NI].center != ArmorRect[NL].center)
                        {
                            if (ArmorRect[NI].size.area() > 10 && ArmorRect[NL].size.area() > 10)
                                {

                                    pair<RotatedRect, RotatedRect> realarmor;
                                    Point2f rect1[4];
                                    Point2f rect2[4];
                                    float dis_armor = distances(ArmorRect[NI].center, ArmorRect[NL].center);
                                    float dis_height = ( ArmorRect[NI].size.height + ArmorRect[NL].size.height) / 2;
                                    float dis_width = (ArmorRect[NI].size.width + ArmorRect[NL].size.width) / 2;
                                    float height_armor = max(dis_height, dis_width);
                                    float dis = max(dis_armor /height_armor,height_armor/dis_armor);
                                    double angle_dis =abs(atan((ArmorRect[NI].center.y- ArmorRect[NL].center.y) / (ArmorRect[NI].center.x - ArmorRect[NL].center.x))*180/PI);


                                    ArmorRect[NI].points(rect1);
                                    ArmorRect[NL].points(rect2);
                                    float area_size= ArmorRect[NI].size.area() /ArmorRect[NL].size.area() ;
                                    if(dis_armor>20)
                         {
                                   if(AST::Mode == 0)
                                    {
                                       if(AST::armormode == 1)
                                        {
                                           if (angle_dis<10)//平行度
                                           {
                                            if((dis>1.1&&dis<5.85))
                                              {
                                                if(area_size>0.2&&area_size<5)
                                                  {
                                                       realarmor.first = ArmorRect[NI];
                                                       realarmor.second = ArmorRect[NL];
                                                       RectChoose.push_back(realarmor);
                                                  }
                                              }
                                        }
                                       }
                                        else if(AST::armormode ==11)
                                        {
                                           if (angle_dis<10)//平行度
                                           {
                                              if((dis>3.1&&dis<7.1))
                                              {
                                                  if(area_size>0.4&&area_size<2.5)
                                                    {
                                                      realarmor.first = ArmorRect[NI];
                                                      realarmor.second = ArmorRect[NL];
                                                      RectChoose.push_back(realarmor);
                                                     }
                                              }
                                       }
                                       }
                                   }


                                   else  if(AST::Mode == 1)
                                    {

                                            if(AST::armormode == 1)
                                             {
                                                if (angle_dis<10)//平行度
                                                {
                                                 if((dis>1.1&&dis<5.85))
                                                   {
                                                     if(area_size>0.2&&area_size<5)
                                                       {
                                                            realarmor.first = ArmorRect[NI];
                                                            realarmor.second = ArmorRect[NL];
                                                            RectChoose.push_back(realarmor);
                                                       }
                                                   }
                                             }
                                            }
                                             else if(AST::armormode ==11)
                                             {
                                                if (angle_dis<15)//平行度
                                                {
                                                   if((dis>3.1&&dis<7.1))
                                                   {
                                                       if(area_size>0.4&&area_size<2.5)
                                                         {
                                                           realarmor.first = ArmorRect[NI];
                                                           realarmor.second = ArmorRect[NL];
                                                           RectChoose.push_back(realarmor);
                                                          }
                                                   }
                                            }
                                            }
                                        }

                          }
                            }
                        }
                }
        }

    if (RectChoose.empty())
        return 0;
    else
        return 1;
}

/*
* @brief 清除函数
* @
*/
void Armorfind::armorclear()
{
    Armorcoutour.clear();
    ArmorRect.clear();
    ArmorRect_real.clear();
    armor.clear();
//	armor_points.clear();
    all_points.clear();
}

/*
* @brief 画出矩形/调试
*/
void Armorfind::RectArmorss(vector<RotatedRect> Rect, Mat src)
{
    for (int i = 0; i < Rect.size(); i++)
    {
     //   Point2f vertex[4];
       // double mins = 100000;
        Point2f vertex1[4];

        Rect[i].points(vertex1);


        for (int i = 0; i < 4; i++)
        {
            line(src, vertex1[i], vertex1[(i + 1) % 4],255, 1, LINE_AA);
        }

    }
}

/*
* @brief 画出矩形/调试
*/
void Armorfind::RectArmor(Mat src,vector<Point2f>points)
{
    if (points.size() == 4)
    {
        for (int i = 0; i <4 ; i++)
        {
            line(src, points[i], points[(i + 1) % 4], Scalar(0, 255, 255), 1, 8, 0);
        }
    }
   else
        cout<<"nmsl"<<endl;
}

/*
* @brief 颜色选择层
*/
bool Armorfind::colorchoose(Mat src, Point it)
{
    if (it.x < 640 && it.y < 480)
    {
        Point pt = Point(abs(it.x), abs(it.y));

        int R, G, B;
        B = src.at<Vec3b>(pt)[0];
        G = src.at<Vec3b>(pt)[1];
        R = src.at<Vec3b>(pt)[2];
        if (AST::Mode == 0)
        {
            if ((B - R > 80))
            {
                return 1;
            }
            else
            {
                return 0;
     }
        }
        else if (AST::Mode == 1)
        {
            if( (R-B>40) )
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }


    else
        return 0;
}
}

/*
* @brief 返回所有矩形
* @brief 将矩形按12  分布
*                             03
*/
vector<vector<Point2f>> Armorfind::ALLpoint(vector<pair<RotatedRect, RotatedRect>> Rect,Mat&src)
{
    vector<vector<Point2f>> points_4;
    if (Rect.size() > 0)
    {
        for (size_t i = 0; i < Rect.size(); i++)
        {
            vector<Point2f> rect_points;
            vector<Point2f> rect_point4s;
            int width = src.rows;//480
            int height = src.cols;//640
            int left_top = 10000, left_down = 10000, right_top = 10000, right_down = 10000;
            Point2f vertex1[4];
            Point2f vertex2[4];
            Point2f vertex[4];
            Rect[i].first.points(vertex1);
            Rect[i].second.points(vertex2);
            for (int j = 0; j < 4; j++)
            {
                rect_points.push_back(vertex1[j]);
                rect_points.push_back(vertex2[j]);
            }
            if (rect_points.size() == 8)
            {
                for (size_t k = 0; k < rect_points.size(); k++)
                {
                    float distance_left_top = distances(rect_points[k], Point2f( 0,0));
                    float distance_left_down = distances(rect_points[k], Point2f(0, width));
                    float distance_right_top = distances(rect_points[k], Point2f(height,0));
                    float distance_right_down = distances(rect_points[k], Point2f(height, width));
                    while (distance_left_down < left_down)//左下角为1
                    {
                        left_down = distance_left_down;
                        vertex[0] = rect_points[k];
                    }
                    while (distance_left_top < left_top)//左上方为1
                    {
                        left_top = distance_left_top;
                        vertex[1] = rect_points[k];

                    }
                    while (distance_right_top < right_top)//右上方为2
                    {
                        right_top = distance_right_top;
                        vertex[2] = rect_points[k];
                    }
                    while (distance_right_down < right_down)//右下方为3
                    {
                        right_down = distance_right_down;
                        vertex[3] = rect_points[k];
                    }

                }
            }



            rect_point4s.push_back(vertex[0]);
            rect_point4s.push_back(vertex[1]);
            rect_point4s.push_back(vertex[2]);
            rect_point4s.push_back(vertex[3]);

            points_4.push_back(rect_point4s);
        }
    }
    return points_4;
}

/*
*@brief 宽松匹配，当严格匹配无法识别时进行补偿
*/
bool Armorfind::Armorloose(vector<RotatedRect>  realarmor, vector<RotatedRect> ArmorRect)
{
    float Aangle;
    Point center_armor;

    size_t size = ArmorRect.size();
    if (size < 2)
    {
        return 0;
    }
    for (unsigned int NI = 0; NI < size; NI++)
    {
        for (unsigned int NL = 0; NL < size; NL++)
        {

            if (ArmorRect[NI].size.area() != ArmorRect[NL].size.area())
            {
                   pair<RotatedRect, RotatedRect> realarmor;
              //  float dis_armor = distances(ArmorRect[NI].center, ArmorRect[NL].center);
             //   Point2f center = Point2f((ArmorRect[NI].center.x + ArmorRect[NL].center.x) / 2, (ArmorRect[NI].center.y + ArmorRect[NI].center.y) / 2);
               // float dis_light = (ArmorRect[NI].size.height + ArmorRect[NL].size.height) / 2;
              double angle_dis =abs(atan((ArmorRect[NI].center.y- ArmorRect[NL].center.y) / (ArmorRect[NI].center.x - ArmorRect[NL].center.x))*180/PI);
                //double dis_angle = abs(ArmorRect[NI].angle - ArmorRect[NL].angle);
                if (angle_dis < 5)
                {
                    realarmor.first = ArmorRect[NI];
                    realarmor.second = ArmorRect[NL];
                    RectChoose.push_back(realarmor);
                }
            }

        }

    }
    return 1;
}

/*dou
* @brief 图像处理层面抉择
*
*
*/
vector<Point2f> Armorfind::high_chooses(vector<vector<Point2f>> ptss,bool  &armor_label)//装甲板大小,距离,离鼠标的位置
{
    vector<Point2f> last_armor;
    float best_grade = 0;
    Point2f center_one(320,320);
   if (ptss.size() > 0)
    {
        int j=0;
            float Grade = 0;
        for (size_t i = 0; i < ptss.size(); i++)
        {
            cout << "第" << i << "个:" << ptss[i].size() << endl;
            if (ptss[i].size() == 4)
            {
     //           float rect_index = 0;//矩形参数
                float parallel_index = 0;//平行参数
                float area_index = 0;//mj参数
                float angle_index = 20;//长宽比参数
                float center_index =0;

        /*        /*矩形拟合参数*/
                float center_x = (ptss[i][0].x + ptss[i][1].x + ptss[i][2].x + ptss[i][3].x) / 4;
                float center_y = (ptss[i][0].y + ptss[i][1].y + ptss[i][2].y + ptss[i][3].y) / 4;
                Point armor_center = Point2f(center_x, center_y);
                center_index =distances(armor_center,center_one);
/*
                float center_distance[4];
                center_distance[0] = distances(armor_center, ptss[i][0]);
                center_distance[1] = distances(armor_center, ptss[i][1]);
                center_distance[2] = distances(armor_center, ptss[i][2]);
                center_distance[3] = distances(armor_center, ptss[i][3]);

                for (int first = 0; first < 4; first++)
                {
                    for (int sec = 0; sec < 4 - first; sec++)
                    {
                        rect_index += (center_distance[first] / center_distance[sec]) / 10;
                    }

                }
*/

                /*平行拟合参数*/
                float parallel_indexs[4];
                float height_01, height_23, width_12, width_30;
                parallel_indexs[0] = distances(ptss[i][0], ptss[i][1]);
                parallel_indexs[1] = distances(ptss[i][1], ptss[i][2]);
                parallel_indexs[2] = distances(ptss[i][2], ptss[i][3]);
                parallel_indexs[3] = distances(ptss[i][3], ptss[i][0]);
                height_01 = ptss[i][0].y - ptss[i][1].y;
                width_12 = ptss[i][2].x - ptss[i][1].x;
                height_23 = ptss[i][3].y - ptss[i][2].y;
                width_30 = ptss[i][3].x - ptss[i][0].x;
                parallel_index =min(min(width_12/height_01,width_12/height_23),min(width_30/height_01,width_30/height_23));

               float   angle_x1 =abs(atan( (ptss[i][0].y -ptss[i][3].y)/(ptss[i][0].x -ptss[i][3].x))*180/PI);
               float   angle_x2 =abs(atan( (ptss[i][1].y -ptss[i][2].y)/(ptss[i][1].x -ptss[i][2].x))*180/PI);
               float   angle_y1 =abs(atan( (ptss[i][1].x -ptss[i][0].x)/(ptss[i][1].y -ptss[i][0].y))*180/PI);
               float   angle_y2 =abs(atan( (ptss[i][2].x -ptss[i][3].x)/(ptss[i][2].y -ptss[i][3].y))*180/PI);
               angle_index =max(max(angle_x1,angle_x2),max(angle_y1,angle_y2));
                /*长宽比拟合参数*/
          /*      float radio = (parallel_indexs[1] + parallel_indexs[3]) / (parallel_indexs[0] + parallel_indexs[2]);
                if (radio > 1)
                {
                    lwr_index = abs(2.8 - radio);
                }
                else
                    lwr_index = 1;
                */
                area_index =parallel_indexs[0] *parallel_indexs[1]/4+parallel_indexs[1] *parallel_indexs[2]/4+parallel_indexs[2] *parallel_indexs[3]/4+parallel_indexs[3] *parallel_indexs[1]/4;

                Grade =area_index+
                center_index;
                 //   + 4 * (1 - abs(0.1 - lwr_index))
                    ;
            //    cout << "此装甲对的成绩为：" << Grade << endl;
              //  cout << rect_index << endl;//1.25
              //  cout << parallel_index << endl;//0.99
         //       cout << lwr_index << endl;//20

            if(AST::armormode ==1)
                {
                         if (Grade > best_grade)
                    {
                      if((parallel_index>1&&parallel_index<2.55))
                         {
                            if(angle_index<10)
                            {
                                 armor_label =0;
                                 j=i;
                                 best_grade = Grade;
                            }
                         }
                     }
                }
            else if (AST::armormode==11)
                   {
                  if (Grade > best_grade)
                     {
                       if((parallel_index>big_armor_lowest&&parallel_index<big_armor_highest))
                           {
                              if(angle_index<10)
                              {
                                   armor_label =0;
                                   j=i;
                                   best_grade = Grade;
                              }
                           }
                     }
                  }

        }
}

        if(best_grade !=0)
     last_armor = ptss[j];
    }
    return last_armor;
}


