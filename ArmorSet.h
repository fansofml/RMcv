#ifndef ARMORSET_H
#define ARMORSET_H

#include <stdio.h>
#include <stdlib.h>

#include <iostream>

using namespace std
;
class AST
{

public:
    /*
    mode =1;小装甲板：步兵，工程
    mode=11;大装甲板：英雄，哨兵
    */
    static int armormode;
    /*
    Mode =0;在红方时打开自瞄
    Mode =1;在蓝方时打开自瞄
    */
    static int Mode;

    static float light_max_sizes;
        static float light_min_sizes;//灯条长宽比的值
    static float light_min_areas;//灯条外包矩形和轮廓的比值

   /*二值化参数*/
    static int h_min;
    static int h_max;
    static int r_min;
    static int r_max;
    static int s_min;
    static int s_max;
    static int v_min;
    static int v_max;
    static int g_min;
    static int g_max;
    static int b_min;
    static int b_max;
    /**/

};



#endif // !ARMORSET_H
