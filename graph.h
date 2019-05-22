#ifndef GRAPH_H
#define GRAPH_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;


void graph(Mat src, float yaw0, float yaw1, float yaw_pre, int &num);

void flash_graph(Mat src);


#endif // GRAPH_H
