#ifndef FIND_OBJECTS_H
#define FIND_OBJECTS_H

#include "utils.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/types.hpp"

#include <stdlib.h>
#include <iostream>

namespace student {
    std::pair<int, int> getDigitFromRoi(cv::Mat &roi);

    int extrapolateVictimNumber(cv::Mat &roi);

    void getVictims(const cv::Mat &img_in, std::vector <std::pair<int, Polygon>> &victim_list, const double scale);

    void getObstacles(const cv::Mat &img_in, std::vector <Polygon> &obstacle_list, const double scale);

    bool getGate(const cv::Mat &img_in, Polygon &gate, const double scale);
}

#endif
