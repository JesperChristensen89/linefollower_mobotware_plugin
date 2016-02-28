/* 
 * File:   LineFollower.h
 * Author: jesper
 *
 * Created on February 22, 2016, 6:14 PM
 */

#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include "uart.h"
#include <opencv2/core/core.hpp>

class LineFollower{
public:
    void start(cv::Mat, bool turnLeft, bool black, int errThresh);
private:
    void errorMatch(int, int);
    void errorContour(int);
};

#endif /* LINEFOLLOWER_H */