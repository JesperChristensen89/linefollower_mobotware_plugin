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
    bool start(cv::Mat, UART, bool, bool, int);
private:
    void errorMatch(int, int);
    void errorContour(int);
};

#endif /* LINEFOLLOWER_H */