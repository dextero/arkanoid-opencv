//
// Created by dex on 24.03.15.
//

#ifndef _ARKANOID_PERCENT_POINT_H_
#define _ARKANOID_PERCENT_POINT_H_

#include <opencv2/opencv.hpp>
#include <cassert>

struct PercentPoint: public cv::Point2f {
    PercentPoint():
            cv::Point2f(0, 0)
    {
    }

    PercentPoint(float x, float y):
            cv::Point2f(x, y)
    {
        assert(0 <= x && x <= 1
               && 0 <= y && y <= 1);
    }

    PercentPoint(const cv::Point2f& point,
                 const cv::Point2f& scale):
            cv::Point2f(point.x * scale.x,
                        point.y * scale.y)
    {
    }

    PercentPoint operator +(const PercentPoint& rhs) {
        return { x + rhs.x, y + rhs.y };
    }
    PercentPoint& operator +=(const PercentPoint& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    PercentPoint operator -(const PercentPoint& rhs) {
        return { x - rhs.x, y - rhs.y };
    }
    PercentPoint& operator -=(const PercentPoint& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    PercentPoint operator *(float rhs) {
        return { x * rhs, y * rhs };
    }
    PercentPoint& operator *=(float rhs) {
        x *= rhs;
        y *= rhs;
        return *this;
    }

    PercentPoint operator /(float rhs) {
        return { x / rhs, y / rhs };
    }
    PercentPoint& operator /=(float rhs) {
        x /= rhs;
        y /= rhs;
        return *this;
    }
};

#endif //_ARKANOID_PERCENT_POINT_H_
