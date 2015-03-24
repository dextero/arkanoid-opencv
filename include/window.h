//
// Created by dex on 24.03.15.
//

#ifndef _PONG_WINDOW_H_
#define _PONG_WINDOW_H_

#include <opencv2/opencv.hpp>

class Window
{
public:
    Window(const std::string& name):
        _name(name)
    {
        cv::namedWindow(_name.c_str(), cv::WINDOW_AUTOSIZE);
    }

    Window(const Window&) = delete;
    Window& operator =(const Window&) = delete;

    Window(Window&& old) {
        *this = std::move(old);
    }
    Window& operator =(Window&& old) {
        _name = std::move(old._name);
        old._name.clear();
        return *this;
    }

    ~Window() {
        if (!_name.empty()) {
            cv::destroyWindow(_name.c_str());
        }
    }

    void showImage(const cv::Mat& image) {
        cv::imshow(_name.c_str(), image);
    }

private:
    std::string _name;
};

#endif //_PONG_WINDOW_H_
