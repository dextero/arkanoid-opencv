//
// Created by dex on 24.03.15.
//

#ifndef _PONG_IMAGE_H_
#define _PONG_IMAGE_H_

#include <opencv2/opencv.hpp>

class Image: public cv::Mat
{
public:
    template<typename... T>
    Image(T &&... args):
            cv::Mat(args...)
    { }

    static Image fromChannels(const Image& b,
                              const Image& g,
                              const Image& r)
    {
        assert(r.size == g.size);
        assert(r.size == b.size);
        assert(r.type() == CV_8UC1);
        assert(g.type() == CV_8UC1);
        assert(b.type() == CV_8UC1);

        Image rgb(r.size(), CV_8UC3);
        cv::insertChannel(r, rgb, 2);
        cv::insertChannel(g, rgb, 1);
        cv::insertChannel(b, rgb, 0);

        return rgb;
    }

    enum class FlipAxis
    {
        Both = -1,
        X = 0,
        Y = 1,
    };

    Image flipped(FlipAxis axis) const
    {
        Image ret(size(), type());
        cv::flip(*this, ret, (int) axis);
        return ret;
    }

    Image &flip(FlipAxis axis)
    {
        return *this = std::move(flipped(axis));
    }

    Image absdiff(const Image &other) const
    {
        Image diff(size(), type());
        cv::absdiff(*this, other, diff);
        return diff;
    }

    Image toGreyscale() const
    {
        assert(type() == CV_8UC3);

        Image ret(size(), type(), 1);
        cv::cvtColor(*this, ret, cv::COLOR_RGB2GRAY);
        return ret;
    }

    Image toColored() const
    {
        assert(type() == CV_8UC1);

        Image ret(size(), type(), 3);
        cv::cvtColor(*this, ret, cv::COLOR_GRAY2RGB);
        return ret;
    }

    std::array<Image, 3> toChannels() const
    {
        assert(type() == CV_8UC3);

        std::array<Image, 3> ret;
        cv::extractChannel(*this, ret[0], 0);
        cv::extractChannel(*this, ret[1], 1);
        cv::extractChannel(*this, ret[2], 2);

        return ret;
    }

    Image resized(const cv::Size &dst_size) const
    {
        Image ret(dst_size, type());
        cv::resize(*this, ret, dst_size);
        return ret;
    };

    inline Image resized(int width, int height) const
    {
        return resized({ width, height });
    }

    Image blurred(int kernel_size) const
    {
        Image ret;
        cv::blur(*this, ret, { kernel_size, kernel_size });
        return ret;
    }
};

#endif //_PONG_IMAGE_H_
