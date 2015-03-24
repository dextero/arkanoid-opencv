//
// Created by dex on 24.03.15.
//

#ifndef _PONG_MOTION_DETECTOR_H_
#define _PONG_MOTION_DETECTOR_H_

#include <opencv2/opencv.hpp>
#include <utils/logger.h>

#include "image.h"
#include "bounded_value.h"
#include "percent_point.h"

namespace {
    cv::RNG rng(1);

    cv::Scalar randomColor() {
        return cv::Scalar(rng.uniform(0, 255),
                          rng.uniform(0, 255),
                          rng.uniform(0, 255));
    }
}

class Marker
{
public:
    Marker():
        _kalman(4, 2, 0)
    {
        _kalman.statePre = cv::Mat::zeros(2, 1, CV_32F);
        cv::setIdentity(_kalman.measurementMatrix);
        cv::setIdentity(_kalman.processNoiseCov, cv::Scalar::all(1e-4));
        cv::setIdentity(_kalman.measurementNoiseCov, cv::Scalar::all(1));
        cv::setIdentity(_kalman.errorCovPost, cv::Scalar::all(.5));
    }

    void nextPosition(const cv::Point2f& pos,
                      const cv::Point2i& image_size) {
        nextPosition({ pos.x / image_size.x,
                       pos.y / image_size.y });
    }

    void nextPosition(const PercentPoint& pos) {
        Logger::get("marker").debug("pos: %f %f", pos.x, pos.y);
        _last_position = pos;
        update();
    }

    void update() {
        _kalman.correct(cv::Mat(_last_position));
    }

    cv::Point2f getSmoothedPosition(const cv::Point2i& image_size) const {
        const cv::Mat& prediction = _kalman.predict(cv::Mat());

        Logger::get("kalman").debug("kalman: %f %f",
                                    prediction.at<float>(0),
                                    prediction.at<float>(1));

        return { prediction.at<float>(0) * image_size.x,
                 prediction.at<float>(1) * image_size.y };
    }

    cv::Point2f getLastPosition(const cv::Point2i& image_size) const {
        return { _last_position.x * image_size.x,
                 _last_position.y * image_size.y };
    }

private:
    cv::Point2f _last_position;
    mutable cv::KalmanFilter _kalman;
};

class MotionDetector {
public:
    MotionDetector(size_t width,
                   size_t height):
        width(width),
        height(height)
    {
    }

    std::vector<std::vector<cv::Point>> _contours;

    std::vector<std::vector<cv::Point>> getSignificantContours(const Image& greyscale_image) const {
        std::vector<std::vector<cv::Point>> significant_contours;

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(greyscale_image, contours, hierarchy,
                         cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,
                         cv::Point(0, 0));

        for (auto& contour: contours) {
            cv::Mat contour_mat(contour);

            if (fabs(cv::contourArea(contour_mat, false)) >= settings.min_poly_area) {
                significant_contours.emplace_back(contour);
            }
        }

        return significant_contours;
    }

    void nextFrame(const Image &frame) {
        _prev_frame = std::move(_curr_frame);
        _curr_frame = preprocessFrame(frame);

        if (!_curr_frame.empty() && !_prev_frame.empty()) {
            Image preprocessed = amplifyMotion(_prev_frame, _curr_frame);
            //            diff = diff.toGreyscale();

            //            cv::equalizeHist(diff, diff);
            //            cv::blur(diff, preprocessed, cv::Size(7, 7), cv::Point(0, 0), cv::BORDER_DEFAULT);

            _contours = getSignificantContours(preprocessed.toGreyscale());

            cv::Point2f center;
            if (tryGetCenterPoint(_contours, center)) {
                _marker.nextPosition(center, preprocessed.size());
            } else {
                _marker.update();
            }
        }
    }

    Image toImage(const Image &background) const {
        Image ret = Image::zeros(height, width, CV_8UC3);

        if (settings.show_background) {
            Image scaled_bg(ret.size(), background.type());
            cv::resize(background, scaled_bg, scaled_bg.size());
            ret += scaled_bg;
        }

        if (settings.show_debug_contours) {
            debugDrawContours(ret);
        }

        settings.display(ret);
        return ret;
    }

    struct Settings {
        BoundedValue<uint8_t, 0, 255> motion_threshold = 250;
        BoundedValue<int, 0, 1000> min_poly_area = 300;
        bool show_background = true;
        bool show_debug_contours = true;

        void display(Image &image) const {
            Image textImage(image.size(), image.type(), cv::Scalar(0, 0, 0, 255));
            size_t line_num = 0;

#define DRAW_SETTING(Name) \
    drawTextLine(textImage, line_num++, #Name " = " + std::to_string(Name))
            DRAW_SETTING(motion_threshold);
            DRAW_SETTING(min_poly_area);
            DRAW_SETTING(show_background);
            DRAW_SETTING(show_debug_contours);
#undef DRAW_SETTING

            image ^= textImage;
        }

        void drawTextLine(Image& image,
                          size_t line_num,
                          const std::string& text) const {
            static const int FONT_FACE = cv::FONT_HERSHEY_PLAIN;
            static const double FONT_SCALE = 1.2;
            static const int THICKNESS = 1;
            static const cv::Scalar COLOR { 255, 255, 255, 255 };

            int baseline = 0;
            cv::Size text_size = cv::getTextSize(text, FONT_FACE, FONT_SCALE, THICKNESS, &baseline);
            baseline += THICKNESS;

            cv::Point text_origin(0, (text_size.height * (int)(line_num + 1)) + baseline);
            cv::putText(image, text, text_origin, FONT_FACE, FONT_SCALE, COLOR, THICKNESS);
        }
    };

    Settings settings;
    size_t width;
    size_t height;

private:
    Image _prev_frame;
    Image _curr_frame;

    Marker _marker;

    Image preprocessFrame(const Image &image) {
//        Image ret(image.cols / 2, image.rows / 2, image.type());
//        cv::resize(image, ret, ret.size());
//        cv::Sobel(image, ret, image.depth(), 1, 0, 3, 1.0, 1.0, cv::BORDER_REFLECT101);
        return image.clone();
    }

    static bool tryGetCenterPoint(const std::vector<std::vector<cv::Point>>& contours,
                                  cv::Point2f& out_point) {
        if (contours.size() < 1) {
            return false;
        }

        out_point.x = 0;
        out_point.y = 0;

        size_t num_points = 0;
        for (const auto& poly: contours) {
            for (const cv::Point& p: poly) {
                out_point.x += p.x;
                out_point.y += p.y;
            }

            num_points += poly.size();
        }

        out_point.x /= (float)num_points;
        out_point.y /= (float)num_points;
        return true;
    }

    void debugDrawContours(Image& out_image) const {
        /// Approximate contours to polygons + get bounding rects and circles
        std::vector<std::vector<cv::Point>> contours_poly;
        std::vector<cv::Rect> bounding_boxes;
        std::vector<cv::Point2f> circle_centers;
        std::vector<float> circle_radii;

        contours_poly.reserve(_contours.size());
        bounding_boxes.reserve(_contours.size());
        circle_centers.reserve(_contours.size());
        circle_radii.reserve(_contours.size());

        cv::Point2f scale((float)width / _curr_frame.cols,
                          (float)height / _curr_frame.rows);
        for (const auto& contour: _contours) {
            std::vector<cv::Point> poly;
            cv::approxPolyDP(cv::Mat(contour), poly, 3, true);

            for (cv::Point& p: poly) {
                p.x *= scale.x;
                p.y *= scale.y;
            }

            cv::Mat poly_mat(poly);
            cv::Rect bb = cv::boundingRect(poly_mat);

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(poly_mat, center, radius);

            contours_poly.emplace_back(std::move(poly));
            bounding_boxes.emplace_back(std::move(bb));
            circle_centers.emplace_back(std::move(center));
            circle_radii.emplace_back(radius);
        }

        for (size_t i = 0; i < contours_poly.size(); ++i) {
            cv::Scalar color = randomColor();
            cv::drawContours(out_image, contours_poly, (int)i, color,
                             1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
            cv::rectangle(out_image,
                          bounding_boxes[i].tl(), bounding_boxes[i].br(),
                          color, 2, 8, 0 );
//            cv::circle(drawing, circle_centers[i], (int)circle_radii[i],
//                       color, 2, 8, 0);
        }

        cv::circle(out_image, _marker.getSmoothedPosition(out_image.size()),
                   20, cv::Scalar(255, 255, 255), 2, 8, 0 );
        cv::circle(out_image, _marker.getLastPosition(out_image.size()),
                   20, cv::Scalar(0, 0, 255), 2, 8, 0 );
    }

    Image amplifyMotion(const Image& prev_frame,
                        const Image& curr_frame) {
        Image diff = curr_frame - prev_frame;
        cv::fastNlMeansDenoising(diff, diff, 15, 3, 3);

        Image r, g, b;
//            std::tie(b, g, r) = unpack(diff.toChannels());
        cv::extractChannel(diff, r, 2);
        cv::extractChannel(diff, g, 1);
        cv::extractChannel(diff, b, 0);

        cv::equalizeHist(r, r);
        cv::equalizeHist(g, g);
        cv::equalizeHist(b, b);

        Image rgb(diff.size(), diff.type());
        cv::insertChannel(r, rgb, 2);
        cv::insertChannel(g, rgb, 1);
        cv::insertChannel(b, rgb, 0);

        Image preprocessed;
        cv::threshold(rgb.toGreyscale().toColored(), preprocessed,
                      settings.motion_threshold, 255, cv::THRESH_BINARY);

        return preprocessed;
    }
};

#endif //_PONG_MOTION_DETECTOR_H_
