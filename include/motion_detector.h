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
#include "window.h"

namespace {
    cv::RNG rng(1);

    cv::Scalar randomColor() {
        return cv::Scalar(rng.uniform(0, 255),
                          rng.uniform(0, 255),
                          rng.uniform(0, 255));
    }

    cv::Rect enclosingRect(const cv::Rect& a,
                           const cv::Rect& b) {
        cv::Rect rect;
        rect.x = std::min(a.x, b.x);
        rect.y = std::min(a.y, b.y);
        rect.width = (std::max(a.x + a.width, b.x + b.width) - rect.x);
        rect.height = (std::max(a.y + a.height, b.y + b.height) - rect.y);
        return rect;
    }
}

class Marker
{
public:
    Marker():
        _grip(false),
        _kalman(4, 2, 0)
    {
        _kalman.statePre = cv::Mat::zeros(2, 1, CV_32F);
        cv::setIdentity(_kalman.measurementMatrix);
        cv::setIdentity(_kalman.processNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(_kalman.measurementNoiseCov, cv::Scalar::all(1));
        cv::setIdentity(_kalman.errorCovPost, cv::Scalar::all(.5));
    }

    void nextPosition(const cv::Point2f& pos,
                      const cv::Point2i& image_size) {
        nextPosition({ pos.x / image_size.x,
                       pos.y / image_size.y });
    }

    void nextPosition(const PercentPoint& pos) {
//        Logger::get("marker").debug("pos: %f %f", pos.x, pos.y);
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

    void grip() {
        Logger::get("marker").debug("grip");
        _grip = true;
    }

    void release() {
        Logger::get("marker").debug("release");
        _grip = false;
    }

    bool hasGrip() const {
        return _grip;
    }

private:
    cv::Point2f _last_position;
    bool _grip;
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

    cv::Rect _prev_enclosing_rect;
    std::deque<int> _last_frames_bb_area;

    cv::Rect getCalibrationRect(const cv::Size& image_size) const {
        const float HALF_WIDTH_PERCENT = 5;
        const float HALF_HEIGHT_PERCENT = 5;
        const int HALF_WIDTH_PX = (int)(image_size.width * HALF_WIDTH_PERCENT / 100);
        const int HALF_HEIGHT_PX = (int)(image_size.height * HALF_HEIGHT_PERCENT / 100);

        return cv::Rect(image_size.width / 2 - HALF_WIDTH_PX,
                        image_size.height / 2 - HALF_HEIGHT_PX,
                        HALF_WIDTH_PX * 2, HALF_HEIGHT_PX * 2);
    }

    void toggleCalibration() {
        switch (calibration_state) {
        case CalibrationState::NotCalibrating:
            calibration_state = CalibrationState::Preparing;
            break;
        case CalibrationState::Preparing:
            calibration_state = CalibrationState::Calibrating;
            break;
        default:
            break;
        }
    }

    void doCalibrate(const Image &frame) {
        cv::Rect rect = getCalibrationRect(frame.size());
        cv::Mat sample(frame, rect);
        _significant_color = cv::mean(sample, cv::Mat());
        calibration_state = CalibrationState::NotCalibrating;
        Logger::get("calibrate").debug("significant color = %d %d %d",
                                       _significant_color[0],
                                       _significant_color[1],
                                       _significant_color[2]);
    }

    void detectGrip(const cv::Point& marker_pos) {
        cv::Rect enclosing_rect = findEnclosingRect();
        if (_prev_enclosing_rect.contains(marker_pos)
                || enclosing_rect.contains(marker_pos)) {
            int area = enclosing_rect.area() / 10000;
            Logger::get("marker").debug("area = %d", area);
            _last_frames_bb_area.push_back(area);
            if (_last_frames_bb_area.size() > 5) {
                _last_frames_bb_area.pop_front();
            }

            int last = _last_frames_bb_area.front();
            bool ascending = true;
            bool descending = true;
            for (int val: _last_frames_bb_area) {
                if (val < last) {
                    ascending = false;
                } else if (val > last) {
                    descending = false;
                }
                last = val;
            }
            if (ascending) {
                _marker.release();
            } else if (descending) {
                _marker.grip();
            }
        }

        _prev_enclosing_rect = enclosing_rect;
    }

    void nextFrame(const Image &frame) {
        if (calibration_state == CalibrationState::Calibrating) {
            doCalibrate(frame);
        }

        _prev_frame = std::move(_curr_frame);
        _curr_frame = preprocessFrame(frame);

        if (!_curr_frame.empty() && !_prev_frame.empty()) {
            Image preprocessed = amplifyMotion(_prev_frame, _curr_frame);
            //            diff = diff.toGreyscale();

            //            cv::equalizeHist(diff, diff);
            //            cv::blur(diff, preprocessed, cv::Size(7, 7), cv::Point(0, 0), cv::BORDER_DEFAULT);

            _contours = getSignificantContours(preprocessed.toGreyscale());

            if (_contours.size()) {
                cv::Point marker_pos = _marker.getSmoothedPosition(frame.size());
                detectGrip(marker_pos);
            }

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

        bool show_bg = settings.show_background
               || calibration_state == CalibrationState::Preparing;

        if (!_curr_frame.empty()) {
            Image scaled_frame(ret.size(), _curr_frame.type());
            cv::resize(_curr_frame, scaled_frame, scaled_frame.size());
            ret += scaled_frame;
        }

//        if (show_bg) {
//            Image scaled_bg(ret.size(), background.type());
//            cv::resize(background, scaled_bg, scaled_bg.size());
//            ret += scaled_bg;
//        }

        if (calibration_state == CalibrationState::Preparing) {
            Image rect = cv::Mat::zeros(ret.size(), ret.type());
            cv::rectangle(rect, getCalibrationRect(ret.size()), cv::Scalar(255, 255, 255), 8, 2, 0);
            ret ^= rect;
        }

        drawDebugInfo(ret);

        settings.display(ret);
        return ret;
    }

    struct Settings {
        BoundedValue<uint8_t, 0, 255> motion_threshold = 200;
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

    enum class CalibrationState {
        NotCalibrating,
        Preparing,
        Calibrating
    };
    CalibrationState calibration_state = CalibrationState::Preparing;

private:
    Image _prev_frame;
    Image _curr_frame;

    Marker _marker;

    cv::Scalar _significant_color = cv::Scalar(0, 0, 0);

    Image preprocessFrame(const Image &image)
    {
        float THRESHOLD = 40;
        float GREYSCALE_THRESHOLD = 230;

        Image ret(image.size(), image.type());
        cv::absdiff(image, _significant_color, ret);
        cv::threshold(ret, ret, THRESHOLD, 255, cv::THRESH_BINARY_INV);

        Image mask(image.size(), CV_8UC1);
        cv::threshold(ret.toGreyscale(), mask, GREYSCALE_THRESHOLD, 255, cv::THRESH_BINARY);

        Image rgbMask(image.size(), image.type());
        cv::cvtColor(mask, rgbMask, cv::COLOR_GRAY2BGR);

        return image & rgbMask;
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

    cv::Rect findEnclosingRect() const {
        std::vector<cv::Rect> bounding_boxes;

        cv::Rect big_bb;
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

            if (big_bb.area() == 0) {
                big_bb = bb;
            } else {
                big_bb = enclosingRect(big_bb, bb);
            }
        }

        return big_bb;
    }

    void drawDebugContours(Image& out_image) const {
        /// Approximate contours to polygons + get bounding rects and circles
        std::vector<std::vector<cv::Point>> contours_poly;
        std::vector<cv::Rect> bounding_boxes;

        contours_poly.reserve(_contours.size());
        bounding_boxes.reserve(_contours.size());

        cv::Rect big_bb;
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

            if (big_bb.area() == 0) {
                big_bb = bb;
            } else {
                big_bb = enclosingRect(big_bb, bb);
            }

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(poly_mat, center, radius);

            contours_poly.emplace_back(std::move(poly));
            bounding_boxes.emplace_back(std::move(bb));
        }

        for (size_t i = 0; i < contours_poly.size(); ++i) {
            cv::Scalar color = randomColor();
            cv::drawContours(out_image, contours_poly, (int)i, color,
                             1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
            cv::rectangle(out_image,
                          bounding_boxes[i].tl(), bounding_boxes[i].br(),
                          color, 2, 8, 0 );
        }
    }

    void drawDebugInfo(Image& out_image) const {
        if (settings.show_debug_contours) {
            drawDebugContours(out_image);
        }

        cv::Rect enclosing_rect = findEnclosingRect();
        cv::rectangle(out_image, enclosing_rect.tl(), enclosing_rect.br(),
                      cv::Scalar(0, 255, 0), 2, 8, 0);

        cv::Scalar last_pos_color = _marker.hasGrip() ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);

        cv::circle(out_image, _marker.getSmoothedPosition(out_image.size()),
                   20, cv::Scalar(255, 255, 255), 2, 8, 0 );
        cv::circle(out_image, _marker.getLastPosition(out_image.size()),
                   20, last_pos_color, 2, 8, 0 );
    }

    Image amplifyMotion(const Image& prev_frame,
                        const Image& curr_frame) {
        Image diff = curr_frame - prev_frame;
//        cv::fastNlMeansDenoising(diff, diff, 15, 3, 3);

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
