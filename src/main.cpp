#include <iostream>
#include <opencv2/opencv.hpp>
#include <ratio>
#include <functional>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

#include <signal.h>
#include <pong/pong.h>
#include <utils/timer.h>

#include "motion_detector.h"
#include "window.h"
#include "utils/message_queue.h"
#include "utils/fps_counter.h"
#include "utils/logger.h"
#include "utils/misc.h"

std::atomic<bool> force_stop(false);

void signal_handler(int sig) {
    Logger::get("signal").info("got signal: %d (%s)", sig, strsignal(sig));
    force_stop.store(true);
}

struct Message {
    enum class Type {
        Exit,
    };

    Type type;

    static Message exit() { return { Type::Exit }; }
};

class CaptureThread: public std::thread
{
public:
    std::atomic<bool> running;

    CaptureThread():
        std::thread(),
        running(true)
    {
        std::thread actual_thread(&CaptureThread::run, this);
        swap(actual_thread);
    }

    void run() {
        cv::VideoCapture capture(-1);
//        capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
//        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
//        capture.set(CV_CAP_PROP_FPS, 60);

        Logger::get("capture").debug("starting");

        if (capture.isOpened()) {
            Logger::get("capture").info("capture initialized");
        } else {
            Logger::get("capture").error("could not initialize capture");
            running = false;
        }

        while (!force_stop && running) {
            Logger::get("capture").trace("grabbing");
            Image frame;
            if (!capture.read(frame)) {
                Logger::get("capture").error("read failed");
                running = false;
            }

            images->try_push(std::move(frame));
            Logger::get("capture").trace("frame");
        }

        Logger::get("capture").info("capture thread stopped");
    }

    std::shared_ptr<message_queue<Image>> images = std::make_shared<message_queue<Image>>(3);
};

class DetectorThread: public std::thread
{
public:
    std::atomic<bool> running;

    DetectorThread(size_t width,
                   size_t height,
                   const std::shared_ptr<message_queue<Image>>& capture):
        running(true),
        _capture(capture),
        _toggle_calibration(false)
    {
        std::thread actual_thread(&DetectorThread::run, this, width, height);
        swap(actual_thread);
    }

    void toggleCalibration()
    { _toggle_calibration = true; }

    void run(size_t width,
             size_t height)
    {
        MotionDetector detector(width, height);
        Image background;

        while (!force_stop && running) {
            if (_capture->try_pop(background)) {
                background.flip(Image::FlipAxis::Y);
                detector.nextFrame(background);
            }

            if (_toggle_calibration) {
                _toggle_calibration = false;
                detector.toggleCalibration();
            }

            MotionDetector::Settings settings;
            if (setting_changes->try_pop(settings)) {
                detector.settings = settings;
            }

            images->try_push(detector.toImage(background));
            marker_positions->try_push(detector.getMarkerPos());
        }
    }

    std::shared_ptr<message_queue<Image>> images = std::make_shared<message_queue<Image>>(3);
    std::shared_ptr<message_queue<cv::Point2f>> marker_positions = std::make_shared<message_queue<cv::Point2f>>(3);
    std::shared_ptr<message_queue<MotionDetector::Settings>> setting_changes = std::make_shared<message_queue<MotionDetector::Settings>>(3);

private:
    std::shared_ptr<message_queue<Image>> _capture;
    std::atomic<bool> _toggle_calibration;
};

template<typename T, T Min, T Max>
void add_setting_adjuster(std::map<char, std::function<void(void)>>& key_handlers,
                          BoundedValue<T, Min, Max>& setting,
                          char key_add,
                          char key_subtract,
                          T step) {
    key_handlers.insert(std::make_pair(key_add, [&setting, step](){
        setting += step;
    }));

    key_handlers.insert(std::make_pair(key_subtract, [&setting, step](){
        setting -= step;
    }));
}

int main() {
    for (int sig: { SIGINT, SIGTERM, SIGABRT, SIGHUP }) {
        signal(sig, signal_handler);
    }

    Window window("pong");
    FpsCounter fpsCounter;

    const size_t WIDTH = 1440;
    const size_t HEIGHT = 900;

    CaptureThread capture;
    DetectorThread detector(WIDTH, HEIGHT, capture.images);
    Game pong(WIDTH, HEIGHT);

    try {
        char key = 0;
        MotionDetector::Settings settings;

        std::map<char, std::function<void(void)>> key_handlers;
        add_setting_adjuster(key_handlers, settings.motion_threshold, '1', 'q', (unsigned char)5);
        add_setting_adjuster(key_handlers, settings.min_poly_area,    '2', 'w', 5);
        add_setting_adjuster(key_handlers, settings.blurred_threshold, '3', 'e', (unsigned char)5);
        key_handlers['a'] = [&settings](){ settings.show_background = !settings.show_background; };
        key_handlers['s'] = [&settings](){ settings.show_debug_contours = !settings.show_debug_contours; };
        key_handlers['d'] = [&settings](){ settings.show_marker_pos = !settings.show_marker_pos; };
        key_handlers['f'] = [&settings](){ settings.show_debug_frame = !settings.show_debug_frame; };
        key_handlers['c'] = [](){ Logger::toggle("capture"); };
        key_handlers['z'] = [](){ Logger::toggle("fps"); };
        key_handlers['x'] = [](){ Logger::toggle("threads"); };
        key_handlers['c'] = [](){ Logger::toggle("kalman"); };
        key_handlers['v'] = [](){ Logger::toggle("marker"); };
        key_handlers[' '] = [&detector] { detector.toggleCalibration(); };

        Logger::get("capture").set_log_level(Logger::LogLevel::DEBUG);
        Logger::disable("capture");
        Logger::disable("kalman");
        Logger::disable("fps");

        Timer timer;
        double dt = -3.0;
        const double UPDATE_STEP_S = 1.0 / 60.0;

        Image background(WIDTH, HEIGHT, CV_8UC3);

        while (!force_stop && key != 27) {
            auto fpsGuard = fpsCounter.startNextFrame();
            Logger::get("fps").info("%lf FPS", fpsCounter.getFps());

            cv::Point2f marker_pos;
            if (detector.marker_positions->try_pop(marker_pos)) {
                pong.setPaddlePos((size_t)marker_pos.x);
            }

            dt += timer.getElapsedSeconds();
            timer.reset();
            while (dt > UPDATE_STEP_S) {
                pong.update((float)UPDATE_STEP_S);
                dt -= UPDATE_STEP_S;
            }

            if (pong.isGameOver() || pong.isGameWon()) {
                pong.reset();
            }

            detector.images->try_pop(background);
            pong.drawOnto(background);
            window.showImage(background);

            key = (char) cvWaitKey(20);
            auto it = key_handlers.find(key);
            if (it != key_handlers.end()) {
                it->second();
                detector.setting_changes->try_push(MotionDetector::Settings(settings));
            }
        }
    } catch (...) {

    }

    Logger::get("threads").info("exiting");
    capture.running = false;
    detector.running = false;
    capture.join();
    detector.join();

    return 0;
};
