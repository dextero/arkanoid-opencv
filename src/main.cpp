#include <iostream>
#include <opencv2/opencv.hpp>
#include <ratio>
#include <functional>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

#include <signal.h>

#include "motion_detector.h"
#include "window.h"
#include "utils/message_queue.h"
#include "utils/fps_counter.h"
#include "utils/logger.h"

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
    CaptureThread():
        std::thread(&CaptureThread::run, this)
    {}

    void run() {
        cv::VideoCapture capture(-1);
//        capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
//        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
//        capture.set(CV_CAP_PROP_FPS, 60);

        Logger::get("capture").debug("starting");

        bool running = true;
        while (!force_stop && running) {
            Message msg;
            while (control.try_pop(msg)) {
                Logger::get("capture").debug("msg: %d", (int)msg.type);

                switch (msg.type) {
                case Message::Type::Exit: running = false; break;
                }
            }

            Logger::get("capture").debug("grabbing");
            Image frame;
            capture.read(frame);
            images.try_push(std::move(frame));
            Logger::get("capture").debug("frame");
        }

        Logger::get("capture").info("capture thread stopped");
    }

    message_queue<Image> images = {3};
    message_queue<Message> control = {25};
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
    MotionDetector detector(1920, 1080);

    CaptureThread capture;

    try {
        char key = 0;
        Image background(detector.width, detector.height, CV_8UC3, cv::Scalar(0, 100, 0));
        MotionDetector::Settings& settings = detector.settings;

        std::map<char, std::function<void(void)>> key_handlers;
        add_setting_adjuster(key_handlers, settings.motion_threshold, '1', 'q', (unsigned char)5);
        add_setting_adjuster(key_handlers, settings.min_bb_size,      '2', 'w', 2);
        key_handlers[' '] = [&settings](){ settings.show_background = !settings.show_background; };
        key_handlers['0'] = [&settings](){ settings.show_debug_contours = !settings.show_debug_contours; };
        key_handlers['c'] = [](){ Logger::toggle("capture"); };
        key_handlers['f'] = [](){ Logger::toggle("fps"); };
        key_handlers['t'] = [](){ Logger::toggle("threads"); };

        while (!force_stop && key != 27) {
            auto fpsGuard = fpsCounter.startNextFrame();
            Logger::get("fps").info("%lf FPS", fpsCounter.getFps());

            if (capture.images.try_pop(background)) {
                background.flip(Image::FlipAxis::Y);
                detector.nextFrame(background);
            }

            window.showImage(detector.toImage(background));

            key = (char) cvWaitKey(20);
            auto it = key_handlers.find(key);
            if (it != key_handlers.end()) {
                it->second();
            }
        }
    } catch (...) {

    }

    Logger::get("threads").info("exiting");
    capture.control.try_push(Message::exit());
    capture.join();

    return 0;
}