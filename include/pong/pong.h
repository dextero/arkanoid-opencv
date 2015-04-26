//
// Created by dex on 26.04.15.
//

#ifndef PONG_PONG_H
#define PONG_PONG_H

#include <opencv2/core/core.hpp>
#include <utils/array_2d.h>
#include <image.h>
#include <stdint.h>
#include <utils/logger.h>

struct MovingObject
{
    MovingObject(const cv::Point2f& pos,
                 const cv::Point2f& v):
        position(pos),
        velocity(v)
    {}

    cv::Point2f position;
    cv::Point2f velocity;
};

cv::Point2f reflect(const cv::Point2f& v,
                    const cv::Point2f& normal)
{
    return v - normal * ((v * 2.0f).dot(normal) / normal.dot(normal));
}

cv::Scalar hsv2rgb(float h,
                   float s,
                   float v)
{
    assert(0.0f <= h && h < 360.0f);
    assert(0.0f <= s && s <= 1.0f);
    assert(0.0f <= v && v <= 1.0f);

    float c = v * s;
    float x = c * (1.0f - std::abs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;

    c += m;
    x += m;

    c *= 255.0f;
    x *= 255.0f;

    if (h < 60.0f) {
        return { c, x, 0 };
    } else if (h < 120.0f) {
        return { x, c, 0 };
    } else if (h < 180.0f) {
        return { 0, c, x };
    } else if (h < 240.0f) {
        return { 0, x, c };
    } else if (h < 300.0f) {
        return { x, 0, c };
    } else {
        return { c, 0, x };
    }
}

float distance_sq(const cv::Point2f& a,
                  const cv::Point2f& b)
{
    cv::Point2f diff = a - b;
    return diff.dot(diff);
}

bool ballHitsRect(const cv::Point2f& ball_center,
                  float ball_radius,
                  const cv::Rect& rect,
                  cv::Point2f* normal = nullptr)
{
    cv::Point2f tl { (float) rect.tl().x, (float) rect.tl().y };
    cv::Point2f br { (float) rect.br().x, (float) rect.br().y };

    if (ball_center.x + ball_radius < tl.x
        || ball_center.x - ball_radius >= br.x
        || ball_center.y + ball_radius < tl.y
        || ball_center.y - ball_radius >= br.y) {
        return false;
    }

    if (normal) {
        struct Pair
        {
            float distance;
            cv::Point2f normal;

            bool operator <(const Pair& other) const
            { return distance < other.distance; }
        };
        Pair to_top { std::abs(tl.y - ball_center.y), { 0.0f, -1.0f }};
        Pair to_bottom { std::abs(br.y - ball_center.y), { 0.0f, 1.0f }};
        Pair to_left { std::abs(tl.x - ball_center.x), { -1.0f, 0.0f }};
        Pair to_right { std::abs(br.x - ball_center.x), { 1.0f, 0.0f }};

        Pair tuples[] = { to_top, to_bottom, to_left, to_right };
        *normal = (*std::min_element(tuples, tuples + 4)).normal;
    };

    return true; // TODO
};

class Game
{
public:
    static constexpr float BALL_SPEED = 250.0f;
    static constexpr int BALL_RADIUS = 10;
    static constexpr float PADDLE_WIDTH = 200.0f;
    static constexpr float PADDLE_HEIGHT = 30.0f;
    static constexpr size_t BLOCK_WIDTH = 40;
    static constexpr size_t BLOCK_HEIGHT = 10;

    Game(size_t board_width,
         size_t board_height):
        _board_width(board_width),
        _board_height(board_height),
        _blocks(board_width / BLOCK_WIDTH,
                board_height / 2 / BLOCK_HEIGHT),
        _paddle(board_width / 2.0f - PADDLE_WIDTH / 2, board_height - PADDLE_HEIGHT,
                PADDLE_WIDTH, PADDLE_HEIGHT)
    {
        _balls.emplace_back(cv::Point2f(board_width / 2, board_height / 2 - 10),
                            cv::Point2f(0.0f, BALL_SPEED));
        _blocks.fill(1);
    }

    void update(float dt)
    {
        for (auto& ball: _balls) {
            ball.position += ball.velocity * dt;
            handleCollisions(ball);
        }
    }

    void setPaddlePos(size_t pos)
    {
        _paddle.x = pos - _paddle.width / 2;
    }

    void drawOnto(Image& img)
    {
        cv::Rect board_rect { 0, 0, (int)_board_width, (int)_board_height };

        for (size_t y = 0; y < _blocks.height; ++y) {
            for (size_t x = 0; x < _blocks.width; ++x) {
                if (!_blocks[x][y]) {
                    continue;
                }

                cv::Scalar color = colorFromPos(x, y);
//                Logger::get("pong").info("color = %f %f %f", color[0], color[1], color[2]);
                cv::rectangle(img, rectForBlock(x, y), color, -1);
            }
        };

        for (const auto& ball: _balls) {
            cv::Scalar color = cv::Scalar(255, 0, 0);
            cv::circle(img, ball.position, BALL_RADIUS, color, -1);
        }

        cv::rectangle(img, _paddle, cv::Scalar(0, 255, 0), -1);
        cv::rectangle(img, board_rect, cv::Scalar(0, 255, 0));
    };

private:
    void handleCollisions(MovingObject& ball)
    {
        if (ball.position.x < 0.0f) {
            ball.velocity = reflect(ball.velocity, { 1.0f, 0.0f });
            ball.position.x = 0.0f;
        } else if (ball.position.x > _board_width) {
            ball.velocity = reflect(ball.velocity, { -1.0f, 0.0f });
            ball.position.x = _board_width;
        }

        if (ball.position.y < 0.0f) {
            Logger::get("pong").info("y < 0");
            ball.velocity = reflect(ball.velocity, { 0.0f, 1.0f });
            ball.position.y = 0.0f;
        } else if (ball.position.y > _board_height) {
            Logger::get("pong").info("y > height");
            ball.velocity = reflect(ball.velocity, { 0.0f, -1.0f });
            ball.position.y = _board_height;
        }

        if (ballHitsRect(ball.position, (float)BALL_RADIUS, _paddle)) {
            ball.velocity = velocityFromBallPos(ball.position.x);
        }

        for (size_t y = 0; y < _blocks.height; ++y) {
            for (size_t x = 0; x < _blocks.width; ++x) {
                cv::Point2f normal;
                if (ballHitsRect(ball.position, (float)BALL_RADIUS, rectForBlock(x, y), &normal)) {
                    ball.velocity = reflect(ball.velocity, normal);
                    _blocks[x][y] = 0;
                    return;
                }
            }
        }
    }

    cv::Point2f velocityFromBallPos(float ball_x)
    {
        float relative_x = (ball_x - _paddle.x) / _paddle.width;
        float angle = (relative_x - 0.5f) * ((float)CV_PI * 0.5f);
        return { std::sin(angle) * BALL_SPEED,
                 -std::cos(angle) * BALL_SPEED };
    }

    cv::Scalar colorFromPos(size_t x, size_t y)
    {
        return hsv2rgb((float)x / (float)_board_width * 359.0f,
                       (float)y / (float)_board_height,
                       1.0f);
    }

    cv::Rect rectForBlock(size_t x, size_t y)
    {
        return {
            (int) x * (int)BLOCK_WIDTH,
            (int) y * (int)BLOCK_HEIGHT,
            (int)BLOCK_WIDTH,
            (int)BLOCK_HEIGHT
        };
    }

    size_t _board_width;
    size_t _board_height;
    Array2D<uint8_t> _blocks;
    std::vector<MovingObject> _balls;
    cv::Rect_<float> _paddle;
};

#endif //PONG_PONG_H
