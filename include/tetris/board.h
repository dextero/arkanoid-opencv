//
// Created by dex on 25.04.15.
//

#ifndef PONG_BOARD_H
#define PONG_BOARD_H

#include <vector>
#include <array>
#include <random>
#include <string.h>
#include <assert.h>
#include <bits/algorithmfwd.h>

#include <opencv2/core/core.hpp>
#include <image.h>
#include <cstdarg>
#include <utils/logger.h>

template<typename T>
T clamp(T val, T min_val, T max_val)
{
    return std::max(min_val,
                    std::min(max_val, val));
}

namespace tetris {

typedef cv::Point_<unsigned> Coords;

template<typename ValueT,
         typename Distribution = std::uniform_int_distribution<ValueT>,
         typename Generator = std::mt19937,
         typename RandomDevice = std::random_device>
class RNG
{
public:
    RNG(ValueT min, ValueT max):
        _device(),
        _generator(_device()),
        _distribution(min, max)
    { }

    inline ValueT next()
    {
        return _distribution(_generator);
    }

private:
    RandomDevice _device;
    Generator _generator;
    Distribution _distribution;
};

template<typename T>
class Array2D
{
public:
    template<typename ArrayT>
    class Column
    {
    public:
        T& operator[](size_t row)
        { return _array.at(_column, row); }

        const T& operator[](size_t row) const
        { return _array.at(_column, row); }

    private:
        Column(ArrayT& array,
               size_t column):
            _array(array),
            _column(column)
        { }

        ArrayT& _array;
        size_t _column;

        friend class Array2D;
    };

    Array2D(size_t width,
            size_t height):
        width(width),
        height(height),
        _fields(width * height)
    { }

    Column<Array2D> operator[](size_t idx)
    { return { *this, idx }; }

    const Column<const Array2D> operator[](size_t idx) const
    { return { *this, idx }; }

    void fill(T value)
    {
        for (auto& elem: _fields) {
            elem = value;
        }
    }

    const size_t width;
    const size_t height;

private:
    size_t idx(size_t x,
               size_t y) const
    {
        assert(x < width);
        assert(y < height);

        return x + y * width;
    }

    T& at(size_t x, size_t y)
    { return _fields[idx(x, y)]; }

    const T& at(size_t x, size_t y) const
    { return _fields[idx(x, y)]; }

    std::vector<T> _fields;

    friend class Column<Array2D>;

    friend class Column<const Array2D>;
};

struct OrientedTetromino
{
    std::vector<uint8_t> fields;
    size_t width;
    size_t height;
};

class Tetromino
{
public:
    Tetromino(std::vector<OrientedTetromino>&& orientations):
        _orientations(std::move(orientations)),
        _curr_orientation(0)
    {}

    Tetromino& operator =(const Tetromino& src)
    {
        _orientations = src._orientations;
        _curr_orientation = src._curr_orientation;
        return *this;
    }

    void rotate(bool reverse = false)
    {
        assert(_orientations.size() > 0);
        if (reverse) {
            _curr_orientation += (_orientations.size() - 1);
        } else {
            ++_curr_orientation;
        }
        _curr_orientation = _curr_orientation % _orientations.size();
    }

    bool canPlaceAt(const Array2D<uint8_t>& board_fields,
                    const Coords& at) const
    {
        if (at.x + width() > board_fields.width) {
            Logger::get("tetris").info("cannot place: %u+%u is too far right", at.x, width());
            return false;
        }
        if (at.y > board_fields.height) {
            Logger::get("tetris").info("cannot place: %u is too far up", at.y);
            return false;
        }

        const OrientedTetromino& tetromino_fields = _orientations[_curr_orientation];
        for (size_t idx = 0; idx < tetromino_fields.fields.size(); ++idx) {
            if (!tetromino_fields.fields[idx]) {
                continue;
            }

            cv::Point2i c = coords(idx) + cv::Point2i((int)at.x, (int)at.y);
            if (c.y == -1) {
                return false;
            }

            assert(c.x >= 0);
            assert(c.y >= 0);
            assert(c.x < (int)board_fields.width);
            assert(c.y < (int)board_fields.height);

            if (board_fields[c.x][c.y]) {
                Logger::get("tetris").info("cannot place: field %u, %u occupied", c.x, c.y);
                return false;
            }
        }

        return true;
    }

    void placeAt(Array2D<uint8_t>& board_fields,
                 const Coords& at,
                 uint8_t value)
    {
        const OrientedTetromino& tetromino_fields = _orientations[_curr_orientation];
        for (size_t idx = 0; idx < tetromino_fields.fields.size(); ++idx) {
            if (!tetromino_fields.fields[idx]) {
                continue;
            }

            cv::Point2i c = coords(idx) + cv::Point2i((int)at.x, (int)at.y);
            assert(c.x >= 0);
            assert(c.y >= 0);
            assert(c.x < (int)board_fields.width);
            assert(c.y < (int)board_fields.height);

            board_fields[c.x][c.y] = value;
        }
    }

    size_t width() const { return _orientations[_curr_orientation].width; }
    size_t height() const { return _orientations[_curr_orientation].height; }

private:
    inline size_t idx(size_t x, size_t y) const
    {
        assert(x < width());
        assert(y < height());

        return x + y * width();
    }

    inline cv::Point2i coords(size_t idx) const
    {
        assert(idx < width() * height());

        return { (int)(idx % width()), -(int)(idx / width()) };
    }

    std::vector<OrientedTetromino> _orientations;
    size_t _curr_orientation;
};

class TetrominoBuilder
{
public:
    TetrominoBuilder& operator /(const char* def)
    {
        if (curr_orientation == orientation_defs.size()) {
            orientation_defs.emplace_back();
        }

        if (def) {
            orientation_defs[curr_orientation].emplace_back(def);
        }

        ++curr_orientation;
        return *this;
    }

    TetrominoBuilder& operator *(const char* def)
    {
        curr_orientation = 0;
        return *this / def;
    }

    Tetromino build()
    {
        assert(orientation_defs.size() > 0);
        for (size_t def_idx = 0; def_idx < orientation_defs.size(); ++def_idx) {
            const std::vector<std::string>& def = orientation_defs[def_idx];
            assert(def.size() > 0);
            size_t len = def.front().size();
            for (size_t i = 0 ; i < def.size(); ++i) {
                const std::string& row = def[i];
                assert(row.size() == len);
            }
        }

        std::vector<OrientedTetromino> orientations(orientation_defs.size());
        for (size_t i = 0; i < orientation_defs.size(); ++i) {
            std::vector<std::string>& def = orientation_defs[i];
            OrientedTetromino& ot = orientations[i];

            ot.width = def.front().size();
            ot.height = def.size();
            ot.fields.resize(ot.width * ot.height);

            for (size_t y = 0; y < ot.height; ++y) {
                const std::string& row = def[y];

                for (size_t x = 0; x < row.size(); ++x) {
                    size_t idx = x + y * ot.width;
                    ot.fields[idx] = (uint8_t)(row[x] != ' ' ? 1 : 0);
                }
            }
        }

        std::cout << "loaded tetromino:\n";
        for (size_t row = 0; row < orientations.front().width; ++row) {
            std::stringstream line;

            for (const auto& o: orientations) {
                line << "|";
                size_t start_idx = row * o.width;
                for (size_t col = 0; col < o.width; ++col) {
                    line << (o.fields[start_idx + col] ? "X" : " ");
                }
                line << "| ";
            }

            std::cout << line.str() << "\n";
        }

        return Tetromino(std::move(orientations));
    }

private:
    std::vector<std::vector<std::string>> orientation_defs;
    size_t curr_orientation = 0;
};

const std::array<Tetromino, 7> BLOCKS {{
    (TetrominoBuilder()
        * "XX " / " X"
        * " XX" / "XX"
        * NULL  / "X ").build(),
    (TetrominoBuilder()
        * " XX" / "X "
        * "XX " / "XX"
        * NULL  / " X").build(),
    (TetrominoBuilder()
        * "X " / "XXX" / "XX" / "  X"
        * "X " / "X  " / " X" / "XXX"
        * "XX" / NULL  / " X" / NULL).build(),
    (TetrominoBuilder()
        * " X" / "X  " / "XX" / "XXX"
        * " X" / "XXX" / "X " / "  X"
        * "XX" / NULL  / "X " / NULL).build(),
    (TetrominoBuilder()
        * "X" / "XXXX"
        * "X" / NULL
        * "X" / NULL
        * "X" / NULL).build(),
    (TetrominoBuilder()
        * "XX"
        * "XX").build(),
    (TetrominoBuilder()
        * "X " / "XXX" / " X" / " X "
        * "XX" / " X " / "XX" / "XXX"
        * "X " / NULL  / " X" / NULL).build(),
}};

class Board
{
public:
    Board(size_t width, size_t height):
        _rng(0, BLOCKS.size() - 1),
        _fields(width, height),
        _curr_piece(getNextPiece()),
        _curr_piece_pos(getPieceInitialPos())
    {
        assert(width > 0);
        assert(height > 0);
    }

    void restart()
    {
        _fields.fill(0);
        resetPiece();
    }

    void advance()
    {
        Coords new_pos = _curr_piece_pos;
        --new_pos.y;

        if (_curr_piece.canPlaceAt(_fields, new_pos)) {
            _curr_piece_pos = new_pos;
        } else {
            _curr_piece.placeAt(_fields, _curr_piece_pos, 1);

            for (size_t y = 0; y < _fields.height;) {
                if (isRowFull(y)) {
                    clearRow(y);
                } else {
                    ++y;
                }
            }

            resetPiece();
        }
    }

    void rotatePiece()
    {
        _curr_piece.rotate();

        size_t max_x = _fields.width - _curr_piece.width();
        Logger::get("tetris").info("x was %u, new max = %u, width = %u", _curr_piece_pos.x, max_x, _curr_piece.width());
        Coords new_pos {
            clamp(_curr_piece_pos.x, 0U, (unsigned) max_x),
            _curr_piece_pos.y
        };

        if (_curr_piece.canPlaceAt(_fields, new_pos)) {
            _curr_piece_pos = new_pos;
        } else {
            _curr_piece.rotate(true);
        }
    };

    void movePiece(int delta)
    {
        int curr_x = (int) _curr_piece_pos.x;
        size_t max_x = _fields.width - _curr_piece.width();
        Logger::get("tetris").info("x was %u, new max = %u, width = %u", _curr_piece_pos.x, max_x, _curr_piece.width());
        int new_x = clamp(curr_x + delta, 0, (int)max_x);

        assert(new_x >= 0);
        Coords new_pos { (unsigned)new_x, _curr_piece_pos.y };
        if (_curr_piece.canPlaceAt(_fields, new_pos)) {
            _curr_piece_pos = new_pos;
        }
    };

    void drawOnto(Image& img)
    {
        Image board_img(img.size(), img.type());

        cv::Rect board_rect = getBoardRect(img.size());
        int field_size = std::min(board_rect.width / (int) _fields.width,
                                  board_rect.height / (int) _fields.height);

        _curr_piece.placeAt(_fields, _curr_piece_pos, 2);
        for (size_t y = 0; y < _fields.height; ++y) {
            for (size_t x = 0; x < _fields.width; ++x) {
                if (!_fields[x][y]) {
                    continue;
                }

                cv::Point top_left {
                    board_rect.x + (int) x * field_size,
                    board_rect.y + board_rect.height -
                                   ((int) y + 1) * field_size
                };
                cv::Point bottom_right {
                    top_left.x + field_size,
                    top_left.y + field_size
                };

                cv::Scalar color =
                    _fields[x][y] == 1 ? cv::Scalar(0, 255, 0, 128)
                                       : cv::Scalar(255, 0, 0, 128);
                cv::rectangle(board_img, top_left, bottom_right, color, -1);
            }
        };
        _curr_piece.placeAt(_fields, _curr_piece_pos, 0);

        cv::rectangle(board_img, board_rect, cv::Scalar(0, 255, 0));
        img ^= board_img;
    };

private:
    bool isRowFull(size_t row)
    {
        for (size_t x = 0; x < _fields.width; ++x) {
            if (!_fields[x][row]) {
                return false;
            }
        }

        return true;
    }

    void clearRow(size_t row)
    {
        for (size_t y = row + 1; y < _fields.height - 1; ++y) {
            for (size_t x = 0; x < _fields.width; ++x) {
                _fields[x][y - 1] = _fields[x][y];
            }
        }

        for (size_t x = 0; x < _fields.width; ++x) {
            _fields[x][_fields.height - 1] = 0;
        }
    }

    cv::Rect getBoardRect(const cv::Size& img_size) const
    {
        float board_ratio = (float) _fields.width / (float) _fields.height;
        float img_ratio = (float) img_size.width / (float) img_size.height;

        if (board_ratio > img_ratio) {
            float x_scale = img_size.width / _fields.width;
            int board_height = (int) (_fields.height * x_scale);

            return {
                0, (img_size.height - board_height) / 2,
                img_size.width, board_height
            };
        } else {
            float y_scale = img_size.height / _fields.height;
            int board_width = (int) (_fields.width * y_scale);

            return {
                (img_size.width - board_width) / 2, 0,
                board_width, img_size.height
            };
        }
    }

    Tetromino getNextPiece()
    {
        return BLOCKS[_rng.next()];
    }

    Coords getPieceInitialPos()
    {
        return { (unsigned)(_fields.width / 2 - _curr_piece.width() / 2),
                 (unsigned)(_fields.height - 1) };
    }

    void resetPiece()
    {
        _curr_piece = getNextPiece();
        _curr_piece_pos = getPieceInitialPos();

        if (!_curr_piece.canPlaceAt(_fields, _curr_piece_pos)) {
            restart();
        }
    }

    RNG<size_t> _rng;
    Array2D<uint8_t> _fields;
    Tetromino _curr_piece;
    Coords _curr_piece_pos;
};

}; // namespace tetris

#endif //PONG_BOARD_H
