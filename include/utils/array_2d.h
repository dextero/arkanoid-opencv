//
// Created by dex on 26.04.15.
//

#ifndef PONG_ARRAY_2D_H
#define PONG_ARRAY_2D_H

#include <vector>
#include <stddef.h>
#include <assert.h>

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


#endif //PONG_ARRAY_2D_H
