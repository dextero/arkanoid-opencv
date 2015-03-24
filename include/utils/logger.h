//
// Created by dex on 21.03.15.
//

#ifndef _PONG_LOGGER_H_
#define _PONG_LOGGER_H_

#include <cstdarg>
#include <cstdio>
#include <deque>
#include <sstream>
#include <iostream>
#include <set>
#include <assert.h>
#include <memory>
#include <map>

class Logger
{
public:
    virtual ~Logger() { flush(); }

    Logger(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator =(const Logger&) = delete;
    Logger& operator =(Logger&&) = delete;

    static Logger& get(const std::string& name);
    static void toggle(const std::string& name);

    virtual void  printf(const char* /*msg*/, ...) {}
    virtual void   trace(const char* /*msg*/, ...) {}
    virtual void   debug(const char* /*msg*/, ...) {}
    virtual void    info(const char* /*msg*/, ...) {}
    virtual void warning(const char* /*msg*/, ...) {}
    virtual void   error(const char* /*msg*/, ...) {}

    virtual void flush() {}

protected:
    Logger() {}

private:
    static std::map<std::string, std::unique_ptr<Logger>> _loggers;
    static std::set<std::string> _disabled_loggers;
    static Logger _null_logger;
};

#endif //_PONG_LOGGER_H_
