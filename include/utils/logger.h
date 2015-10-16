//
// Created by dex on 21.03.15.
//

#ifndef _ARKANOID_LOGGER_H_
#define _ARKANOID_LOGGER_H_

#include <cstdarg>
#include <cstdio>
#include <deque>
#include <sstream>
#include <iostream>
#include <set>
#include <assert.h>
#include <memory>
#include <map>
#include <mutex>

class Logger
{
public:
    enum class LogLevel
    {
        TRACE,
        DEBUG,
        INFO,
        WARNING,
        ERROR,
        QUIET,
    };

    virtual ~Logger() { flush(); }

    Logger(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator =(const Logger&) = delete;
    Logger& operator =(Logger&&) = delete;

    static Logger& get(const std::string& name);
    static void toggle(const std::string& name);
    static void enable(const std::string& name);
    static void disable(const std::string& name);

    virtual void set_log_level(LogLevel /*level*/) {}

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
    static std::mutex _logger_mutex;
};

#endif //_ARKANOID_LOGGER_H_
