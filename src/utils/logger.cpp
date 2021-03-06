//
// Created by dex on 21.03.15.
//

#include <utils/logger.h>
#include <utils/make_unique.h>

std::set<std::string> Logger::_disabled_loggers;
std::map<std::string, std::unique_ptr<Logger>> Logger::_loggers;
Logger Logger::_null_logger;
std::mutex Logger::_logger_mutex;

class FileLogger: public Logger
{
public:
    FileLogger(const std::string& name,
               FILE* output):
            _name(name),
            _output(output)
    {}

    enum class Color
    {
        White = 0,
        Green = 32,
        Yellow = 33,
        Red = 31,
        Blue = 34
    };

#define DEFINE_LOG_LEVEL(Name, Level, Color, Prefix) \
    virtual void Name(const char* msg, ...) override \
    { \
        if (Level >= _level) { \
            va_list list; \
            va_start(list, msg); \
            log_internal(Color, Prefix, msg, list); \
            va_end(list); \
        } \
    }

    virtual void set_log_level(LogLevel level) override
    {
        _level = level;
    }

    DEFINE_LOG_LEVEL(printf, LogLevel::ERROR, Color::White, "")
    DEFINE_LOG_LEVEL(trace, LogLevel::TRACE, Color::White, "[TRACE] ")
    DEFINE_LOG_LEVEL(debug, LogLevel::DEBUG, Color::Blue, "[DEBUG] ")
    DEFINE_LOG_LEVEL(info, LogLevel::INFO, Color::Green, "[INFO] ")
    DEFINE_LOG_LEVEL(warning, LogLevel::WARNING, Color::Yellow, "[WARN] ")
    DEFINE_LOG_LEVEL(error, LogLevel::ERROR, Color::Red, "[ERR] ")

private:
    LogLevel _level = LogLevel::TRACE;
    const std::string _name;
    FILE* _output;

    void log_internal(Color color,
                      const char* prefix,
                      const char* msg,
                      va_list args)
    {
#ifndef WITH_COLORS
        (void)color;
#endif
        char format_buffer[1024];
        char buffer[4096];

        int bytes_written = snprintf(format_buffer, sizeof(format_buffer),
#ifdef WITH_COLORS
# define SUFFIX_LENGTH 4
                                     "\033\[%dm<%s> %s%s\033\[0m", color,
#else
# define SUFFIX_LENGTH 0
                                     "<%s> %s%s",
#endif
                                     _name.c_str(), prefix, msg);
        assert(bytes_written >= 0
               && bytes_written < (int)sizeof(format_buffer)
               && "format_buffer too small");

        bytes_written = vsnprintf(buffer, sizeof(buffer),
                                  format_buffer, args);
        assert(bytes_written >= 0
               && bytes_written < (int)sizeof(buffer)
               && "buffer too small");

        if (bytes_written > SUFFIX_LENGTH + 1
                && buffer[bytes_written - SUFFIX_LENGTH - 1] != '\n'
                && (size_t)(bytes_written + 1) < sizeof(buffer)) {
            buffer[bytes_written] = '\n';
            buffer[++bytes_written] = '\0';
        }

        fwrite(buffer, bytes_written, 1, _output);
    }
};

Logger& Logger::get(const std::string& name) {
    std::lock_guard<std::mutex> lock(_logger_mutex);

    if (_disabled_loggers.count(name) == 0) {
        auto it = _loggers.find(name);

        if (it == _loggers.end()) {
            auto logger = std::make_unique<FileLogger>(name, stderr);
            it = _loggers.insert(std::make_pair(name, std::move(logger))).first;
        }

        return *it->second;
    }

    return _null_logger;
}

void Logger::toggle(const std::string& name) {
    std::lock_guard<std::mutex> lock(_logger_mutex);
    auto it = _disabled_loggers.find(name);

    if (it == _disabled_loggers.end()) {
        _disabled_loggers.insert(it, name);
    } else {
        _disabled_loggers.erase(it);
    }
}

void Logger::enable(const std::string& name) {
    std::lock_guard<std::mutex> lock(_logger_mutex);
    _disabled_loggers.erase(name);
}

void Logger::disable(const std::string& name) {
    std::lock_guard<std::mutex> lock(_logger_mutex);
    _disabled_loggers.insert(name);
}
