#pragma once

#include <string>
#include <functional>
#include <utility> // std::forward
#include <cstdio>  // std::snprintf
#include <cassert>

enum class LogLevel : int
{
    NOTSET = 0,
    DEBUG = 10,
    INFO = 20,
    WARNING = 30,
    ERROR = 40,
    CRITICAL = 50,
};

using LoggerFunc = std::function<void(int, std::string)>;

class PyLogger
{
  private:
    LoggerFunc logger_func_;

  public:
    PyLogger(LoggerFunc logger_func) : logger_func_(std::move(logger_func))
    {
        assert(this->logger_func_ && "logger must be set");
    }

    void setLogger(LoggerFunc logger_func)
    {
        this->logger_func_ = std::move(logger_func);
    }

    // 整形不要の文字列なら直接logを呼ぶのが一番高速
    void log(LogLevel level, std::string msg) const
    {
        if (this->logger_func_)
        {
            this->logger_func_(static_cast<int>(level), msg);
        }
        else
        {
            assert(this->logger_func_ && "logger must be set");
        }
    }

    // C++ 側で printf 風に整形してからログ出力
    // 例: logf(LogLevel::INFO, "id=%d temp=%.1fC", id, temp);
    template <typename... Ts> void logf(LogLevel level, const char* fmt, Ts&&... ts) const;

    template <typename... Ts> void debug(const char* fmt, Ts&&... ts) const;

    template <typename... Ts> void info(const char* fmt, Ts&&... ts) const;

    template <typename... Ts> void warning(const char* fmt, Ts&&... ts) const;

    template <typename... Ts> void error(const char* fmt, Ts&&... ts) const;

    template <typename... Ts> void critical(const char* fmt, Ts&&... ts) const;
};

namespace logger_sink_detail
{
template <typename... Ts> inline std::string sprintf_to_string(const char* fmt, Ts&&... ts)
{
    // std::snprintf を 2 回呼ぶ（1 回目で必要サイズを取得）
    // 必要文字数（終端 NUL を含まない）を取得
    int n = std::snprintf(nullptr, 0, fmt, std::forward<Ts>(ts)...);
    if (n <= 0)
    {
        // フォーマット不一致やエラー時は空文字（落とさない）
        return std::string();
    }

    // NUL 終端分 +1 で確保 → 書き込んでから縮める
    std::string out;
    out.resize(static_cast<size_t>(n) + 1);
    std::snprintf(out.data(), out.size(), fmt, std::forward<Ts>(ts)...);
    out.resize(static_cast<size_t>(n)); // 終端NUL除去
    return out;
}
} // namespace logger_sink_detail

template <typename... Ts> void PyLogger::logf(LogLevel level, const char* fmt, Ts&&... ts) const
{
    std::string msg = logger_sink_detail::sprintf_to_string(fmt, std::forward<Ts>(ts)...);
    this->log(level, msg);
}

template <typename... Ts> void PyLogger::debug(const char* fmt, Ts&&... ts) const
{
    this->logf(LogLevel::DEBUG, fmt, std::forward<Ts>(ts)...);
}

template <typename... Ts> void PyLogger::info(const char* fmt, Ts&&... ts) const
{
    this->logf(LogLevel::INFO, fmt, std::forward<Ts>(ts)...);
}

template <typename... Ts> void PyLogger::warning(const char* fmt, Ts&&... ts) const
{
    this->logf(LogLevel::WARNING, fmt, std::forward<Ts>(ts)...);
}

template <typename... Ts> void PyLogger::error(const char* fmt, Ts&&... ts) const
{
    this->logf(LogLevel::ERROR, fmt, std::forward<Ts>(ts)...);
}

template <typename... Ts> void PyLogger::critical(const char* fmt, Ts&&... ts) const
{
    this->logf(LogLevel::CRITICAL, fmt, std::forward<Ts>(ts)...);
}