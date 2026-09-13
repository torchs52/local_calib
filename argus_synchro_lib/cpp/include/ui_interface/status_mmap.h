#pragma once
#include <string>

enum class StatusCode : int
{
    INIT = 0,
    REBOOT = 1,
    BOOTING = 2,
    RUNNING = 3,
    ERROR = -2,
    SHUTDOWN = -1
};

class StatusMMAP
{
  public:
    StatusMMAP(const std::string& path, bool create = false);
    ~StatusMMAP();

    void write_status(int code) const;
    void write_status(StatusCode code) const;

    int read_status();

    void close_mmap();

    static bool is_recent(double timeout = 5.0);

    static std::string get_status_name(int code);

  private:
    std::string path;
    const size_t size;
    int fd;
    char* mmap_ptr;

    static double last_read_time;

    static double current_time_seconds();
};