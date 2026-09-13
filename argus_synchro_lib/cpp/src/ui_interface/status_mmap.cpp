#include "ui_interface/status_mmap.h"

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <sys/mman.h>
#include <unistd.h>

// static変数
double StatusMMAP::last_read_time = 0.0;

StatusMMAP::StatusMMAP(const std::string& path, bool create) : path(path), size(4), fd(-1), mmap_ptr(nullptr)
{
    std::filesystem::path dir = std::filesystem::path(path).parent_path();
    if (!dir.empty() && !std::filesystem::exists(dir))
    {
        std::filesystem::create_directories(dir);
    }

    if (create)
    {
        if (std::filesystem::exists(path))
        {
            std::cout << "[StatusMMAP] 既存 mmap 削除" << std::endl;
            std::filesystem::remove(path);
        }

        int tmp_fd = open(path.c_str(), O_RDWR | O_CREAT, 0666);
        if (tmp_fd < 0)
        {
            throw std::runtime_error("Failed to create mmap file");
        }
        if (ftruncate(tmp_fd, size) == -1)
        {
            close(tmp_fd);
            throw std::runtime_error("Failed to truncate file");
        }
        char zeros[4] = {0, 0, 0, 0};
        if (write(tmp_fd, zeros, size) != (ssize_t)size)
        {
            close(tmp_fd);
            throw std::runtime_error("Failed to write initial data");
        }
        close(tmp_fd);
    }
    fd = open(path.c_str(), O_RDWR);
    if (fd < 0)
    {
        throw std::runtime_error("Failed to open mmap file");
    }
    mmap_ptr = static_cast<char*>(mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    if (mmap_ptr == MAP_FAILED)
    {
        close(fd);
        throw std::runtime_error("Failed to mmap file");
    }
}

void StatusMMAP::write_status(int code) const
{
    std::memcpy(mmap_ptr, &code, sizeof(int));
    if (msync(mmap_ptr, size, MS_SYNC) == -1)
    {
        std::cerr << "msync failed" << std::endl;
    }
}

void StatusMMAP::write_status(StatusCode code) const
{
    write_status(static_cast<int>(code));
}

int StatusMMAP::read_status()
{
    int code = 0;
    std::memcpy(&code, mmap_ptr, sizeof(int));
    last_read_time = current_time_seconds();
    return code;
}

void StatusMMAP::close_mmap()
{
    if (mmap_ptr && mmap_ptr != MAP_FAILED)
    {
        munmap(mmap_ptr, size);
        mmap_ptr = nullptr;
    }
    if (fd >= 0)
    {
        close(fd);
        fd = -1;
    }
}

StatusMMAP::~StatusMMAP()
{
    close_mmap();
}

double StatusMMAP::current_time_seconds()
{
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

bool StatusMMAP::is_recent(double timeout)
{
    double now = current_time_seconds();
    return (now - last_read_time) < timeout;
}

std::string StatusMMAP::get_status_name(int code)
{
    switch (code)
    {
    case static_cast<int>(StatusCode::INIT):
        return "INIT";
    case static_cast<int>(StatusCode::REBOOT):
        return "REBOOT";
    case static_cast<int>(StatusCode::BOOTING):
        return "BOOTING";
    case static_cast<int>(StatusCode::RUNNING):
        return "RUNNING";
    case static_cast<int>(StatusCode::ERROR):
        return "ERROR";
    case static_cast<int>(StatusCode::SHUTDOWN):
        return "SHUTDOWN";
    default:
        return "UNKNOWN";
    }
}