#include "clsmmap/ClsMMap.h"

#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sys/mman.h>
#include <unistd.h>

#include <iostream>
#include <chrono>

// 実行環境を取得
static inline std::string getPlatform(void)
{
#if defined(_WIN32)
    std::string system = "Windows";
#elif defined(__linux__)
    std::string system = "Linux";
#elif defined(__APPLE__)
    std::string system = "Darwin";
#else
    std::string system = "Unknown OS";
#endif
    return system;
}

ByteOrder GetSystemByteOrder()
{
    int n = 1;
    // little endian if true
    if (*(char*)&n == 1)
    {
        return ByteOrder::LITTLE;
    }
    return ByteOrder::BIG;
}

classMMap::classMMap(const MMapConfig& cfg, const LoggerFunc logfunc)
    : cfg_(cfg), system_endian_(GetSystemByteOrder()), current_index_(0), logger_(PyLogger(logfunc))
{
    this->logger_.info("platform: %s", getPlatform().c_str());
    this->logger_.info("endian: %s", (bool(this->system_endian_) ? "little" : "big"));

    if (cfg_.paths.empty())
    {
        throw std::runtime_error("classMMap: dat_paths is empty.");
    }
    if (cfg_.map_size_bytes == 0)
    {
        throw std::runtime_error("classMMap: map_size_bytes must be > 0.");
    }
    ensure_and_map_all();
}

classMMap::~classMMap()
{
    this->dispose();
}

void classMMap::begin_frame() const
{
    this->WriteInt8(cfg_.proto.IsWriting_ADR, 1);
}

void classMMap::end_frame(bool is_initial)
{
    bool isRead = false;
    if (auto v = ReadInt8(cfg_.proto.IsReading_ADR))
    {
        isRead = (v.value() > 0);
    }

    auto isWriting = ReadInt8(cfg_.proto.IsWriting_ADR).value_or(255);
    logger_.debug("END   idx=%zu IsW=%u IsR=%u initial=%d", current_index_, static_cast<unsigned>(isWriting),
                  static_cast<unsigned>(isRead ? 1 : 0), static_cast<int>(is_initial));

    if (isRead || is_initial)
    {
        const size_t next_index = (current_index_ + 1) % mms_.size();

        // 現在のmapの書き込み完了
        WriteInt8(cfg_.proto.IsWriting_ADR, 0);

        logger_.debug("CHANGE %zu (%s) -> %zu (%s)", current_index_, cfg_.paths[current_index_].c_str(), next_index,
                      cfg_.paths[next_index].c_str());

        current_index_ = next_index;

        // 次に書くmapを直ちに予約
        WriteInt8(cfg_.proto.IsWriting_ADR, 1);
    }
    else
    {
        logger_.debug("KEEP  idx=%zu (%s) IsReading=0", current_index_, cfg_.paths[current_index_].c_str());
    }
}

void classMMap::dispose()
{
    unmap_all();
}

std::optional<uint64_t> classMMap::ReadInt64(int adr) const
{
    std::optional<uint64_t> val = this->ReadInt<uint64_t>(adr, false);
    if (!val.has_value())
    {
        return val;
    }
    if (cfg_.endian != this->system_endian_)
    {
        // system内のendianと使用したいendianが異なっていたらswap
        val = __builtin_bswap64(val.value());
    }
    return val;
}

std::optional<uint32_t> classMMap::ReadInt32(int adr) const
{
    std::optional<uint32_t> val = this->ReadInt<uint32_t>(adr, false);
    if (!val.has_value())
    {
        return val;
    }
    if (cfg_.endian != this->system_endian_)
    {
        // system内のendianと使用したいendianが異なっていたらswap
        val = __builtin_bswap32(val.value());
    }
    return val;
};

std::optional<uint16_t> classMMap::ReadInt16(int adr) const
{
    std::optional<uint16_t> val = this->ReadInt<uint16_t>(adr, false);
    if (!val.has_value())
    {
        return val;
    }
    if (cfg_.endian != this->system_endian_)
    {
        // system内のendianと使用したいendianが異なっていたらswap
        val = __builtin_bswap16(val.value());
    }
    return val;
};

std::optional<int16_t> classMMap::ReadSignedInt16(int adr) const
{
    std::optional<int16_t> val = this->ReadInt<int16_t>(adr, false);
    if (!val.has_value())
    {
        return val;
    }
    if (cfg_.endian != this->system_endian_)
    {
        // system内のendianと使用したいendianが異なっていたらswap
        val = __builtin_bswap16(val.value());
    }
    return val;
}

std::optional<uint8_t> classMMap::ReadInt8(int adr) const
{
    return this->ReadInt<uint8_t>(adr, false);
};

std::optional<float> classMMap::ReadFloat(int adr) const
{
    try
    {
        this->isValidMemoryMappingAndAddress(adr, sizeof(float));
        float val = 0.0f;
        std::memcpy(&val, current_mm() + adr, sizeof(float));
        return val;
    }
    catch (const std::exception& e)
    {
        // try catch
        std::cerr << "ReadFloat except " << e.what() << std::endl;
        return std::nullopt;
    }
}

void classMMap::WriteInt64(int adr, uint64_t data) const
{
    if (cfg_.endian != this->system_endian_)
    {
        // system内のendianと使用したいendianが異なっていたらswap
        data = __builtin_bswap64(data);
    }
    this->WriteInt<uint64_t>(adr, data, false);
};

void classMMap::WriteInt32(int adr, uint32_t data) const
{
    if (cfg_.endian != this->system_endian_)
    {
        // system内のendianと使用したいendianが異なっていたらswap
        data = __builtin_bswap32(data);
    }
    this->WriteInt<uint32_t>(adr, data, false);
};

void classMMap::WriteInt16(int adr, uint16_t data) const
{
    if (cfg_.endian != this->system_endian_)
    {
        // system内のendianと使用したいendianが異なっていたらswap
        data = __builtin_bswap16(data);
    }
    this->WriteInt<uint16_t>(adr, data, false);
};

void classMMap::WriteSignedInt16(int adr, int16_t data) const
{
    if (cfg_.endian != this->system_endian_)
    {
        // system内のendianと使用したいendianが異なっていたらswap
        data = __builtin_bswap16(data);
    }
    this->WriteInt<int16_t>(adr, data, true);
};

void classMMap::WriteInt8(int adr, uint8_t data) const
{
    this->WriteInt<uint8_t>(adr, data, false);
};

void classMMap::WriteFloat(int adr, float data) const
{
    try
    {
        this->isValidMemoryMappingAndAddress(adr, sizeof(float));

        std::memcpy(current_mm() + adr, &data, sizeof(float));
    }
    catch (const std::exception& e)
    {
        // try catch
        std::cerr << "WriteFloat except " << e.what() << std::endl;
    }
}

void classMMap::WriteBytes(int adr, const Eigen::Ref<const Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>>& data) const
{
    try
    {
        this->isValidMemoryMappingAndAddress(adr, data.size());
        std::memcpy(current_mm() + adr, data.data(), data.size());
    }
    catch (const std::exception& e)
    {
        // try catch
        std::cerr << "WriteBytes except " << e.what() << std::endl;
    }
};

void classMMap::WriteBytes(int adr, const std::vector<unsigned char>& data) const
{
    try
    {
        this->isValidMemoryMappingAndAddress(adr, data.size());
        std::memcpy(current_mm() + adr, data.data(), data.size());
    }
    catch (const std::exception& e)
    {
        // try catch
        std::cerr << "WriteBytes except " << e.what() << std::endl;
    }
};

size_t classMMap::get_file_index() const
{
    return cfg_.paths.size();
}

std::optional<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> classMMap::ReadBytes(size_t index, int adr, int length) const
{
    try
    {
        if (index >= mms_.size() || mms_[index] == nullptr || fds_[index] == -1)
        {
            throw std::out_of_range("ReadBytes: invalid buffer index");
        }
        if (adr < 0 || static_cast<size_t>(adr) >= cfg_.map_size_bytes)
        {
            throw std::out_of_range("ReadBytes: adr out of range");
        }

        const char* base = mms_[index];

        if (length < 0)
        {
            const int start = cfg_.proto.Start_ADR;
            const size_t total = cfg_.map_size_bytes;
            Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> val(total);

            const size_t tail = total - static_cast<size_t>(start);
            std::memcpy(val.data(), base + start, tail);
            if (start > 0)
            {
                std::memcpy(val.data() + tail, base, static_cast<size_t>(start));
            }
            return val;
        }

        // 部分読み出し
        if (length < 0 || static_cast<size_t>(adr + length) > cfg_.map_size_bytes)
        {
            throw std::out_of_range("ReadBytes: length out of range");
        }
        Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> val(length);
        std::memcpy(val.data(), base + adr, static_cast<size_t>(length));
        return val;
    }
    catch (const std::exception& e)
    {
        std::cerr << "ReadBytes(indexed) except " << e.what() << std::endl;
        return std::nullopt;
    }
}

// fileに明示的に書き込むための関数
void classMMap::flush() const
{
    if (current_mm() && msync(current_mm(), cfg_.map_size_bytes, MS_SYNC) == -1)
    {
        std::cerr << "msync failed" << std::endl;
    }
}

void classMMap::ensure_and_map_all()
{
    const auto& paths = cfg_.paths;
    fds_.resize(paths.size(), -1);
    mms_.resize(paths.size(), nullptr);

    for (size_t i = 0; i < paths.size(); ++i)
    {
        const std::string& path = paths[i];

        if (std::filesystem::exists(path))
        { // 前回異常終了対策でファイル削除
            std::filesystem::remove(path);
        }

        int fd = ::open(path.c_str(), O_RDWR | O_CREAT, 0666);
        if (fd == -1)
        {
            throw std::runtime_error("Failed to open or create file: " + path);
        }

        try
        {
            // サイズが異なるなら ftruncate
            this->logger_.info("dataPathIndex: {%d}, {%s]}", i, path.c_str());
            if (std::filesystem::exists(path))
            {
                const auto cur = static_cast<int64_t>(std::filesystem::file_size(path));
                if (static_cast<size_t>(cur) != cfg_.map_size_bytes)
                {
                    if (ftruncate(fd, static_cast<off_t>(cfg_.map_size_bytes)) == -1)
                    {
                        ::close(fd);
                        throw std::runtime_error("Failed to set file size: " + path);
                    }
                }
            }
            else
            {
                if (ftruncate(fd, static_cast<off_t>(cfg_.map_size_bytes)) == -1)
                {
                    ::close(fd);
                    throw std::runtime_error("Failed to set file size: " + path);
                }
            }
            this->logger_.info("classMMap %s created", path.c_str());

            void* mm = ::mmap(nullptr, cfg_.map_size_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            if (mm == MAP_FAILED)
            {
                ::close(fd);
                throw std::runtime_error("Failed to mmap the file: " + path);
            }

            fds_[i] = fd;
            mms_[i] = static_cast<char*>(mm);
        }
        catch (...)
        {
            ::close(fd);
            throw;
        }
    }
}

void classMMap::unmap_all()
{
    for (size_t i = 0; i < mms_.size(); ++i)
    {
        if (mms_[i])
        {
            ::munmap(mms_[i], cfg_.map_size_bytes);
            mms_[i] = nullptr;
        }
        if (fds_[i] != -1)
        {
            ::close(fds_[i]);
            fds_[i] = -1;
        }
    }
    fds_.clear();
    mms_.clear();
}

void classMMap::isValidMemoryMappingAndAddress(int adr, size_t size) const
{
    if (mms_.empty() || current_fd() == -1 || !current_mm())
    {
        throw std::runtime_error("Memory not initialized");
    }
    if (adr < 0 || size > (cfg_.map_size_bytes - static_cast<size_t>(adr)))
    {
        throw std::out_of_range("Out-of-range address");
    }
}

template <typename T> std::optional<T> classMMap::ReadInt(int adr, bool /*is_signed*/) const
{
    try
    {
        isValidMemoryMappingAndAddress(adr, sizeof(T));
        T v{};
        std::memcpy(&v, current_mm() + adr, sizeof(T));
        return v;
    }
    catch (const std::exception& e)
    {
        std::cerr << "ReadIntT except: " << e.what() << std::endl;
        return std::nullopt;
    }
}

template <typename T> void classMMap::WriteInt(int adr, T data, bool /*is_signed*/) const
{
    try
    {
        isValidMemoryMappingAndAddress(adr, sizeof(T));
        std::memcpy(current_mm() + adr, &data, sizeof(T));
    }
    catch (const std::exception& e)
    {
        std::cerr << "WriteIntT except: " << e.what() << std::endl;
    }
}