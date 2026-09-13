#include "error_mmap_writer/error_mmap_writer.h"
#include <cstring>
#include <stdexcept>
#include <chrono>

ErrorMMapWriter::ErrorMMapWriter(const std::vector<std::string>& mmapPaths, const LoggerFunc logfunc)
    : mmap_(createMMapConfig(mmapPaths), logfunc)
{
}

ErrorMMapWriter::~ErrorMMapWriter()
{
    this->close();
}

void ErrorMMapWriter::init()
{
    this->start_write();
    this->mmap_.end_frame(true);
}

void ErrorMMapWriter::start_write() const
{
    this->mmap_.begin_frame();
    this->writeUnixTimeOffsetMs();
}

void ErrorMMapWriter::rotate_if_busy()
{
    this->mmap_.end_frame(false);
}

void ErrorMMapWriter::close()
{
    this->mmap_.dispose();
}

void ErrorMMapWriter::writeStateError(const uint8_t* data, size_t size) const
{
    if (size != StateError_Size)
    {
        throw std::invalid_argument("state error requires exactly 16 bytes");
    }
    const auto adr = this->stateErrorAddress();
    std::vector<unsigned char> buf(data, data + size);

    this->mmap_.WriteBytes(adr, buf);
}

void ErrorMMapWriter::writeActionError(const uint8_t* data, size_t size) const
{
    if (size != ActionError_Size)
    {
        throw std::invalid_argument("action error requires exactly 32 bytes");
    }
    const auto adr = this->actionErrorAddress();
    std::vector<unsigned char> buf(data, data + size);

    this->mmap_.WriteBytes(adr, buf);
}

void ErrorMMapWriter::writeStatus(const uint8_t* data, size_t size) const
{
    if (size != Status_Size)
    {
        throw std::invalid_argument("status requires exactly 3 bytes");
    }
    const auto adr = this->statusAddress();
    std::vector<unsigned char> buf(data, data + size);

    this->mmap_.WriteBytes(adr, buf);
}

void ErrorMMapWriter::writeUnixTimeOffsetMs() const
{
    const auto adr = this->unixTimeAddress();
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    int64_t now_unix_time = now_ms - this->kBaseUnixTimeMs;

    this->mmap_.WriteInt64(adr, now_unix_time);
}

MMapConfig ErrorMMapWriter::createMMapConfig(const std::vector<std::string>& mmapPaths) const
{
    return MMapConfig{.paths = mmapPaths,
                      .map_size_bytes = static_cast<size_t>(ErrorMMapWriter::Map_All),
                      .endian = ByteOrder::LITTLE,
                      .proto = createProtocol()};
}

MMapProtocol ErrorMMapWriter::createProtocol() const
{
    return MMapProtocol{.IsWriting_ADR = ErrorMMapWriter::IsWriting_ADR,
                        .IsReading_ADR = ErrorMMapWriter::IsReading_ADR,
                        .Start_ADR = ErrorMMapWriter::Start_ADR,
                        .UNIX_TIME_ADR = ErrorMMapWriter::UNIX_TIME_ADR};
}