/**
 * ---------------------------
 * 共有メモリ管理クラス
 * ---------------------------
 */
#pragma once
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include "logger/py_logger.h"

enum class ByteOrder
{
    BIG = 0,
    LITTLE = 1
};

ByteOrder GetSystemByteOrder();

struct MMapProtocol
{
    int IsWriting_ADR;
    int IsReading_ADR;
    int Start_ADR;
    int UNIX_TIME_ADR;
};

struct MMapConfig
{
    std::vector<std::string> paths;
    size_t map_size_bytes;
    ByteOrder endian;
    MMapProtocol proto;
};

class classMMap
{
  public:
    explicit classMMap(const MMapConfig& cfg, const LoggerFunc logfunc);
    ~classMMap();

    void begin_frame() const;
    void end_frame(bool is_initial);
    int start_address() const
    {
        return static_cast<int>(cfg_.proto.Start_ADR);
    }

    std::optional<uint64_t> ReadInt64(int adr) const;
    std::optional<uint32_t> ReadInt32(int adr) const;
    std::optional<uint16_t> ReadInt16(int adr) const;
    std::optional<int16_t> ReadSignedInt16(int adr) const;
    std::optional<uint8_t> ReadInt8(int adr) const;
    std::optional<float> ReadFloat(int adr) const;

    void WriteInt64(int adr, uint64_t data) const;
    void WriteInt32(int adr, uint32_t data) const;
    void WriteInt16(int adr, uint16_t data) const;
    void WriteSignedInt16(int adr, int16_t data) const;
    void WriteInt8(int adr, uint8_t data) const;
    void WriteFloat(int adr, float data) const;
    void WriteBytes(int adr, const std::vector<unsigned char>& data) const;
    void WriteBytes(int adr, const Eigen::Ref<const Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>>& data) const;

    size_t get_file_index() const;
    std::optional<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> ReadBytes(size_t index, int adr, int length) const;
    void flush() const;
    void dispose();

  private:
    void ensure_and_map_all();
    void unmap_all();
    void isValidMemoryMappingAndAddress(int adr, size_t size) const;

    inline char* current_mm() const
    {
        return mms_.empty() ? nullptr : mms_[current_index_];
    }
    inline int current_fd() const
    {
        return fds_.empty() ? -1 : fds_[current_index_];
    }

    template <typename T> std::optional<T> ReadInt(int adr, bool is_signed = false) const;
    template <typename T> void WriteInt(int adr, T data, bool is_signed = false) const;

  private:
    MMapConfig cfg_;
    ByteOrder system_endian_;
    std::vector<int> fds_;
    std::vector<char*> mms_;
    size_t current_index_;
    PyLogger logger_;
};