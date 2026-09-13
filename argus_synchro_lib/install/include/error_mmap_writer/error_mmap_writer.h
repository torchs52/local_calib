#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "clsmmap/ClsMMap.h"

class ErrorMMapWriter
{

  private:
    static constexpr int Start_ADR = 0;
    static constexpr int IsWriting_ADR = 0;          // 1byte
    static constexpr int IsReading_ADR = 1;          // 1byte
    static constexpr int UNIX_TIME_ADR = 2;          // 8byte
    static constexpr int StateError_ADR = 10;        // 16byte
    static constexpr int ActionError_ADR = 26;       // 32byte
    static constexpr int Reserved_ADR = 58;          // 3byte
    //static constexpr int Status_ADR = 61;            // 3byte
    //static constexpr int ReduceLoadMode_ADR = 61;    // 1byte
    //static constexpr int CameraConnections_ADR = 62; // 1byte
    //static constexpr int LidarConnections_ADR = 63;  // 1byte
    //static constexpr int Map_All = 64;               // マップサイズ
    static constexpr int Status_ADR = 60;            // 3byte
    static constexpr int ReduceLoadMode_ADR = 60;    // 1byte
    static constexpr int CameraConnections_ADR = 61; // 1byte
    static constexpr int LidarConnections_ADR = 62;  // 1byte
    static constexpr int Map_All = 63;               // マップサイズ

    static constexpr int StateError_Size = 16;
    static constexpr int ActionError_Size = 32;
    static constexpr int Status_Size = 3;
    static constexpr int64_t kBaseUnixTimeMs = 1732600000000LL;

  public:
    explicit ErrorMMapWriter(const std::vector<std::string>& mmapPaths, const LoggerFunc logfunc);
    ~ErrorMMapWriter();

    void init();
    void start_write() const;
    void rotate_if_busy();
    void close();

    void writeStateError(const uint8_t* data, size_t size) const;
    void writeActionError(const uint8_t* data, size_t size) const;
    void writeStatus(const uint8_t* data, size_t size) const;

  private:
    void writeUnixTimeOffsetMs() const;

    int unixTimeAddress() const
    {
        return UNIX_TIME_ADR;
    }
    int stateErrorAddress() const
    {
        return StateError_ADR;
    }
    int actionErrorAddress() const
    {
        return ActionError_ADR;
    }
    int statusAddress() const
    {
        return Status_ADR;
    }

    MMapConfig createMMapConfig(const std::vector<std::string>& mmapPaths) const;
    MMapProtocol createProtocol() const;

    classMMap mmap_;
};