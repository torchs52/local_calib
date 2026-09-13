#pragma once

#include <cstddef>
#include <cstdint>
#include <tuple>

class SpscSlotController
{
  public:
    static constexpr std::size_t kStateSize = 4;

    SpscSlotController(std::int64_t* state_ptr, int slot_count);

    std::int64_t published_seq() const;

    int reserve_write_slot_latest();
    int reserve_write_slot_sync();

    void publish(int slot);

    std::tuple<int, std::int64_t> acquire_read_slot_latest(std::int64_t last_seq);
    std::tuple<int, std::int64_t> acquire_read_slot_sync();

    void release_read_slot(int slot);

  private:
    static constexpr std::size_t kPublishedSeq = 0;
    static constexpr std::size_t kPublishedSlot = 1;
    static constexpr std::size_t kConsumerSlot = 2;
    static constexpr std::size_t kLastWrittenSlot = 3;

    std::int64_t* state_ptr_ = nullptr;
    int slot_count_ = 0;
};
