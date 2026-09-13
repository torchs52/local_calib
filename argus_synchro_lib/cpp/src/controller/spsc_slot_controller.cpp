#include "controller/spsc_slot_controller.h"

#include <stdexcept>

namespace
{
inline std::int64_t atomic_load_acquire(const std::int64_t* p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}

inline std::int64_t atomic_load_relaxed(const std::int64_t* p)
{
    return __atomic_load_n(p, __ATOMIC_RELAXED);
}

inline void atomic_store_release(std::int64_t* p, std::int64_t v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}

inline bool atomic_cas_acq_rel(std::int64_t* p, std::int64_t expected, std::int64_t desired)
{
    return __atomic_compare_exchange_n(p, &expected, desired, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}
} // namespace

SpscSlotController::SpscSlotController(std::int64_t* state_ptr, int slot_count)
    : state_ptr_(state_ptr), slot_count_(slot_count)
{
    if (state_ptr_ == nullptr)
    {
        throw std::invalid_argument("state_ptr must not be null");
    }
    if (slot_count_ < 2)
    {
        throw std::invalid_argument("slot_count must be >= 2");
    }
}

std::int64_t SpscSlotController::published_seq() const
{
    return atomic_load_acquire(&state_ptr_[kPublishedSeq]);
}

int SpscSlotController::reserve_write_slot_latest()
{
    const std::int64_t consumer_slot = atomic_load_acquire(&state_ptr_[kConsumerSlot]);
    const std::int64_t published_slot = atomic_load_acquire(&state_ptr_[kPublishedSlot]);
    const std::int64_t last_written = atomic_load_relaxed(&state_ptr_[kLastWrittenSlot]);

    for (int i = 1; i <= slot_count_; ++i)
    {
        const int cand = static_cast<int>((last_written + i) % slot_count_);
        if (cand != consumer_slot && cand != published_slot)
        {
            atomic_store_release(&state_ptr_[kLastWrittenSlot], cand);
            return cand;
        }
    }

    int cand = static_cast<int>((last_written + 1) % slot_count_);
    if (cand == consumer_slot)
    {
        cand = static_cast<int>((cand + 1) % slot_count_);
    }
    atomic_store_release(&state_ptr_[kLastWrittenSlot], cand);
    return cand;
}

int SpscSlotController::reserve_write_slot_sync()
{
    const std::int64_t consumer_slot = atomic_load_acquire(&state_ptr_[kConsumerSlot]);
    const std::int64_t last_written = atomic_load_relaxed(&state_ptr_[kLastWrittenSlot]);

    int cand = static_cast<int>((last_written + 1) % slot_count_);
    if (cand == consumer_slot)
    {
        cand = static_cast<int>((cand + 1) % slot_count_);
    }
    atomic_store_release(&state_ptr_[kLastWrittenSlot], cand);
    return cand;
}

void SpscSlotController::publish(int slot)
{
    atomic_store_release(&state_ptr_[kPublishedSlot], slot);
    __atomic_add_fetch(&state_ptr_[kPublishedSeq], 1, __ATOMIC_RELEASE);
}

std::tuple<int, std::int64_t> SpscSlotController::acquire_read_slot_latest(std::int64_t last_seq)
{
    while (true)
    {
        const std::int64_t seq1 = atomic_load_acquire(&state_ptr_[kPublishedSeq]);
        const std::int64_t slot = atomic_load_acquire(&state_ptr_[kPublishedSlot]);
        if (slot < 0 || seq1 <= last_seq)
        {
            return std::make_tuple(-1, seq1);
        }

        atomic_store_release(&state_ptr_[kConsumerSlot], slot);

        const std::int64_t seq2 = atomic_load_acquire(&state_ptr_[kPublishedSeq]);
        const std::int64_t slot2 = atomic_load_acquire(&state_ptr_[kPublishedSlot]);
        if (seq1 == seq2 && slot == slot2)
        {
            return std::make_tuple(static_cast<int>(slot), seq1);
        }

        atomic_store_release(&state_ptr_[kConsumerSlot], -1);
    }
}

std::tuple<int, std::int64_t> SpscSlotController::acquire_read_slot_sync()
{
    while (true)
    {
        const std::int64_t slot = atomic_load_acquire(&state_ptr_[kPublishedSlot]);
        if (slot < 0)
        {
            return std::make_tuple(-1, atomic_load_acquire(&state_ptr_[kPublishedSeq]));
        }

        if (atomic_cas_acq_rel(&state_ptr_[kPublishedSlot], slot, -1))
        {
            atomic_store_release(&state_ptr_[kConsumerSlot], slot);
            return std::make_tuple(static_cast<int>(slot), atomic_load_acquire(&state_ptr_[kPublishedSeq]));
        }
    }
}

void SpscSlotController::release_read_slot(int slot)
{
    (void)slot;
    atomic_store_release(&state_ptr_[kConsumerSlot], -1);
}
