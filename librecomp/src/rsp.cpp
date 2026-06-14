#include <cassert>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include "rsp.hpp"

static recomp::rsp::callbacks_t rsp_callbacks {};
namespace {
constexpr uint32_t kRspDmemTaskOffset = 0xFC0;
constexpr uint32_t kRspDefaultUcodeDataSize = 0xF80;
constexpr uint32_t kOsTaskYielded = 0x0001;

OSTask physicalize_task(const OSTask* task) {
    OSTask ret = *task;

    ret.t.ucode = normalize_rsp_dram_addr(ret.t.ucode);
    ret.t.ucode_data = normalize_rsp_dram_addr(ret.t.ucode_data);
    ret.t.dram_stack = normalize_rsp_dram_addr(ret.t.dram_stack);
    ret.t.output_buff = normalize_rsp_dram_addr(ret.t.output_buff);
    ret.t.output_buff_size = normalize_rsp_dram_addr(ret.t.output_buff_size);
    ret.t.data_ptr = normalize_rsp_dram_addr(ret.t.data_ptr);
    ret.t.yield_data_ptr = normalize_rsp_dram_addr(ret.t.yield_data_ptr);

    if ((ret.t.flags & kOsTaskYielded) != 0) {
        ret.t.ucode_data = ret.t.yield_data_ptr;
        ret.t.ucode_data_size = ret.t.yield_data_size;
    }

    return ret;
}
}

void recomp::rsp::set_callbacks(const callbacks_t& callbacks) {
    rsp_callbacks = callbacks;
}

uint8_t dmem[0x1000];
uint16_t rspReciprocals[512];
uint16_t rspInverseSquareRoots[512];

// From Ares emulator. For license details, see rsp_vu.h
void recomp::rsp::constants_init() {
    rspReciprocals[0] = u16(~0);
    for (u16 index = 1; index < 512; index++) {
        u64 a = index + 512;
        u64 b = (u64(1) << 34) / a;
        rspReciprocals[index] = u16((b + 1) >> 8);
    }

    for (u16 index = 0; index < 512; index++) {
        u64 a = (index + 512) >> ((index % 2 == 1) ? 1 : 0);
        u64 b = 1 << 17;
        //find the largest b where b < 1.0 / sqrt(a)
        while (a * (b + 1) * (b + 1) < (u64(1) << 44)) b++;
        rspInverseSquareRoots[index] = u16(b >> 1);
    }
}

// Runs a recompiled RSP microcode
bool recomp::rsp::run_task(uint8_t* rdram, const OSTask* task) {
    assert(rsp_callbacks.get_rsp_microcode != nullptr);
    RspUcodeFunc* ucode_func = rsp_callbacks.get_rsp_microcode(task);

    if (ucode_func == nullptr) {
        fprintf(stderr, "No registered RSP ucode for %" PRIu32 " (returned `nullptr`)\n", task->t.type);
        return false;
    }

    OSTask physical_task = physicalize_task(task);

    // Load the OSTask into DMEM
    memcpy(&dmem[kRspDmemTaskOffset], &physical_task, sizeof(OSTask));

    // Load the ucode data into DMEM
    uint32_t ucode_data_size = physical_task.t.ucode_data_size;
    if (ucode_data_size == 0 || ucode_data_size > kRspDefaultUcodeDataSize) {
        ucode_data_size = kRspDefaultUcodeDataSize;
    }
    dma_rdram_to_dmem(rdram, 0x0000, physical_task.t.ucode_data, ucode_data_size - 1);

    // Run the ucode
    RspExitReason exit_reason = ucode_func(rdram, physical_task.t.ucode);

    // Ensure that the ucode exited correctly
    if (exit_reason != RspExitReason::Broke) {
        fprintf(stderr, "RSP ucode %" PRIu32 " exited unexpectedly. exit_reason: %i\n", task->t.type, static_cast<int>(exit_reason));
        assert(exit_reason == RspExitReason::Broke);
        return false;
    }

    return true;
}
