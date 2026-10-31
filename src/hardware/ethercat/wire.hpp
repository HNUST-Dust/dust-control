/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <cstdint>
#include <span>
namespace dust_control::ethercat::wire {
inline constexpr uint16_t kMajor = 5, kMinor = 0;
inline constexpr uint32_t kDiscover = 1, kArm = 2, kStop = 4;
inline constexpr uint32_t kReady = 1, kActive = 2, kProtected = 3;
inline constexpr uint32_t kFirmwareProtection = 1U << 31;
inline constexpr uint16_t kDataValid = 1, kAckValid = 2, kDiscontinuity = 4;
inline uint16_t U16(const uint8_t* p) {
    return uint16_t(p[0]) | uint16_t(p[1]) << 8;
}
inline uint32_t U32(const uint8_t* p) {
    return uint32_t(U16(p)) | uint32_t(U16(p + 2)) << 16;
}
inline uint64_t U64(const uint8_t* p) {
    return uint64_t(U32(p)) | uint64_t(U32(p + 4)) << 32;
}
inline void Put16(uint8_t* p, uint16_t n) {
    p[0] = uint8_t(n);
    p[1] = uint8_t(n >> 8);
}
inline void Put32(uint8_t* p, uint32_t n) {
    Put16(p, uint16_t(n));
    Put16(p + 2, uint16_t(n >> 16));
}
inline void Put64(uint8_t* p, uint64_t n) {
    Put32(p, uint32_t(n));
    Put32(p + 4, uint32_t(n >> 32));
}
inline bool Newer(uint32_t n, uint32_t old) {
    const auto delta = uint32_t(n - old);
    return delta && delta < 0x80000000U;
}
}  // namespace dust_control::ethercat::wire
