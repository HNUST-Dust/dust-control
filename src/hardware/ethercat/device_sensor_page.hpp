/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <algorithm>
#include <array>
#include <cstdint>

#include "pdo_schema.hpp"
#include "wire.hpp"

namespace dust_control::ethercat::sensor {
constexpr size_t kBytes = pdo::kHi91Bytes;
constexpr size_t kHeaderBytes = 12;
constexpr size_t kChunkBytes = kBytes - kHeaderBytes;
enum class Kind : uint8_t { Sbus = 0x41, ImuCompact = 0x45 };
using Page = std::array<uint8_t, kBytes>;
inline void Put32(uint8_t* bytes, uint32_t value) {
    wire::Put32(bytes, value);
}
inline uint32_t Get32(const uint8_t* bytes) {
    return wire::U32(bytes);
}
struct Fragment {
    Kind kind{};
    uint32_t sequence = 0, capture_us = 0;
    uint8_t length = 0;
    bool discontinuity = false;
    std::array<uint8_t, kChunkBytes> data{};
};
inline bool Decode(const uint8_t* bytes, size_t size, Fragment& out) {
    if (!bytes || size != kBytes) {
        return false;
    }
    const auto kind = Kind(bytes[0]);
    const auto length = bytes[11];
    if ((kind != Kind::Sbus && kind != Kind::ImuCompact) || !(bytes[1] & 1) ||
        (bytes[1] & ~3U) || bytes[10] || !length ||
        (kind == Kind::Sbus ? length > 32 : length != 16) ||
        !std::all_of(bytes + kHeaderBytes + length, bytes + kBytes,
                     [](uint8_t value) { return value == 0; })) {
        return false;
    }
    out = {kind,   Get32(bytes + 2),   Get32(bytes + 6),
           length, bool(bytes[1] & 2), {}};
    std::copy_n(bytes + kHeaderBytes, length, out.data.begin());
    return true;
}
inline bool Encode(Kind kind, uint32_t sequence, uint32_t capture_us,
                   bool discontinuity, const uint8_t* data, size_t size,
                   Page& page) {
    if (!data || !size ||
        (kind == Kind::Sbus ? size > 32
                            : kind != Kind::ImuCompact || size != 16)) {
        return false;
    }
    page = {};
    page[0] = uint8_t(kind);
    page[1] = 1 | (discontinuity ? 2 : 0);
    Put32(page.data() + 2, sequence);
    Put32(page.data() + 6, capture_us);
    page[11] = uint8_t(size);
    std::copy_n(data, size, page.begin() + kHeaderBytes);
    return true;
}
}  // namespace dust_control::ethercat::sensor
