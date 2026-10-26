/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <memory>
#include <vector>

#include "pdo_config.hpp"
namespace dust_control::ethercat {
struct BoardImage {
    std::string name;
    AdapterPdoConfig config;
    AdapterPdoLayout layout;
    std::vector<uint8_t> output, input;
    bool operational = false, complete = false;
};
// A single master thread owns every call, including Open/Close.
class Transport {
   public:
    virtual ~Transport() = default;
    virtual bool Open(uint32_t master_index, std::vector<BoardImage>& boards,
                      std::string& error) = 0;
    virtual bool Receive(std::vector<BoardImage>& boards,
                         std::string& error) = 0;
    virtual bool Send(const std::vector<BoardImage>& boards,
                      std::string& error) = 0;
    virtual void Close() = 0;
};
std::unique_ptr<Transport> CreateIghTransport();
}  // namespace dust_control::ethercat
