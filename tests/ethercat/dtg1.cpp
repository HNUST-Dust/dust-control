/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <algorithm>
#include "hardware/ethercat/session.hpp"
#include "hardware/ethercat/wire.hpp"
#include "chassis/runtime/realtime_loop.hpp"
namespace ec=dust_control::ethercat;
namespace host=dust_control::host;
namespace dw=dust::device_pdo;
namespace ds=dust::device_sensor_page;
int main() {
    auto capacity=ec::Dtg1Capacity();ec::PdoConfig config;config.cycle_time_us=125;config.adapters["base"]=capacity;
    const auto layout=ec::BuildPdoLayout(config).adapters.at("base");
    assert(layout.rx.bytes==60 && layout.tx.bytes==180 && layout.layout_signature==0x44544731);
    host::EthercatDeviceOptions options;options.uart_sensors={5,2,0};options.decode_motors=false;
    host::EthercatSession session(capacity,layout,options);
    std::array<host::CanCommandGroup,6> groups{};
    for (size_t i=0;i<6;++i) groups[i]={dw::kRoutes[i].bus,dw::kRoutes[i].command};
    assert(session.transmitter().ConfigureCan(host::CanProducer::kChassis,groups,20000000));
    assert(!session.transmitter().ConfigureCan(host::CanProducer::kGimbal,groups,20000000));
    session.transmitter().SealRegistration();
    const auto now=dust_control::realtime::MonotonicTimeNs();constexpr uint32_t nonce=77;
    std::array<uint8_t,180> input{};std::array<uint8_t,60> output{};
    dw::Payload token{};ds::Put32(token.data(),nonce);
    auto ready=dw::Encode(dw::Type::Ready,1,token);std::copy(ready.begin(),ready.end(),input.begin());
    assert(session.Receive(input,now,nonce));assert(session.boot_id() && !session.active());
    assert(session.Encode(output,now,nonce,1,ec::wire::kArm));assert(output[0]==uint8_t(dw::Type::Arm));
    token[4]=2;auto status=dw::Encode(dw::Type::Status,2,token);std::copy(status.begin(),status.end(),input.begin());
    assert(session.Receive(input,now,nonce));assert(session.active());session.Activate(now);
    std::array<dust_control::devices::CanFrame,6> frames{};
    for(size_t i=0;i<6;++i)frames[i]={groups[i].bus,groups[i].id,{uint8_t(i+1)}};
    assert(session.transmitter().SubmitCan(host::CanProducer::kChassis,frames,1,now));
    assert(session.Encode(output,now,nonce,2,ec::wire::kArm));uint8_t sequence;
    assert(dw::DecodeMotorBatch(output.data(),60,sequence));const auto held=output;
    assert(session.Encode(output,now+125000,nonce,3,ec::wire::kArm));assert(output==held);
    // A retained SBUS page must be decoded and ACKed independently of motor snapshots.
    input.fill(0);std::array<uint8_t,25> sbus{};sbus[0]=15;
    ds::Page page{};assert(ds::Encode(ds::Kind::Sbus,1,123,false,sbus.data(),sbus.size(),0,page));
    std::copy(page.begin(),page.end(),input.begin()+120);
    assert(session.Receive(input,now+250000,nonce));assert(session.wfly_sbus_decoder().Read().remote.sequence);
    assert(session.Encode(output,now+375000,nonce,4,ec::wire::kArm));
    assert(output[0]==uint8_t(dw::Type::Arm) && (output[7]&2) && ds::Get32(output.data()+14)==1);
    assert(!session.Encode(output,now+20000000,nonce,5,ec::wire::kArm));
    // Stop remains encodable after a producer timeout has failed the transmitter.
    assert(session.Encode(output,now+20000000,nonce,6,ec::wire::kStop));assert(output[0]==uint8_t(dw::Type::Stop));
    host::EthercatSession partial(capacity,layout,options);
    assert(partial.transmitter().ConfigureCan(host::CanProducer::kChassis,groups,20000000));partial.Activate(now);
    std::copy(status.begin(),status.end(),input.begin());assert(partial.Receive(input,now,nonce));
    assert(partial.transmitter().SubmitCan(host::CanProducer::kChassis,{frames.data(),5},1,now));
    assert(!partial.Encode(output,now,nonce,1,ec::wire::kArm));
}
