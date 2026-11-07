/* SPDX-License-Identifier: Apache-2.0 */
#include "session.hpp"
#include <bit>
#include <cmath>
#include "wire.hpp"
namespace dust_control::host {
namespace dw = dust::device_pdo;
namespace ds = dust::device_sensor_page;
bool EthercatSession::ReceiveDtg1(std::span<const uint8_t> image,uint64_t now,uint64_t session) {
    if (image.size()!=180) return FailInput("DTG1 TxPDO length mismatch");
    const auto* p=image.data();
    remote_.PollTimeout(now/1000);
    if (p[0]==uint8_t(dw::Type::Ready) || p[0]==uint8_t(dw::Type::Status)) {
        dw::View v{};
        if (!dw::Decode(dw::Direction::Tx,p,10,v)) return FailInput("DTG1 invalid status");
        if (ds::Get32(v.payload.data())!=uint32_t(session)) return true;
        if (v.type==dw::Type::Ready) {
            if (active_) return FailInput("DTG1 session regressed; restart required");
            boot_id_=1; // DTG1 has a token acknowledgement, no numeric boot counter.
        } else {
            if (v.payload[4]==3) return FailInput("DTG1 slave protected");
            if (v.payload[4]==2) {boot_id_=1;active_=true;}
            else if (active_) return FailInput("DTG1 session lost");
        }
        return true;
    }
    if (!active_) return true;
    for (size_t i=0;i<6;++i) {
        const auto* record=p+i*10;
        if (std::all_of(record,record+10,[](uint8_t b){return b==0;})) continue;
        dw::View v{};
        if (!dw::Decode(dw::Direction::Tx,record,10,v) || unsigned(v.type)!=unsigned(dw::Type::MotorFirst)+i)
            return FailInput("DTG1 invalid motor page");
        if (dtg1_.have_motor[i] && dtg1_.motor_sequence[i]==v.sequence) continue;
        dtg1_.have_motor[i]=true;dtg1_.motor_sequence[i]=v.sequence;
        const auto& route=dw::kRoutes[i];
        if (options_.decode_motors) motors_.Consume(route.bus,route.feedback,v.payload,now/1000,true);
    }
    // The IMU page is 28 bytes within the middle 60-byte region; its tail
    // belongs to the independently published SBUS page, so pad locally.
    ds::Page imu_page{};std::copy_n(p+60,28,imu_page.begin());
    if (imu_page[0]) {
        ds::Fragment f{};ds::Complete complete{};
        if (!ds::Decode(imu_page.data(),60,f) || f.kind!=ds::Kind::ImuCompact)
            return FailInput("DTG1 invalid onboard IMU page");
        if (dtg1_.imu.Accept(imu_page.data(),60,uint32_t(now/1000),complete)) {
            const float temperature=std::bit_cast<float>(ds::Get32(complete.data.data()+12));
            if (!std::isfinite(temperature)) return FailInput("DTG1 invalid IMU temperature");
            std::array<uint8_t,25> raw{};std::copy_n(complete.data.begin(),12,raw.begin()+1);
            onboard_imu_.Consume(raw,temperature,complete.sequence,now/1000);
        }
    }
    const auto* hi91=p+120;
    if (hi91[0]==dust::hi91::kPageType) {
        if (hi91[0]!=dust::hi91::kPageType) return FailInput("DTG1 invalid HI91 page");
        const auto seq=ds::Get32(hi91+1);
        if (!dtg1_.have_hi91 || seq!=dtg1_.hi91_sequence) {
            if (dtg1_.have_hi91 && !ethercat::wire::Newer(seq,dtg1_.hi91_sequence))
                return FailInput("DTG1 HI91 sequence regressed");
            dust::hi91::Compact fields{};std::copy_n(hi91+dust::hi91::kFieldsOffset,fields.size(),fields.begin());
            const auto frame=dust::hi91::ExpandControlFields(fields,seq);
            imu_.Consume(frame,now/1000);dtg1_.hi91_sequence=seq;dtg1_.have_hi91=true;
        }
    }
    // HI91 latest-value and retained SBUS share the final 60-byte page.
    if (hi91[0] && hi91[0]!=dust::hi91::kPageType) {
        ds::Fragment f{};ds::Complete complete{};
        if (!ds::Decode(hi91,60,f) || f.kind!=ds::Kind::Sbus)
            return FailInput("DTG1 invalid UART page");
        if (dtg1_.sbus.Accept(hi91,60,uint32_t(now/1000),complete)) {
            if (complete.discontinuity) remote_.Discontinuity();
            remote_.Consume({complete.data.data(),complete.length},now/1000);
            dtg1_.sbus_ack=complete.sequence;dtg1_.ack_repeats=2;
        } else if (f.sequence==dtg1_.sbus_ack) dtg1_.ack_repeats=2;
    }
    return true;
}
bool EthercatSession::EncodeDtg1(std::span<uint8_t> image,uint64_t now,uint64_t session,uint32_t flags) {
    namespace w=ethercat::wire;
    if (image.size()!=60 || !uint32_t(session)) return FailInput("DTG1 invalid output/session");
    std::fill(image.begin(),image.end(),0);
    if (flags!=w::kStop && flags!=w::kDiscover && flags!=w::kArm)
        return FailInput("DTG1 invalid control flags");
    if (flags==w::kArm && active_) {
        EthercatTransmitter::Batch batch;
        if (!transmitter_.ReadBatch(0,now,batch)) return FailInput("DTG1 motor command expired");
        if (batch.valid && (!dtg1_.have_command || batch.sequence!=dtg1_.command_source_sequence)) {
            if (batch.count!=6) return FailInput("DTG1 requires six motor commands");
            std::array<bool,6> seen{};++dtg1_.sequence;
            for (size_t i=0;i<batch.count;++i) {
                const auto& frame=batch.frames[i];
                const auto route=std::find_if(dw::kRoutes.begin(),dw::kRoutes.end(),[&](const auto& r){return r.bus==frame.bus && r.command==frame.id;});
                if (route==dw::kRoutes.end()) return FailInput("DTG1 unknown CAN route");
                const auto index=size_t(route-dw::kRoutes.begin());
                if (seen[index]) return FailInput("DTG1 duplicate motor command");
                seen[index]=true;dw::Record record{};
                dw::EncodeMotor(dw::Device(index),dtg1_.sequence,frame.bytes,record);
                std::copy(record.begin(),record.end(),dtg1_.held.begin()+index*10);
            }
            dtg1_.have_command=true;dtg1_.command_source_sequence=batch.sequence;
            std::copy(dtg1_.held.begin(),dtg1_.held.end(),image.begin());return true;
        }
        if (dtg1_.have_command && !dtg1_.ack_repeats) {
            std::copy(dtg1_.held.begin(),dtg1_.held.end(),image.begin());return true;
        }
    }
    dw::Payload payload{};ds::Put32(payload.data(),uint32_t(session));
    payload[5]=dtg1_.ack_repeats?2:0;
    payload[6]=8|16|32|64; // motors/IMU event pages, combined, real feedback.
    payload[7]=8|16|(2<<5); // compact HI91, retained DTG1 mode bits.
    const auto type=flags==w::kStop?dw::Type::Stop:flags==w::kDiscover?dw::Type::Discover:dw::Type::Arm;
    const auto record=dw::Encode(type,++dtg1_.sequence,payload);
    std::copy(record.begin(),record.end(),image.begin());
    ds::Put32(image.data()+14,dtg1_.sbus_ack);
    if (dtg1_.ack_repeats) --dtg1_.ack_repeats;
    return true;
}
}
