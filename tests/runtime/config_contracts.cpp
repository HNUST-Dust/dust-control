/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_config.hpp"
#include "chassis_config.h"
#include "chassis_input_reader.h"
#include "sensor_decoder.hpp"
#include <stdexcept>
#include <iostream>
void Check(bool ok) { if(!ok) throw std::runtime_error("runtime config contract failed"); }
int main(int argc,char **argv) {
  try {
    if(argc==3) {
      bool rejected=false;
      try { dust::config::Load(argv[2]); } catch(const std::exception &) { rejected=true; }
      Check(rejected);
      Check(platform::motor_map::kMotors[0].feedback_id==0x201);
      Check(dust::config::Freshness().motors[0]==3000);
    }
    dust::config::Load(argv[1]);
    const auto &address=platform::motor_map::kMotors[0];
    Check(address.feedback_id==0x202 && address.command_slot==1);
    Check(modules::chassis_config::WheelMotor(modules::Side::kLeft).feedback_id==0x202);
    wbr::host::SensorStore store;
    wbr::host::SensorDecoder decoder(store);
    wbr::bridge::Packet packet{};
    wbr::bridge::Header(packet.data(),wbr::bridge::Kind::Can,10000,1);
    packet[10]=1;
    uint8_t payload[8]{};
    wbr::bridge::CanRecord(packet.data()+16,address.bus,address.feedback_id,payload,8,9000);
    Check(decoder.Consume(packet,10000));
    Check(store.Read().control.motors[0].valid);
    modules::ControllerInputs source;
    for(auto &motor:source.motors) { motor.valid=true; motor.precise_timestamp_us=8000; }
    source.imu.valid=true; source.imu.sequence=1; source.imu.precise_timestamp_us=8000;
    source.remote.sequence=1;
    modules::ChassisInputReader reader;
    modules::ChassisCycleInput input;
    reader.Read(source,input,10000,.001,false);
    Check((input.motor_stale_mask&1)!=0 && (input.motor_stale_mask&2)==0);
    Check(!input.imu_fresh);
    reader.Read(source,input,11501,.001,false); Check(input.remote_fresh);
    reader.Read(source,input,11502,.001,false); Check(!input.remote_fresh);
    bool frozen=false;
    try { dust::config::Load(argv[1]); } catch(const std::exception &) { frozen=true; }
    Check(frozen);
    std::cout<<"Runtime mapping, per-motor freshness and sensor microsecond boundaries passed\n";
    return 0;
  } catch(const std::exception &error) { std::cerr<<error.what()<<'\n'; return 1; }
}
