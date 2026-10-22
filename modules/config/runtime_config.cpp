/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_config.hpp"
#include <chassis_controller/motor_map.h>
#include <yaml-cpp/yaml.h>
#include <set>
#include <stdexcept>
#include <filesystem>

namespace {
struct Settings {
  std::array<platform::motor_map::MotorAddress,6> motors = platform::motor_map::kDefaultMotors;
  dust::config::Deadlines deadlines;
};
Settings &Storage() { static Settings value; return value; }
void Keys(const YAML::Node &node, const std::set<std::string> &allowed) {
  if (!node.IsMap()) throw std::runtime_error("Configuration must be a mapping");
  std::set<std::string> seen;
  for (const auto &entry:node) {
    const auto key=entry.first.as<std::string>();
    if (!allowed.count(key) || !seen.insert(key).second)
      throw std::runtime_error("Unknown or duplicate configuration key: "+key);
  }
  if (seen != allowed) throw std::runtime_error("Missing configuration keys");
}
uint64_t Integer(const YAML::Node &node, uint64_t maximum) {
  const auto text=node.as<std::string>();
  if (text.empty() || text[0]=='-' || text[0]=='+') throw std::runtime_error("Expected unsigned integer");
  size_t used=0;
  const auto value=std::stoull(text,&used,0);
  if (used!=text.size() || value>maximum) throw std::runtime_error("Configuration integer out of range");
  return value;
}
uint64_t Deadline(const YAML::Node &node, uint64_t maximum) {
  const auto value=Integer(node,maximum);
  if (!value) throw std::runtime_error("freshness_deadline_us must be positive");
  return value;
}
}
namespace platform::motor_map {
const std::array<MotorAddress,6> &Addresses() { return Storage().motors; }
const std::array<MotorAddress,6> &kMotors = Addresses();
}
namespace dust::config {
const Deadlines &Freshness() { return Storage().deadlines; }
void Load(const std::string &directory) {
  static bool loaded=false;
  if (loaded) throw std::runtime_error("Configuration is startup-only");
  auto candidate=Storage();
  const std::array<std::string,6> names={"left_wheel","right_wheel","left_joint_b","left_joint_d","right_joint_b","right_joint_d"};
  const auto root=YAML::LoadFile((std::filesystem::path(directory)/"motors_params.yaml").string());
  auto keys=std::set<std::string>(names.begin(),names.end()); keys.insert("schema_version");
  Keys(root,keys);
  if (Integer(root["schema_version"],1)!=1) throw std::runtime_error("Unsupported motors schema version");
  std::set<std::pair<unsigned,unsigned>> feedback;
  std::set<std::pair<unsigned,unsigned>> commands;
  for (size_t i=0;i<6;++i) {
    const auto motor=root[names[i]];
    auto expected=std::set<std::string>{"bus","command_id","feedback_id","freshness_deadline_us"};
    if (i<2) expected.insert("command_slot");
    Keys(motor,expected);
    auto &address=candidate.motors[i];
    address.bus=Integer(motor["bus"],3);
    if (address.bus!=2 && address.bus!=3) throw std::runtime_error("Gateway supports only CAN2/CAN3");
    address.command_id=Integer(motor["command_id"],0x7ff);
    address.feedback_id=Integer(motor["feedback_id"],0x7ff);
    address.command_slot=i<2 ? Integer(motor["command_slot"],3):0;
    // Preserve the current DJI codec and feedback-slot correspondence.
    if (i<2 && (address.command_id!=0x200 || address.feedback_id!=0x201+address.command_slot))
      throw std::runtime_error("DJI command/feedback slot mismatch");
    if (!feedback.insert({address.bus,address.feedback_id}).second ||
        !commands.insert({address.bus,address.command_id}).second)
      throw std::runtime_error("Duplicate motor CAN address");
    candidate.deadlines.motors[i]=Deadline(motor["freshness_deadline_us"],UINT32_MAX-1ULL);
  }
  const auto sensors=YAML::LoadFile((std::filesystem::path(directory)/"sensors_params.yaml").string());
  Keys(sensors,{"schema_version","hi91","remote"});
  if (Integer(sensors["schema_version"],1)!=1) throw std::runtime_error("Unsupported sensor schema version");
  Keys(sensors["hi91"],{"freshness_deadline_us"});
  Keys(sensors["remote"],{"freshness_deadline_us"});
  candidate.deadlines.hi91_us=Deadline(sensors["hi91"]["freshness_deadline_us"],UINT32_MAX-1ULL);
  candidate.deadlines.remote_us=Deadline(sensors["remote"]["freshness_deadline_us"],UINT32_MAX-1ULL);
  Storage()=candidate; // Publish only after every file validates, before threads exist.
  loaded=true;
}
}
