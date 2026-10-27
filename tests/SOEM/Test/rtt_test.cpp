// 最简单的 EtherCAT 过程数据往返时间（RTT）测试。
//
// 流程：打开网卡 -> 扫描从站 -> 映射 PDO -> 进入 OP -> 连续收发 1000 次。
// RTT 定义为 ecx_send_processdata() 到 ecx_receive_processdata() 返回的耗时。
//
// 用法：
//   sudo ./ecat_rtt_test enp2s0
//
// 注意：同一个网卡不能同时跑其他 EtherCAT 主站（包括 hpm_ecat_master）。

#include <soem/soem.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>

namespace {

constexpr int kMeasureIterations = 1000;
constexpr int kWarmupIterations = 100;
constexpr std::size_t kIoMapBytes = 8192;

std::int64_t nowNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int roundtrip(ecx_contextt& ctx) {
  ecx_send_processdata(&ctx);
  return ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
}

// 把要发送的过程数据写成递增计数：前 4 字节是小端 32 位帧计数，
// 其余字节清 0。这样抓包时能直接看出每一帧的数据在依次递增。
void writeOutputCounter(ecx_contextt& ctx, std::uint32_t value) {
  std::uint8_t* outputs = ctx.slavelist[0].outputs;
  std::uint32_t remaining = ctx.slavelist[0].Obytes;
  if (outputs == nullptr || remaining == 0) {
    return;
  }

  for (int i = 0; i < 4 && remaining > 0; ++i, --remaining) {
    *outputs++ = static_cast<std::uint8_t>(value >> (8 * i));
  }
  std::memset(outputs, 0, remaining);
}

void closeMaster(ecx_contextt& ctx) {
  ctx.slavelist[0].state = EC_STATE_INIT;
  ecx_writestate(&ctx, 0);
  ecx_close(&ctx);
}

void printAdapters() {
  ec_adaptert* head = ec_find_adapters();
  std::printf("Available adapters:\n");
  for (ec_adaptert* adapter = head; adapter != nullptr; adapter = adapter->next) {
    std::printf("  %-16s %s\n", adapter->name, adapter->desc);
  }
  ec_free_adapters(head);
}

void printUsage(const char* program) {
  std::printf(
      "Usage: sudo %s <interface> [options]\n"
      "  --rt          use SCHED_FIFO priority 80\n"
      "  --cpu N       pin the test thread to CPU N\n"
      "\n"
      "Example:\n"
      "  sudo %s enp2s0 --rt --cpu 2\n",
      program, program);
}

bool applyRealtime(bool use_rt, int cpu) {
  if (prctl(PR_SET_TIMERSLACK, 1UL) != 0) {
    std::perror("prctl(PR_SET_TIMERSLACK)");
  }

  if (use_rt) {
    sched_param param{};
    param.sched_priority = 80;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (rc != 0) {
      std::fprintf(stderr, "pthread_setschedparam failed: %s\n",
                   std::strerror(rc));
      return false;
    }
  }

  if (cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (rc != 0) {
      std::fprintf(stderr, "pthread_setaffinity_np(cpu=%d) failed: %s\n", cpu,
                   std::strerror(rc));
      return false;
    }
  }

  if (use_rt && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::perror("mlockall");
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  const char* interface = nullptr;
  bool use_rt = false;
  int cpu = -1;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--rt") {
      use_rt = true;
    } else if (arg == "--cpu") {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for --cpu\n");
        return 2;
      }
      cpu = std::atoi(argv[++i]);
    } else if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (interface == nullptr) {
      interface = argv[i];
    } else {
      printUsage(argv[0]);
      return 2;
    }
  }

  if (interface == nullptr) {
    printAdapters();
    printUsage(argv[0]);
    return 1;
  }

  applyRealtime(use_rt, cpu);
  ecx_contextt ctx;
  std::memset(&ctx, 0, sizeof(ctx));
  std::vector<std::uint8_t> io_map(kIoMapBytes, 0);

  if (ecx_init(&ctx, interface) <= 0) {
    std::fprintf(stderr,
                 "ecx_init('%s') failed: raw socket permission or bad interface\n",
                 interface);
    return 1;
  }
  std::printf("interface : %s\n", interface);

  const int slave_count = ecx_config_init(&ctx);
  if (slave_count <= 0) {
    std::fprintf(stderr, "no EtherCAT slaves found\n");
    closeMaster(ctx);
    return 1;
  }

  ecx_config_map_group(&ctx, io_map.data(), 0);
  const ec_groupt* group = &ctx.grouplist[0];
  if (group->Obytes == 0 && group->Ibytes == 0) {
    std::fprintf(stderr, "no process data mapped; check slave PDO settings\n");
    closeMaster(ctx);
    return 1;
  }

  const int expected_wkc = static_cast<int>(group->outputsWKC) * 2 +
                           static_cast<int>(group->inputsWKC);

  std::printf("slaves    : %d\n", slave_count);
  std::printf("mapped    : %u bytes outputs + %u bytes inputs\n",
              static_cast<unsigned>(group->Obytes),
              static_cast<unsigned>(group->Ibytes));
  std::printf("expected  : WKC %d\n", expected_wkc);

  std::printf("bringing slaves to OPERATIONAL ...");
  std::fflush(stdout);
  ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  roundtrip(ctx);

  ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
  ecx_writestate(&ctx, 0);

  bool operational = false;
  for (int attempt = 0; attempt < 20; ++attempt) {
    roundtrip(ctx);
    ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE / 20);
    if (ctx.slavelist[0].state == EC_STATE_OPERATIONAL) {
      operational = true;
      break;
    }
  }
  std::printf(" %s\n", operational ? "ok" : "FAILED");

  if (!operational) {
    ecx_readstate(&ctx);
    for (int i = 1; i <= ctx.slavecount; ++i) {
      const ec_slavet& slave = ctx.slavelist[i];
      if (slave.state != EC_STATE_OPERATIONAL) {
        std::fprintf(stderr, "slave %d state=0x%04X AL=0x%04X %s\n", i,
                     slave.state, slave.ALstatuscode,
                     ec_ALstatuscode2string(slave.ALstatuscode));
      }
    }
    closeMaster(ctx);
    return 1;
  }

  // 发送的过程数据从 0 开始逐帧递增。
  std::uint32_t counter = 0;

  // 预热，避免第一次调用的异常值影响统计。
  for (int i = 0; i < kWarmupIterations; ++i) {
    writeOutputCounter(ctx, counter++);
    roundtrip(ctx);
  }

  double sum_us = 0.0;
  double sum_send_us = 0.0;
  double sum_wait_us = 0.0;
  double min_us = 1e18;
  double max_us = 0.0;
  int wkc_errors = 0;

  for (int i = 0; i < kMeasureIterations; ++i) {
    writeOutputCounter(ctx, counter++);
    const std::int64_t begin_ns = nowNs();
    ecx_send_processdata(&ctx);
    const std::int64_t sent_ns = nowNs();
    const int wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    const std::int64_t end_ns = nowNs();

    const double elapsed_us =
        static_cast<double>(end_ns - begin_ns) / 1000.0;
    sum_us += elapsed_us;
    sum_send_us += static_cast<double>(sent_ns - begin_ns) / 1000.0;
    sum_wait_us += static_cast<double>(end_ns - sent_ns) / 1000.0;
    min_us = std::min(min_us, elapsed_us);
    max_us = std::max(max_us, elapsed_us);
    if (wkc < expected_wkc) {
      ++wkc_errors;
    }
  }

  const double avg_us = sum_us / static_cast<double>(kMeasureIterations);
  const double avg_send_us =
      sum_send_us / static_cast<double>(kMeasureIterations);
  const double avg_wait_us =
      sum_wait_us / static_cast<double>(kMeasureIterations);

  std::printf("\n");
  std::printf("iterations : %d (warmup %d)\n", kMeasureIterations,
              kWarmupIterations);
  std::printf("WKC errors : %d\n", wkc_errors);
  std::printf("send avg   : %.3f us\n", avg_send_us);
  std::printf("wait avg   : %.3f us\n", avg_wait_us);
  std::printf("RTT min    : %.3f us\n", min_us);
  std::printf("RTT avg    : %.3f us\n", avg_us);
  std::printf("RTT max    : %.3f us\n", max_us);
  std::printf("RT mode    : %s, cpu=%d, timer_slack=1ns\n",
              use_rt ? "SCHED_FIFO" : "normal", cpu);

  closeMaster(ctx);
  return wkc_errors == 0 ? 0 : 1;
}
