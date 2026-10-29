// Exercise telemetry without requesting a master or sending any bus traffic.
#define main dust_bench_entry_for_contracts
#include "../../tools/ethercat_dm_rtt.cpp"
#undef main
#include <cstdio>
#include <iterator>
int main(int argc, char** argv) {
    Check(argc == 2, "Temporary output filename required");
    unsetenv("DUST_RTT_CYCLE_LOG");
    CycleLog off;
    off.Prepare(125);
    CycleRecord row{12, 3, 8, 2, 3, 2, 100, 110, 120, 90, 130, 140, .25};
    off.Add(row);
    off.Write();
    Check(!off.enabled() && off.rows.empty() && !off.capacity,
          "Disabled telemetry must not allocate or log");
    setenv("DUST_RTT_CYCLE_LOG", argv[1], 1);
    CycleLog on;
    on.Prepare(125);
    const auto allocation = on.rows.data();
    on.Add(row);
    row.iteration = 13;
    row.send_enter = row.send_return = 0; // final receive before stop send
    on.Add(row);
    Check(on.rows.data() == allocation, "Cyclic logging reallocated storage");
    on.capacity = on.rows.size();
    bool overflow = false;
    try { on.Add(row); } catch (const std::runtime_error&) { overflow = true; }
    Check(overflow && on.rows.size() == 2, "Overflow must fail without dropping");
    on.Write();
    std::ifstream input(argv[1]);
    const std::string csv{std::istreambuf_iterator<char>(input), {}};
    Check(std::count(csv.begin(), csv.end(), '\n') == 3, "CSV row count");
    Check(csv.find("12,3,8,2,3,2,100,110,120,90,130,140,0.250\n") !=
          std::string::npos, "Cycle values/precision changed");
    Check(csv.find("13,3,8,2,3,2,100,110,120,90,0,0,0.250\n") !=
          std::string::npos, "Final receive must allow missing normal send");
    on.path = "/no-such-cycle-log-directory/out.csv";
    bool failed = false;
    try { on.Write(); } catch (const std::runtime_error&) { failed = true; }
    Check(failed, "File errors must fail clearly");
    std::remove(argv[1]);
    std::cout << "Cycle telemetry contracts PASS\n";
}
