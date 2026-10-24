# Standalone communication benchmarks

These tools remain available for performance investigations and are deliberately
outside the default production dependency graph. These benchmarks use historical firmware protocols and remain independent of the
current USB v4 controller. The active MPC algorithm is in src/chassis/mpc; EtherCAT
code is maintained on its separate development branch. Run
`bash tests/benchmarks/can_usb_stress_test/tools/build_native.sh` inside Linux to build
and run the benchmark's offline checks. Real-device benchmark commands retain
their original opt-in requirements and are not launched by the main build.
