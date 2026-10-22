# Standalone communication benchmarks

These tools remain available for performance investigations and are deliberately
outside the default production dependency graph. They do not replace the active
MPC algorithm or EtherCAT backend modules. Run
`bash benchmarks/can_usb_stress_test/tools/build_native.sh` inside Linux to build
and run the benchmark's offline checks. Real-device benchmark commands retain
their original opt-in requirements and are not launched by the main build.
