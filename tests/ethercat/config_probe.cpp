/* SPDX-License-Identifier: Apache-2.0 */
#include <iostream>

#include "dust_control.hpp"
int main(int argc, char** argv) {
    if (argc != 2) {
        return 2;
    }
    try {
        dust_control::app::DustControl app{std::string(argv[1])};
        std::cout << "Configuration valid\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
