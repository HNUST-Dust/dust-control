/* SPDX-License-Identifier: Apache-2.0 */
#include <iostream>
#include <stdexcept>
#include <thread>

#include "chassis/chassis_observation.hpp"
using namespace dust::chassis;
void Require(bool ok) {
    if (!ok) {
        throw std::runtime_error("Chassis observation contract failed");
    }
}

int main() {
    // 队列满时丢弃新样本，不覆盖未读数据；环绕后仍保持顺序。
    ChassisObservationQueue queue;
    ChassisObservation sample;
    for (unsigned i = 0; i < 63; ++i) {
        sample.sequence = i;
        Require(queue.Push(sample));
    }

    Require(!queue.Push(sample));
    for (unsigned i = 0; i < 63; ++i) {
        Require(queue.Pop(sample));
        Require(sample.sequence == i);
    }

    Require(!queue.Pop(sample));
    std::thread producer([&] {
        for (unsigned i = 0; i < 20000; ++i) {
            ChassisObservation value;
            value.sequence = i;
            while (!queue.Push(value)) {
                std::this_thread::yield();
            }
        }
    });
    for (unsigned i = 0; i < 20000; ++i) {
        while (!queue.Pop(sample)) {
            std::this_thread::yield();
        }
        Require(sample.sequence == i);
    }

    producer.join();
    std::cout << "Chassis observation queue saturation, wrap and concurrent "
                 "ordering passed\n";
}
