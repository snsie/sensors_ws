
#include <condition_variable>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "orbbec_lidar/orbbec_lidar.hpp"

void onDeviceConnected(const std::string &ip, const uint16_t port) {
  std::cout << "New device connected: " << ip << ":" << port << std::endl;
}

void onDeviceDisconnected(const std::string &ip, const uint16_t port) {
  std::cout << "Device disconnected: " << ip << ":" << port << std::endl;
}

int main() {
  std::cout << "This example will discover connected devices and print "
               "their IP addresses and ports."
            << std::endl;
  auto device_manager = std::make_shared<ob_lidar::DeviceManager>();
  device_manager->setOnDeviceChangedCallback(onDeviceConnected,
                                             onDeviceDisconnected);
  std::this_thread::sleep_for(std::chrono::seconds(10));
  return 0;
}
