// Reproduce the BufferCore / tf2_ros timer-map lock inversion deterministically.
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include "tf2/buffer_core.hpp"

int main()
{
  tf2::BufferCore buffer;
  std::mutex external;
  std::atomic<bool> entered{false};
  std::unique_lock<std::mutex> held(external);
  auto callback = [&](auto, const auto &, const auto &, auto, auto) {
      entered = true;
      std::lock_guard<std::mutex> lock(external);
    };
  buffer.addTransformableRequest(callback, "odom", "base", tf2::TimePoint(std::chrono::seconds(1)));
  std::thread publisher([&]() {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "odom";
      transform.child_frame_id = "base";
      transform.header.stamp.sec = 1;
      transform.transform.rotation.w = 1.;
      buffer.setTransform(transform, "regression", false);
    });
  while (!entered.load()) {std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  buffer.addTransformableRequest(callback, "odom", "base", tf2::TimePoint(std::chrono::seconds(2)));
  held.unlock();
  publisher.join();
  std::cout << "PASS: transform callbacks run outside BufferCore locks" << std::endl;
}
