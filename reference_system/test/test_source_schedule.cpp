// SPDX-License-Identifier: Apache-2.0
// Focused transport tests run without ROS, Docker, or workload launches.
#include "reference_system/source_schedule.hpp"
#ifdef NDEBUG
#undef NDEBUG // Keep transport assertions active in Release container test builds.
#endif
#include <cassert>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <unistd.h>

int main() {
  const auto dir = std::filesystem::temp_directory_path() /
    ("source-schedule-" + std::to_string(getpid()));
  std::filesystem::create_directory(dir);
  const auto prepared = dir / "source.schedule";
  const auto start = dir / "start";
  const auto write = [](const auto & path, const auto & text) {std::ofstream(path) << text;};
  const std::string text = "LAME_SOURCE_SCHEDULE_V1 session boot hash Source instance 3000\n3\nA1 0 100\nB 1000 200\nA2 2000 100\n";
  write(prepared, text);
  auto rejects = [](std::function<void()> f) {bool failed=false;try {f();} catch (const std::exception &) {failed=true;} assert(failed);};
  rejects([&]{nodes::SourceSchedule s(prepared, "Other", "boot", "instance", 100);});
  rejects([&]{nodes::SourceSchedule s(prepared, "Source", "old-boot", "instance", 100);});
  rejects([&]{nodes::SourceSchedule s(prepared, "Source", "boot", "old-instance", 100);});
  nodes::SourceSchedule s(prepared, "Source", "boot", "instance", 100);
  assert(s.due(5000).empty()); // Prepared is not armed.
  write(start, "LAME_SOURCE_START_V1 session boot hash 5000\n");
  assert(s.due(4999).empty());
  assert(s.due(5000) == std::vector<size_t>{0});
  s.applied(0);
  assert(s.due(5999).empty());
  assert(s.due(6001) == std::vector<size_t>{1});
  s.applied(1);
  assert(s.due(7001) == std::vector<size_t>{2});
  s.applied(2);
  assert(s.due(100000).empty()); // Final phase holds, no periodic restart.
  nodes::SourceSchedule missed(prepared, "Source", "boot", "instance", 100);
  assert((missed.due(7500) == std::vector<size_t>{0,1,2}));
  assert(missed.requested(2) == 7000);
  std::ostringstream record;
  missed.record(record, 1, "skipped", "Source", "boot", "instance", 100, 100, 7500);
  assert(record.str().find("\"requested_ns\":6000,\"applied_ns\":7500") != std::string::npos);
  assert(record.str().find("\"status\":\"skipped\"") != std::string::npos);
  write(start, "LAME_SOURCE_START_V1 stale boot hash 5000\n");
  nodes::SourceSchedule stale(prepared, "Source", "boot", "instance", 100);
  rejects([&]{stale.due(6000);});
  for (const auto & bad : {std::string("garbage"), text + "extra", std::string("LAME_SOURCE_SCHEDULE_V1 session boot hash Source instance 3000\n3\nA 0 0\nB 1000 200\nC 2000 100\n")}) {
    write(prepared,bad);
    rejects([&]{nodes::SourceSchedule malformed(prepared,"Source","boot","instance",100);});
  }
  std::filesystem::remove_all(dir);
}
