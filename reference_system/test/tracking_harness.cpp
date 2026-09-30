#include <cassert>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "reference_system/stdout_mutex.hpp"
#include "reference_system/tracking.hpp"

int main()
{
  std::vector<std::string> records;
  reference_system_tracking::TrackingWriter writer(
    "boot-test", "instance-test",
    [&records](const std::string & record) {records.push_back(record);});

  {
    auto callback = writer.begin_callback("FrontLidarDriver", 100);
    writer.report_delivery("FrontLidarDriver", 200);
    writer.write_event(
      "FrontLidarDriver", 100,
      "{\"schema\":1,\"event\":\"source\",\"t_ns\":100}");
    assert(records.size() == 2);
    assert(records[0].find("\"record_seq\":0") != std::string::npos);
    assert(records[0].find("\"through_ns\":99") != std::string::npos);
    assert(records[1].find("\"record_seq\":1") != std::string::npos);
  }
  writer.report_delivery("FrontLidarDriver", 300);
  assert(records.size() == 3);
  assert(records[2].find("\"record_seq\":2") != std::string::npos);
  assert(records[2].find("\"through_ns\":300") != std::string::npos);
  assert(records[0].find("\"schema\":2") != std::string::npos);
  assert(records[0].find("\"producer\":\"FrontLidarDriver\"") != std::string::npos);

  uint64_t captured_entry = 0;
  std::vector<std::string> ordered_records;
  reference_system_tracking::TrackingWriter ordered_writer(
    "boot-test", "instance-ordered",
    [&ordered_records](const std::string & record) {ordered_records.push_back(record);},
    [&captured_entry] {return captured_entry;});
  captured_entry = 400;
  auto ordered_callback = ordered_writer.begin_callback("Node");
  ordered_writer.report_delivery("Node", 500);
  assert(ordered_records.size() == 1);
  assert(ordered_records[0].find("\"through_ns\":399") != std::string::npos);

  std::ostringstream captured_output;
  auto * previous_buffer = std::cout.rdbuf(captured_output.rdbuf());
  reference_system_tracking::TrackingWriter output_writer(
    "boot-test", "instance-output", [](const std::string & record) {
      reference_system_write_stdout_line(record);
    });
  output_writer.register_producer("Node");
  std::thread reporter([&output_writer] {
    for (uint64_t timestamp = 0; timestamp < 2000; ++timestamp) {
      output_writer.report_delivery("Node", timestamp);
    }
  });
  std::thread logger([] {
    for (int line = 0; line < 2000; ++line) {
      std::lock_guard<std::mutex> lock(reference_system_cout_mutex());
      std::cout << "Sensor FrontLidarDriver: " << line << std::endl;
    }
  });
  reporter.join();
  logger.join();
  std::cout.rdbuf(previous_buffer);
  std::istringstream output_lines(captured_output.str());
  std::string line;
  size_t delivery_lines = 0;
  size_t log_lines = 0;
  std::vector<bool> observed_logs(2000, false);
  while (std::getline(output_lines, line)) {
    if (line.rfind("{\"schema\":2,\"event\":\"delivery\"", 0) == 0) {
      assert(!line.empty() && line.back() == '}');
      assert(line.find("Sensor FrontLidarDriver:") == std::string::npos);
      ++delivery_lines;
    } else if (line.rfind("Sensor FrontLidarDriver: ", 0) == 0) {
      const auto value = std::stoi(line.substr(std::string("Sensor FrontLidarDriver: ").size()));
      assert(value >= 0 && value < static_cast<int>(observed_logs.size()));
      assert(!observed_logs[static_cast<size_t>(value)]);
      observed_logs[static_cast<size_t>(value)] = true;
      ++log_lines;
    } else {
      assert(false && "stdout lines interleaved");
    }
  }
  assert(delivery_lines == 2000);
  assert(log_lines == 2000);
  return 0;
}
