#include <cassert>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include "reference_system/stdout_mutex.hpp"
#include "reference_system/telemetry_buffer.hpp"
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

  alignas(64) uint8_t memory[reference_system_telemetry::header_bytes + 4096]{};
  reference_system_telemetry::Identity identity{"run", "Node", "boot-test", "instance-memory"};
  assert(reference_system_telemetry::initialize(memory, sizeof(memory), identity));
  reference_system_telemetry::MemoryWriter buffer(memory, sizeof(memory), identity);
  size_t formatted_writes = 0;
  reference_system_tracking::TrackingWriter memory_writer(
    "boot-test", "instance-memory", [&formatted_writes](const std::string &) {++formatted_writes;},
    [] {return uint64_t(200);}, reference_system_tracking::WriterMode::memory,
    reference_system_tracking::TrackingWriter::kDefaultQueueCapacity, &buffer);
  memory_writer.register_producer("Node");
  {
    auto active = memory_writer.begin_callback("Node", 100);
    memory_writer.report_delivery("Node", 200);
    memory_writer.write_source("Node", 3, 1, 100);
  }
  memory_writer.report_delivery("Node", 300);
  std::vector<std::pair<uint32_t, reference_system_tracking::TrackingMessage>> inputs{
    {0, {"SensorA", 3, 1}}, {1, {"SensorB", 4, 2}}};
  memory_writer.write_link("Node", 8, 2, 310, inputs, {1, 3, 100}, false);
  memory_writer.write_sink("Node", 9, 320, {"InputNode", 7, 3}, {0, 0, 0});
  reference_system_telemetry::RecordView timing;
  timing.kind = reference_system_telemetry::Kind::timing;
  timing.timestamp_ns = 321;
  timing.name = "Sensor";
  timing.aux_name = "Node";
  timing.value_ns = 4000;
  memory_writer.write_auxiliary("Node", timing);
  assert(reference_system_telemetry::get64(memory + 704) == 6);
  assert(formatted_writes == 0);
  assert(reference_system_telemetry::failure(memory) == 0);
  assert(reference_system_telemetry::committed(memory) > 120);
  auto read64 = [](const uint8_t * p) {return reference_system_telemetry::get64(p);};
  const auto * first = memory + reference_system_telemetry::header_bytes;
  assert(read64(first + 8) == 0 && read64(first + 32) == 99);
  assert(read64(first + 40 + 8) == 1 && first[40 + 4] == 1);
  assert(read64(first + 80 + 8) == 2 && read64(first + 80 + 32) == 300);
  const auto * link_record = first + 120;
  assert(link_record[4] == static_cast<uint8_t>(reference_system_telemetry::Kind::link));
  assert(read64(link_record + 8) == 3);
  assert(reference_system_telemetry::get32(link_record + 32) == 8);
  assert(reference_system_telemetry::get32(link_record + 36) == 2);
  assert(reference_system_telemetry::get32(link_record + 40) == 1);
  assert(reference_system_telemetry::get32(link_record + 44) == 3);
  assert(read64(link_record + 48) == 100);
  const auto * sink_record = link_record + reference_system_telemetry::get32(link_record);
  assert(sink_record[4] == static_cast<uint8_t>(reference_system_telemetry::Kind::sink));
  assert(read64(sink_record + 8) == 4);
  assert(reference_system_telemetry::get32(sink_record + 32) == 9);
  assert(reference_system_telemetry::get32(sink_record + 36) == 0); // null source
  std::vector<std::string> file_records;
  reference_system_tracking::TrackingWriter file_writer("boot-test", "instance-memory",
    [&file_records](const std::string & text) {file_records.push_back(text);});
  file_writer.write_link("Node", 8, 2, 310, inputs, {1, 3, 100}, false);
  file_writer.write_sink("Node", 9, 320, {"InputNode", 7, 3}, {0, 0, 0});
  assert(file_records.size() == 2);
  assert(file_records[0].find("\"seq\":8,\"channel\":2") != std::string::npos);
  assert(file_records[0].find("\"index\":1,\"msg\":{\"name\":\"SensorB\"") != std::string::npos);
  assert(file_records[1].find("\"source\":null") != std::string::npos);
  memory_writer.shutdown();
  assert(reference_system_telemetry::shared_load(memory, 704) == 7);
  bool stopped_callbacks = false;
  try {auto guard = memory_writer.begin_callback("Node", 400);} catch (const std::runtime_error &) {
    stopped_callbacks = true;
  }
  assert(stopped_callbacks);
  alignas(64) uint8_t small[reference_system_telemetry::header_bytes + 40]{};
  assert(reference_system_telemetry::initialize(small, sizeof(small), identity));
  reference_system_telemetry::MemoryWriter small_buffer(small, sizeof(small), identity);
  reference_system_tracking::TrackingWriter full_writer("boot-test", "instance-memory",
    [](const std::string &) {assert(false && "memory mode wrote JSON");},
    [] {return uint64_t(200);}, reference_system_tracking::WriterMode::memory,
    reference_system_tracking::TrackingWriter::kDefaultQueueCapacity, &small_buffer);
  full_writer.write_source("Node", 1, 1, 100);
  bool overflow_reported = false;
  try {full_writer.report_delivery("Node", 200);} catch (const std::runtime_error &) {overflow_reported = true;}
  assert(overflow_reported && reference_system_telemetry::failure(small) == 1);
  assert(reference_system_telemetry::committed(small) == 40);
  assert(reference_system_telemetry::get64(small + reference_system_telemetry::header_bytes + 8) == 0);
  bool no_resume = false;
  try {full_writer.write_source("Node", 2, 1, 201);} catch (const std::runtime_error &) {no_resume = true;}
  assert(no_resume && reference_system_telemetry::committed(small) == 40);
  alignas(64) uint8_t oversized[reference_system_telemetry::header_bytes + 4096]{};
  assert(reference_system_telemetry::initialize(oversized, sizeof(oversized), identity));
  reference_system_telemetry::MemoryWriter oversized_buffer(oversized, sizeof(oversized), identity);
  reference_system_tracking::TrackingWriter oversized_writer("boot-test", "instance-memory",
    [](const std::string &) {assert(false);}, reference_system_tracking::monotonic_now_ns,
    reference_system_tracking::WriterMode::memory,
    reference_system_tracking::TrackingWriter::kDefaultQueueCapacity, &oversized_buffer);
  std::vector<std::pair<uint32_t, reference_system_tracking::TrackingMessage>> many_inputs(65);
  bool invalid_rejected = false;
  try {oversized_writer.write_link("Node", 1, 0, 100, many_inputs, {}, false);}
  catch (const std::runtime_error &) {invalid_rejected = true;}
  assert(invalid_rejected && reference_system_telemetry::failure(oversized) == 2);
  alignas(64) uint8_t concurrent[reference_system_telemetry::header_bytes + 4096]{};
  assert(reference_system_telemetry::initialize(concurrent, sizeof(concurrent), identity));
  reference_system_telemetry::MemoryWriter concurrent_buffer(concurrent, sizeof(concurrent), identity);
  reference_system_tracking::TrackingWriter concurrent_writer("boot-test", "instance-memory",
    [](const std::string &) {assert(false);}, reference_system_tracking::monotonic_now_ns,
    reference_system_tracking::WriterMode::memory,
    reference_system_tracking::TrackingWriter::kDefaultQueueCapacity, &concurrent_buffer);
  std::thread producer_a([&] {
    for (uint32_t i = 0; i < 40; ++i) concurrent_writer.write_source("Node", i, 1, 100 + i);
  });
  std::thread producer_b([&] {
    for (uint32_t i = 0; i < 40; ++i) concurrent_writer.write_source("Node", i, 1, 100 + i);
  });
  producer_a.join();
  producer_b.join();
  assert(reference_system_telemetry::committed(concurrent) == 80 * 40);
  assert(reference_system_telemetry::failure(concurrent) == 0);
  for (uint64_t i = 0; i < 80; ++i) {
    assert(reference_system_telemetry::get64(concurrent +
      reference_system_telemetry::header_bytes + i * 40 + 8) == i);
  }
  char path[] = "/tmp/opencode/tracking-buffer-XXXXXX";
  const int fd = mkstemp(path);
  assert(fd >= 0);
  assert(ftruncate(fd, static_cast<off_t>(sizeof(memory))) == 0);
  auto * mapped = static_cast<uint8_t *>(mmap(nullptr, sizeof(memory),
      PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
  assert(mapped != MAP_FAILED);
  reference_system_telemetry::Identity file_id{"run", "MappedNode", "boot-test", "instance-memory"};
  assert(reference_system_telemetry::initialize(mapped, sizeof(memory), file_id));
  assert(!reference_system_telemetry::MemoryWriter(mapped, sizeof(memory), identity).valid());
  assert(munmap(mapped, sizeof(memory)) == 0);
  assert(close(fd) == 0);
  setenv("REFERENCE_SYSTEM_TELEMETRY_TRANSPORT", "memory", 1);
  setenv("REFERENCE_SYSTEM_TELEMETRY_BUFFER_PATH", path, 1);
  setenv("REFERENCE_SYSTEM_TELEMETRY_PRODUCER", "MappedNode", 1);
  setenv("REFERENCE_SYSTEM_RUN_ID", "run", 1);
  setenv("REFERENCE_SYSTEM_BOOT_ID", "boot-test", 1);
  setenv("REFERENCE_SYSTEM_INSTANCE_ID", "instance-memory", 1);
  assert(reference_system_tracking::writer_mode_from_environment() == reference_system_tracking::WriterMode::memory);
  {
    reference_system_tracking::Runtime runtime;
    runtime.writer().register_producer("MappedNode");
    runtime.writer().write_source("MappedNode", 1, 1, 101);
  }
  assert(unlink(path) == 0);
  return 0;
}
