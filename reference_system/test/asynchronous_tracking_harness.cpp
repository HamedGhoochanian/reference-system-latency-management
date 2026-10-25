#include <cassert>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// This portable harness supplies the generated ROS message shape needed by
// provenance.hpp without including a ROS 2 or generated workspace header.
#define REFERENCE_SYSTEM__MSG_TYPES_HPP_
struct message_t
{
  static constexpr uint64_t STATS_CAPACITY = 128;
  std::string provenance_node;
  uint32_t provenance_sequence = 0;
  uint32_t provenance_channel = 0;
  uint32_t source_kind = 0;
  uint32_t source_sequence = 0;
  uint64_t source_timestamp_ns = 0;
};
namespace reference_interfaces
{
namespace msg
{
struct TransmissionStats
{
  static constexpr size_t NODE_NAME_LENGTH = 128;
  std::array<char, NODE_NAME_LENGTH> node_name{};
  uint64_t timestamp = 0;
  uint32_t sequence_number = 0;
  uint32_t dropped_samples = 0;
};
}  // namespace msg
}  // namespace reference_interfaces

#include "reference_system/provenance.hpp"

namespace
{

using reference_system_tracking::TrackingMessage;
using reference_system_tracking::TrackingSource;
using reference_system_tracking::TrackingWriter;
using reference_system_tracking::WriterMode;

class ControlledOutput
{
public:
  void write(const std::string & record)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (hold_first_ && !entered_) {
      entered_ = true;
      condition_.notify_all();
      condition_.wait(lock, [this] {return released_;});
    }
    records_.push_back(record);
    threads_.push_back(std::this_thread::get_id());
    condition_.notify_all();
  }

  void wait_until_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool entered = condition_.wait_for(
      lock, std::chrono::seconds(2), [this] {return entered_;});
    assert(entered && "consumer did not reach controlled write");
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

  std::vector<std::string> records() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return records_;
  }

  std::vector<std::thread::id> threads() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return threads_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool hold_first_ = true;
  bool entered_ = false;
  bool released_ = false;
  std::vector<std::string> records_;
  std::vector<std::thread::id> threads_;
};

bool contains(const std::string & record, const std::string & field)
{
  return record.find(field) != std::string::npos;
}

void test_unknown_mode_rejected_and_sync_is_default()
{
  unsetenv("REFERENCE_SYSTEM_TRACKING");
  unsetenv("REFERENCE_SYSTEM_TRACKING_WRITER");
  assert(reference_system_tracking::writer_mode_from_environment() == WriterMode::synchronous);

  setenv("REFERENCE_SYSTEM_TRACKING", "1", 1);
  setenv("REFERENCE_SYSTEM_TRACKING_WRITER", "invalid", 1);
  bool rejected = false;
  try {
    register_provenance_producer("InvalidModeNode");
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected && "unknown writer mode must fail producer startup");

  unsetenv("REFERENCE_SYSTEM_TRACKING");
  unsetenv("REFERENCE_SYSTEM_TRACKING_WRITER");
  assert(reference_system_tracking::writer_mode_from_environment() == WriterMode::synchronous);

  std::vector<std::string> records;
  const auto caller = std::this_thread::get_id();
  std::thread::id write_thread;
  TrackingWriter writer(
    "boot-sync", "instance-sync",
    [&records, &write_thread](const std::string & record) {
      write_thread = std::this_thread::get_id();
      records.push_back(record);
    });
  writer.write_source("SyncNode", 3, 1, 100);
  assert(records.size() == 1 && "default writer must remain synchronous");
  assert(write_thread == caller && "default serialization/write must stay on the caller");
}

void test_active_guard_limits_watermark_and_flush_order()
{
  ControlledOutput output;
  TrackingWriter writer(
    "boot-test", "instance-test",
    [&output](const std::string & record) {output.write(record);},
    reference_system_tracking::monotonic_now_ns, WriterMode::asynchronous, 4);

  {
    auto callback = writer.begin_callback("Node", 100);
    writer.write_source("Node", 7, 1, 100);
    output.wait_until_entered();
    writer.report_delivery("Node", 200);
    assert(output.records().empty() && "no line is visible before consumer release");
  }
  writer.report_delivery("Node", 300);
  assert(output.records().empty() && "delivery certificates stay hidden until writes resume");

  output.release();
  writer.shutdown();
  const auto records = output.records();
  assert(records.size() == 3);
  assert(contains(records[0], "\"event\":\"source\""));
  assert(contains(records[0], "\"t_ns\":100"));
  assert(contains(records[0], "\"record_seq\":0"));
  assert(contains(records[1], "\"event\":\"delivery\""));
  assert(contains(records[1], "\"through_ns\":99"));
  assert(contains(records[1], "\"record_seq\":1"));
  assert(contains(records[2], "\"through_ns\":300"));
  assert(contains(records[2], "\"record_seq\":2"));
}

void test_capacity_counts_delivery_and_shutdown_wakes_blocked_producer()
{
  ControlledOutput output;
  TrackingWriter writer(
    "boot-test", "instance-capacity",
    [&output](const std::string & record) {output.write(record);},
    reference_system_tracking::monotonic_now_ns, WriterMode::asynchronous, 4);

  writer.write_source("Node", 0, 1, 0);
  output.wait_until_entered();
  writer.write_source("Node", 1, 1, 1);
  writer.write_source("Node", 2, 1, 2);
  writer.report_delivery("Node", 50);

  std::mutex producer_mutex;
  std::condition_variable producer_condition;
  bool producer_started = false;
  bool producer_finished = false;
  std::string producer_error;
  std::thread blocked_producer([&] {
    {
      std::lock_guard<std::mutex> lock(producer_mutex);
      producer_started = true;
      producer_condition.notify_all();
    }
    try {
      writer.write_source("Node", 3, 1, 3);
    } catch (const std::exception & error) {
      producer_error = error.what();
    }
    {
      std::lock_guard<std::mutex> lock(producer_mutex);
      producer_finished = true;
      producer_condition.notify_all();
    }
  });
  {
    std::unique_lock<std::mutex> lock(producer_mutex);
    const bool started = producer_condition.wait_for(
      lock, std::chrono::seconds(2), [&] {return producer_started;});
    assert(started);
  }
  {
    std::unique_lock<std::mutex> lock(producer_mutex);
    const bool finished = producer_condition.wait_for(
      lock, std::chrono::milliseconds(80), [&] {return producer_finished;});
    assert(!finished && "fifth outstanding entry must block at capacity four, including delivery");
  }

  bool shutdown_failed = false;
  std::thread shutdown([&] {
    try {
      writer.shutdown();
    } catch (...) {
      shutdown_failed = true;
    }
  });
  {
    std::unique_lock<std::mutex> lock(producer_mutex);
    const bool finished = producer_condition.wait_for(
      lock, std::chrono::seconds(2), [&] {return producer_finished;});
    assert(finished && "shutdown must wake a producer blocked on a full queue");
  }
  assert(!producer_error.empty());
  output.release();
  blocked_producer.join();
  shutdown.join();
  assert(!shutdown_failed);

  const auto records = output.records();
  assert(records.size() == 4 && "all accepted entries must drain exactly once");
  for (size_t index = 0; index < records.size(); ++index) {
    assert(contains(records[index], "\"record_seq\":" + std::to_string(index)));
  }
  assert(contains(records[0], "\"t_ns\":0"));
  assert(contains(records[1], "\"t_ns\":1"));
  assert(contains(records[2], "\"t_ns\":2"));
  assert(contains(records[3], "\"event\":\"delivery\""));
}

void test_typed_events_copy_lineage_and_serialize_on_one_consumer()
{
  ControlledOutput output;
  const auto producer_thread = std::this_thread::get_id();
  TrackingWriter writer(
    "boot-test", "instance-typed",
    [&output](const std::string & record) {output.write(record);},
    reference_system_tracking::monotonic_now_ns, WriterMode::asynchronous, 4);

  writer.write_source("Telemetry", 3, 1, 100);
  output.wait_until_entered();
  std::vector<std::pair<uint32_t, TrackingMessage>> inputs{
    {0, {"FrontLidarDriver", 3, 0}}};
  TrackingSource source{1, 3, 100};
  writer.write_link("Telemetry", 7, 0, 200, inputs, source, false);
  inputs[0].second.name = "mutated-input";
  source.kind = 99;

  TrackingMessage sink_input{"VehicleInterface", 7, 2};
  TrackingSource sink_source{1, 3, 100};
  writer.write_sink("Telemetry", 2, 400, sink_input, sink_source);
  sink_input.name = "mutated-sink";
  sink_source.timestamp_ns = 999;
  assert(output.records().empty());

  output.release();
  writer.shutdown();
  const auto records = output.records();
  assert(records.size() == 3);
  assert(contains(records[0], "\"event\":\"source\""));
  assert(contains(records[0], "\"t_ns\":100"));
  assert(contains(records[0], "\"record_seq\":0"));
  assert(contains(records[1], "\"event\":\"link\""));
  assert(contains(records[1], "\"t_ns\":200"));
  assert(contains(records[1], "\"name\":\"FrontLidarDriver\""));
  assert(contains(records[1], "\"kind\":1,\"seq\":3,\"t_ns\":100"));
  assert(!contains(records[1], "mutated"));
  assert(contains(records[1], "\"record_seq\":1"));
  assert(contains(records[2], "\"event\":\"sink\""));
  assert(contains(records[2], "\"t_ns\":400"));
  assert(contains(records[2], "\"name\":\"VehicleInterface\""));
  assert(contains(records[2], "\"t_ns\":100"));
  assert(!contains(records[2], "mutated"));
  assert(contains(records[2], "\"record_seq\":2"));

  const auto threads = output.threads();
  assert(threads.size() == 3);
  assert(threads[0] != producer_thread);
  assert(threads[0] == threads[1] && threads[1] == threads[2]);
}

void test_write_failure_wakes_waiters_without_later_watermark()
{
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false;
  bool released = false;
  std::vector<std::string> visible_records;
  TrackingWriter writer(
    "boot-test", "instance-failure",
    [&](const std::string & record) {
      std::unique_lock<std::mutex> lock(mutex);
      if (!entered) {
        entered = true;
        condition.notify_all();
        condition.wait(lock, [&] {return released;});
        throw std::runtime_error("injected write failure");
      }
      visible_records.push_back(record);
    }, reference_system_tracking::monotonic_now_ns, WriterMode::asynchronous, 3);

  writer.write_source("Node", 0, 1, 0);
  {
    std::unique_lock<std::mutex> lock(mutex);
    const bool started = condition.wait_for(
      lock, std::chrono::seconds(2), [&] {return entered;});
    assert(started);
  }
  writer.write_source("Node", 1, 1, 1);
  writer.report_delivery("Node", 100);

  bool waiter_finished = false;
  std::string waiter_error;
  std::thread waiter([&] {
    try {
      writer.report_delivery("Node", 200);
    } catch (const std::exception & error) {
      waiter_error = error.what();
    }
    std::lock_guard<std::mutex> lock(mutex);
    waiter_finished = true;
    condition.notify_all();
  });
  {
    std::unique_lock<std::mutex> lock(mutex);
    const bool finished = condition.wait_for(
      lock, std::chrono::milliseconds(80), [&] {return waiter_finished;});
    assert(!finished && "delivery enqueue must wait while the bounded queue is full");
    released = true;
    condition.notify_all();
  }
  {
    std::unique_lock<std::mutex> lock(mutex);
    const bool finished = condition.wait_for(
      lock, std::chrono::seconds(2), [&] {return waiter_finished;});
    assert(finished && "write failure must wake a blocked producer");
  }
  waiter.join();
  assert(!waiter_error.empty());

  bool later_delivery_rejected = false;
  try {
    writer.report_delivery("Node", 300);
  } catch (const std::exception &) {
    later_delivery_rejected = true;
  }
  assert(later_delivery_rejected);
  bool shutdown_reports_failure = false;
  try {
    writer.shutdown();
  } catch (const std::exception &) {
    shutdown_reports_failure = true;
  }
  assert(shutdown_reports_failure);
  assert(visible_records.empty() && "a failed write cannot be followed by a false watermark");
}

void test_capacity_release_accepts_fifth_entry_in_fifo_order()
{
  ControlledOutput output;
  TrackingWriter writer(
    "boot-test", "instance-resume",
    [&output](const std::string & record) {output.write(record);},
    reference_system_tracking::monotonic_now_ns, WriterMode::asynchronous, 4);
  writer.write_source("Node", 0, 1, 100);
  output.wait_until_entered();
  writer.write_source("Node", 1, 1, 101);
  writer.report_delivery("Node", 101);
  writer.write_source("Node", 2, 1, 102);
  std::mutex mutex;
  std::condition_variable condition;
  bool started = false;
  bool finished = false;
  std::thread producer([&] {
    {
      std::lock_guard<std::mutex> lock(mutex);
      started = true;
      condition.notify_all();
    }
    writer.write_source("Node", 3, 1, 103);
    std::lock_guard<std::mutex> lock(mutex);
    finished = true;
    condition.notify_all();
  });
  {
    std::unique_lock<std::mutex> lock(mutex);
    assert(condition.wait_for(lock, std::chrono::seconds(2), [&] {return started;}));
    assert(!condition.wait_for(lock, std::chrono::milliseconds(80), [&] {return finished;}));
  }
  output.release();
  {
    std::unique_lock<std::mutex> lock(mutex);
    assert(condition.wait_for(lock, std::chrono::seconds(2), [&] {return finished;}));
  }
  producer.join();
  writer.shutdown();
  const auto records = output.records();
  assert(records.size() == 5);
  for (size_t index = 0; index < records.size(); ++index) {
    assert(contains(records[index], "\"record_seq\":" + std::to_string(index)));
  }
  assert(contains(records[0], "\"t_ns\":100"));
  assert(contains(records[1], "\"t_ns\":101"));
  assert(contains(records[2], "\"event\":\"delivery\""));
  assert(contains(records[3], "\"t_ns\":102"));
  assert(contains(records[4], "\"t_ns\":103"));
}

void test_async_guard_precedes_unenqueued_event()
{
  ControlledOutput output;
  TrackingWriter writer(
    "boot-test", "instance-guard",
    [&output](const std::string & record) {output.write(record);},
    reference_system_tracking::monotonic_now_ns, WriterMode::asynchronous, 4);
  {
    auto callback = writer.begin_callback("Node", 100);
    writer.report_delivery("Node", 200);
    output.wait_until_entered();
    writer.write_source("Node", 1, 1, 100);
  }
  writer.report_delivery("Node", 300);
  output.release();
  writer.shutdown();
  const auto records = output.records();
  assert(records.size() == 3);
  assert(contains(records[0], "\"through_ns\":99"));
  assert(contains(records[1], "\"t_ns\":100"));
  assert(contains(records[2], "\"through_ns\":300"));
}

class FailingFlushBuffer : public std::stringbuf
{
public:
  int sync() override {return -1;}
};

void test_production_stdout_flush_failure_is_explicit()
{
  setenv("REFERENCE_SYSTEM_TRACKING_WRITER", "async", 1);
  FailingFlushBuffer buffer;
  auto * previous_buffer = std::cout.rdbuf(&buffer);
  bool failed = false;
  {
    reference_system_tracking::Runtime instance;
    try {
      instance.writer().write_source("Node", 0, 1, 100);
      instance.writer().report_delivery("Node", 200);
      instance.writer().shutdown();
    } catch (const std::runtime_error &) {
      failed = true;
    }
  }
  std::cout.rdbuf(previous_buffer);
  std::cout.clear();
  unsetenv("REFERENCE_SYSTEM_TRACKING_WRITER");
  assert(failed && "production stdout flush failure must fail the async writer explicitly");
  assert(!contains(buffer.str(), "\"event\":\"delivery\""));
}

void test_async_production_stdout_and_ordinary_logging()
{
  setenv("REFERENCE_SYSTEM_TRACKING_WRITER", "async", 1);
  std::ostringstream output;
  auto * previous_buffer = std::cout.rdbuf(output.rdbuf());
  {
    reference_system_tracking::Runtime instance;
    std::thread producer([&] {
      for (uint32_t index = 0; index < 2000; ++index) {
        instance.writer().write_source("Node", index, 1, index);
      }
    });
    std::thread logger([] {
      for (uint32_t index = 0; index < 2000; ++index) {
        std::lock_guard<std::mutex> lock(reference_system_cout_mutex());
        std::cout << "ordinary log " << index << std::endl;
      }
    });
    producer.join();
    logger.join();
    instance.writer().shutdown();
  }
  std::cout.rdbuf(previous_buffer);
  unsetenv("REFERENCE_SYSTEM_TRACKING_WRITER");
  std::istringstream lines(output.str());
  std::string line;
  uint32_t events = 0;
  uint32_t logs = 0;
  while (std::getline(lines, line)) {
    if (line.rfind("ordinary log ", 0) == 0) {
      assert(line == "ordinary log " + std::to_string(logs++));
    } else {
      assert(line.rfind("{\"schema\":2,\"event\":\"source\"", 0) == 0);
      assert(contains(line, "\"seq\":" + std::to_string(events) + ","));
      assert(contains(line, "\"record_seq\":" + std::to_string(events++) + "}"));
      assert(!contains(line, "ordinary log"));
    }
  }
  assert(events == 2000 && logs == 2000);
}

void test_typed_schema_equivalence_including_timer_and_null_lineage()
{
  unsetenv("REFERENCE_SYSTEM_TRACKING");
  std::ostringstream legacy;
  auto * previous_buffer = std::cout.rdbuf(legacy.rdbuf());
  const provenance_source_t source{1, 3, 100};
  const std::vector<std::pair<uint32_t, provenance_message_t>> inputs{
    {0, {"FrontLidarDriver", 3, 0}}, {2, {"RearLidarDriver", 4, 1}}};
  emit_provenance_source("Source", 3, 1, 100);
  emit_provenance_link("Link", 7, 2, 200, inputs, source, false);
  emit_provenance_link("Link", 8, 0, 250, {}, {}, true);
  emit_provenance_sink("Sink", 1, 300, {"Link", 7, 2}, source);
  emit_provenance_sink("Sink", 2, 400, {"Link", 8, 0}, {});
  std::cout.rdbuf(previous_buffer);

  std::vector<std::string> actual;
  TrackingWriter writer(
    "boot-test", "instance-equality",
    [&actual](const std::string & record) {actual.push_back(record);},
    reference_system_tracking::monotonic_now_ns, WriterMode::asynchronous, 4);
  writer.write_source("Source", 3, 1, 100);
  writer.write_link("Link", 7, 2, 200, inputs, source, false);
  writer.write_link("Link", 8, 0, 250,
    std::vector<std::pair<uint32_t, TrackingMessage>>{}, TrackingSource{}, true);
  writer.write_sink("Sink", 1, 300, provenance_message_t{"Link", 7, 2}, source);
  writer.write_sink("Sink", 2, 400, TrackingMessage{"Link", 8, 0}, TrackingSource{});
  writer.shutdown();
  const std::vector<std::string> producers{"Source", "Link", "Link", "Sink", "Sink"};
  const std::vector<uint64_t> sequences{0, 0, 1, 0, 1};
  std::istringstream lines(legacy.str());
  std::string line;
  assert(actual.size() == producers.size());
  for (size_t index = 0; index < actual.size(); ++index) {
    assert(std::getline(lines, line));
    assert(actual[index] == reference_system_tracking::with_identity(
        line, producers[index], "instance-equality", "boot-test", sequences[index]));
  }
  assert(!std::getline(lines, line));
}

void test_sync_schema_one_and_async_provenance_emitters()
{
  unsetenv("REFERENCE_SYSTEM_TRACKING");
  unsetenv("REFERENCE_SYSTEM_TRACKING_WRITER");
  std::ostringstream legacy_output;
  auto * previous_buffer = std::cout.rdbuf(legacy_output.rdbuf());
  emit_provenance_source("FrontLidarDriver", 3, 1, 100);
  const std::vector<std::pair<uint32_t, provenance_message_t>> inputs{
    {0, {"FrontLidarDriver", 3, 0}}};
  const provenance_source_t source{1, 3, 100};
  emit_provenance_link("PointsTransformerFront", 7, 0, 200, inputs, source, false);
  emit_provenance_sink(
    "VehicleDBWSystem", 2, 400, {"VehicleInterface", 7, 0}, source);
  std::cout.rdbuf(previous_buffer);
  std::istringstream legacy_lines(legacy_output.str());
  std::string line;
  assert(std::getline(legacy_lines, line));
  assert(line == "{\"schema\":1,\"event\":\"source\",\"t_ns\":100,\"node\":\"FrontLidarDriver\",\"seq\":3,\"kind\":1}");
  assert(std::getline(legacy_lines, line));
  assert(line == "{\"schema\":1,\"event\":\"link\",\"t_ns\":200,\"node\":\"PointsTransformerFront\",\"msg\":{\"name\":\"PointsTransformerFront\",\"seq\":7,\"channel\":0},\"inputs\":[{\"index\":0,\"msg\":{\"name\":\"FrontLidarDriver\",\"seq\":3,\"channel\":0}}],\"source\":{\"kind\":1,\"seq\":3,\"t_ns\":100},\"timer\":false}");
  assert(std::getline(legacy_lines, line));
  assert(line == "{\"schema\":1,\"event\":\"sink\",\"t_ns\":400,\"node\":\"VehicleDBWSystem\",\"msg\":{\"name\":\"VehicleDBWSystem\",\"seq\":2,\"channel\":0},\"input\":{\"name\":\"VehicleInterface\",\"seq\":7,\"channel\":0},\"source\":{\"kind\":1,\"seq\":3,\"t_ns\":100},\"sensor_started\":true}");
  assert(!std::getline(legacy_lines, line));

  setenv("REFERENCE_SYSTEM_TRACKING_WRITER", "async", 1);
  std::ostringstream untracked_async_output;
  previous_buffer = std::cout.rdbuf(untracked_async_output.rdbuf());
  emit_provenance_source("FrontLidarDriver", 3, 1, 100);
  emit_provenance_link("PointsTransformerFront", 7, 0, 200, inputs, source, false);
  emit_provenance_sink(
    "VehicleDBWSystem", 2, 400, {"VehicleInterface", 7, 0}, source);
  std::cout.rdbuf(previous_buffer);
  assert(untracked_async_output.str() == legacy_output.str());

  setenv("REFERENCE_SYSTEM_TRACKING", "1", 1);
  setenv("REFERENCE_SYSTEM_TRACKING_WRITER", "async", 1);
  setenv("REFERENCE_SYSTEM_DELIVERY_INTERVAL_MS", "60000", 1);
  std::ostringstream async_output;
  previous_buffer = std::cout.rdbuf(async_output.rdbuf());
  emit_provenance_source("Telemetry", 3, 1, 100);
  const std::vector<std::pair<uint32_t, provenance_message_t>> async_inputs{
    {0, {"FrontLidarDriver", 3, 0}}};
  emit_provenance_link("Telemetry", 7, 1, 200, async_inputs, source, false);
  emit_provenance_sink("Telemetry", 2, 400, {"VehicleInterface", 7, 2}, source);
  reference_system_tracking::runtime().writer().shutdown();
  std::cout.rdbuf(previous_buffer);
  unsetenv("REFERENCE_SYSTEM_TRACKING");
  unsetenv("REFERENCE_SYSTEM_TRACKING_WRITER");
  unsetenv("REFERENCE_SYSTEM_DELIVERY_INTERVAL_MS");

  std::istringstream async_lines(async_output.str());
  std::vector<std::string> records;
  while (std::getline(async_lines, line)) {records.push_back(line);}
  assert(records.size() == 3);
  assert(contains(records[0], "\"schema\":2,\"event\":\"source\""));
  assert(contains(records[0], "\"t_ns\":100"));
  assert(contains(records[0], "\"record_seq\":0"));
  assert(contains(records[1], "\"schema\":2,\"event\":\"link\""));
  assert(contains(records[1], "\"name\":\"FrontLidarDriver\""));
  assert(contains(records[1], "\"record_seq\":1"));
  assert(contains(records[2], "\"schema\":2,\"event\":\"sink\""));
  assert(contains(records[2], "\"name\":\"VehicleInterface\""));
  assert(contains(records[2], "\"record_seq\":2"));
}

}  // namespace

int main()
{
  test_unknown_mode_rejected_and_sync_is_default();
  test_active_guard_limits_watermark_and_flush_order();
  test_capacity_counts_delivery_and_shutdown_wakes_blocked_producer();
  test_typed_events_copy_lineage_and_serialize_on_one_consumer();
  test_write_failure_wakes_waiters_without_later_watermark();
  test_capacity_release_accepts_fifth_entry_in_fifo_order();
  test_async_guard_precedes_unenqueued_event();
  test_production_stdout_flush_failure_is_explicit();
  test_async_production_stdout_and_ordinary_logging();
  test_typed_schema_equivalence_including_timer_and_null_lineage();
  test_sync_schema_one_and_async_provenance_emitters();
  return 0;
}
