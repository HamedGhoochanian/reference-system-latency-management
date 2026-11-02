// SPDX-License-Identifier: Apache-2.0
// Focused transport tests run without ROS, Docker, or workload launches.
#include "reference_system/source_schedule.hpp"
#include "reference_system/tracking.hpp"
#ifdef NDEBUG
#undef NDEBUG // Keep transport assertions active in Release container test builds.
#endif
#include <cassert>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;

namespace {
struct TempDir {
  fs::path path = fs::temp_directory_path() /
    ("source-schedule-" + std::to_string(getpid()));

  TempDir() {assert(fs::create_directory(path));}
  ~TempDir() {fs::remove_all(path);}
};

void write(const fs::path & path, const std::string & text)
{
  std::ofstream file(path);
  file << text;
  assert(file.good());
}

std::string schedule_text(const std::string & source, const std::string & instance,
  int64_t duration, const std::string & count, const std::string & rows,
  const std::string & session = "session", const std::string & boot = "boot",
  const std::string & hash = "hash")
{
  return "LAME_SOURCE_SCHEDULE_V1 " + session + " " + boot + " " + hash + " " + source + " " +
    instance + " " + std::to_string(duration) + "\n" + count + "\n" + rows;
}

std::string row(const std::string & id, int64_t offset, int64_t period)
{
  return id + " " + std::to_string(offset) + " " + std::to_string(period) + "\n";
}

bool rejects(const std::function<void()> & action)
{
  try {action();} catch (const std::exception &) {return true;}
  return false;
}

void test_three_phase_regression(const fs::path & dir)
{
  const auto prepared = dir / "legacy.schedule";
  const auto start = dir / "start";
  const std::string text = schedule_text("Source", "instance", 3000, "3",
    row("A1", 0, 100) + row("B", 1000, 200) + row("A2", 2000, 100));
  write(prepared, text);

  assert(rejects([&] {nodes::SourceSchedule s(prepared, "Other", "boot", "instance", 100);}));
  assert(rejects([&] {nodes::SourceSchedule s(prepared, "Source", "old-boot", "instance", 100);}));
  assert(rejects([&] {nodes::SourceSchedule s(prepared, "Source", "boot", "old-instance", 100);}));

  nodes::SourceSchedule schedule(prepared, "Source", "boot", "instance", 100);
  assert(schedule.due(5000).empty()); // A prepared schedule is not armed.
  write(start, "LAME_SOURCE_START_V1 session boot hash 5000\n");
  assert(schedule.due(4999).empty());
  assert((schedule.due(5000) == std::vector<size_t>{0}));
  schedule.applied(0);
  assert(schedule.due(5999).empty());
  assert((schedule.due(6000) == std::vector<size_t>{1}));
  schedule.applied(1);
  assert((schedule.due(7000) == std::vector<size_t>{2}));
  schedule.applied(2);
  assert(schedule.due(100000).empty()); // The final phase does not periodically restart.

  nodes::SourceSchedule missed(prepared, "Source", "boot", "instance", 100);
  assert((missed.due(7500) == std::vector<size_t>{0, 1, 2}));
  assert(missed.requested(2) == 7000);
  std::ostringstream record;
  missed.record(record, 1, "skipped", "Source", "boot", "instance", 100, 100, 7500);
  assert(record.str().find("\"requested_ns\":6000,\"applied_ns\":7500") != std::string::npos);
  assert(record.str().find("\"status\":\"skipped\"") != std::string::npos);
  alignas(64) uint8_t buffer[reference_system_telemetry::header_bytes + 1024]{};
  reference_system_telemetry::Identity id{"run", "Source", "boot", "instance"};
  assert(reference_system_telemetry::initialize(buffer, sizeof(buffer), id));
  reference_system_telemetry::MemoryWriter memory(buffer, sizeof(buffer), id);
  reference_system_tracking::TrackingWriter tracking("boot", "instance", [](const std::string &) {
      assert(false && "memory source receipt wrote a stream");
    }, reference_system_tracking::monotonic_now_ns,
    reference_system_tracking::WriterMode::memory,
    reference_system_tracking::TrackingWriter::kDefaultQueueCapacity, &memory);
  missed.record_memory(tracking, 1, "skipped", "Source", 100, 100, 7500);
  assert(reference_system_telemetry::committed(buffer) > 0);
  assert(buffer[reference_system_telemetry::header_bytes + 4] ==
    static_cast<uint8_t>(reference_system_telemetry::Kind::source_application));

  write(start, "LAME_SOURCE_START_V1 stale boot hash 5000\n");
  nodes::SourceSchedule stale(prepared, "Source", "boot", "instance", 100);
  assert(rejects([&] {stale.due(6000);}));
}

std::string make_36_phase_trace()
{
  std::string rows;
  for (int i = 0; i < 36; ++i) {
    const int64_t period = (i == 0 ? 100000000 : ((i % 3) + 1) * 100000000LL);
    rows += row("phase-" + std::to_string(i + 1), i * 20000000000LL, period);
  }
  return schedule_text("Source", "instance", 720000000000LL, "36", rows);
}

void test_36_phases_boundaries_and_final_hold(const fs::path & dir)
{
  const auto prepared = dir / "long.schedule";
  const auto start = dir / "start";
  write(prepared, make_36_phase_trace());

  std::unique_ptr<nodes::SourceSchedule> schedule;
  try {
    schedule = std::make_unique<nodes::SourceSchedule>(prepared, "Source", "boot", "instance", 100000000);
  } catch (const std::exception &) {
    assert(false && "a complete 36-phase trace must be accepted");
    return;
  }
  assert(schedule->phase(35).period != schedule->phase(0).period); // Final state need not return to R0.

  const int64_t epoch = 1000000000000LL;
  assert(schedule->due(epoch + 900000000000LL).empty()); // No start record: prepared, not armed.
  write(start, "LAME_SOURCE_START_V1 session boot hash " + std::to_string(epoch) + "\n");
  assert(schedule->due(epoch - 1).empty());
  assert((schedule->due(epoch) == std::vector<size_t>{0}));
  schedule->applied(0);
  for (size_t i = 1; i < 36; ++i) {
    const int64_t boundary = epoch + static_cast<int64_t>(i) * 20000000000LL;
    assert(schedule->due(boundary - 1).empty());
    assert((schedule->due(boundary) == std::vector<size_t>{i}));
    assert(schedule->requested(i) == boundary);
    schedule->applied(i);
  }
  assert(schedule->due(epoch + 720000000000LL).empty());
  assert(schedule->due(epoch + 900000000000LL).empty()); // Hold final state; never repeat or restart.
}

void test_count_limits_and_single_phase(const fs::path & dir)
{
  const auto prepared = dir / "count.schedule";
  auto bad = [&](const std::string & text) {
    write(prepared, text);
    assert(rejects([&] {nodes::SourceSchedule s(prepared, "Source", "boot", "instance", 100);}));
  };
  const std::string one = row("only", 0, 100);
  bad(schedule_text("Source", "instance", 1000, "0", one));
  bad(schedule_text("Source", "instance", 1000, "-1", one));
  bad(schedule_text("Source", "instance", 1000, "65", one));
  bad(schedule_text("Source", "instance", 1000, "18446744073709551615", one));
  bad(schedule_text("Source", "instance", 1000, "18446744073709551616", one));

  std::string rows64;
  for (int i = 0; i < 64; ++i) {rows64 += row("p" + std::to_string(i), i, 100 + i);}
  write(prepared, schedule_text("Source", "instance", 1000, "64", rows64));
  nodes::SourceSchedule max_count(prepared, "Source", "boot", "instance", 100);
  assert(max_count.phase(63).offset == 63);

  write(prepared, schedule_text("Source", "instance", 1000, "1", one));
  nodes::SourceSchedule single(prepared, "Source", "boot", "instance", 100);
  write(dir / "start", "LAME_SOURCE_START_V1 session boot hash 9000\n");
  assert((single.due(9000) == std::vector<size_t>{0}));
  single.applied(0);
  assert(single.due(100000).empty());

  const auto max = std::numeric_limits<int64_t>::max();
  write(prepared, schedule_text("Source", "instance", max, "1", row("only", 0, max)));
  nodes::SourceSchedule largest(prepared, "Source", "boot", "instance", max);
  assert(largest.phase(0).period == max);
}

void test_malformed_schedule_records(const fs::path & dir)
{
  const auto prepared = dir / "malformed.schedule";
  const std::string valid_rows = row("A", 0, 100) + row("B", 50, 200);
  auto bad = [&](const std::string & text, const std::string & source = "Source",
      const std::string & boot = "boot", const std::string & instance = "instance",
      int64_t initial = 100) {
    write(prepared, text);
    assert(rejects([&] {nodes::SourceSchedule s(prepared, source, boot, instance, initial);}));
  };

  bad("garbage");
  bad(schedule_text("Other", "instance", 100, "2", valid_rows));
  bad(schedule_text("bad/source", "instance", 100, "2", valid_rows), "bad/source");
  bad(schedule_text("Source", "instance", 100, "2", valid_rows, "bad/session"));
  bad(schedule_text("Source", "instance", 100, "2", valid_rows, "session", "bad boot"));
  bad(schedule_text("Source", "instance", 100, "2", valid_rows, "session", "boot", "bad/hash"));
  bad(schedule_text("Source", "other", 100, "2", valid_rows), "Source", "boot", "instance");
  bad(schedule_text("Source", "instance", 100, "2", valid_rows), "Source", "other-boot");
  bad(schedule_text("Source", "instance", 100, "2", valid_rows), "Source", "boot", "other-instance");

  bad(schedule_text("Source", "instance", 0, "2", valid_rows));
  bad(schedule_text("Source", "instance", -1, "2", valid_rows));
  bad("LAME_SOURCE_SCHEDULE_V1 session boot hash Source instance 9223372036854775808\n2\n" + valid_rows);
  bad(schedule_text("Source", "instance", 100, "2", row("A", 1, 100) + row("B", 50, 200)));
  bad(schedule_text("Source", "instance", 100, "2", row("A", 0, 100) + row("B", 0, 200)));
  bad(schedule_text("Source", "instance", 100, "3", row("A", 0, 100) + row("B", 25, 200) +
    row("C", 20, 300)));
  bad(schedule_text("Source", "instance", 100, "2", row("A", 0, 100) + row("B", -1, 200)));
  bad(schedule_text("Source", "instance", 100, "2", row("A", 0, 100) + row("B", 100, 200)));
  bad(schedule_text("Source", "instance", 100, "2", row("A", 0, 100) + row("B", 50, 200)),
    "Source", "boot", "instance", 101);
  bad("LAME_SOURCE_SCHEDULE_V1 session boot hash Source instance 100x\n2\n" + valid_rows);
  bad(schedule_text("Source", "instance", 100, "2", row("A", 0, 0) + row("B", 50, 200)));
  bad(schedule_text("Source", "instance", 100, "2", row("A", 0, -1) + row("B", 50, 200)));
  bad(schedule_text("Source", "instance", 100, "2", row("A", 0, 100) + row("B", 50, 0)));
  bad(schedule_text("Source", "instance", 100, "2", row("A", 0, 100) + row("A", 50, 200)));
  bad(schedule_text("Source", "instance", 100, "2", row("bad/id", 0, 100) + row("B", 50, 200)));
  bad(schedule_text("Source", "instance", 100, "2", row("A", 0, 100)));
  bad("LAME_SOURCE_SCHEDULE_V1 session boot hash Source instance 100\n");
  bad(schedule_text("Source", "instance", 100, "2", valid_rows + "extra\n"));
  bad("LAME_SOURCE_SCHEDULE_V1 session boot hash Source instance 100x\n2\n" + valid_rows);
  bad("LAME_SOURCE_SCHEDULE_V1 session boot hash Source instance 100\n2x\n" + valid_rows);
  bad("LAME_SOURCE_SCHEDULE_V1 session boot hash Source instance 100\n2\nA 0 9223372036854775808\n" +
    row("B", 50, 200));
  bad("LAME_SOURCE_SCHEDULE_V1 session boot hash Source instance 100\n2\nA x 100\n" + row("B", 50, 200));
  bad("LAME_SOURCE_SCHEDULE_V1 session boot hash Source instance 100\n2\nA 0 x\n" + row("B", 50, 200));
}

void test_start_identity_and_epoch_overflow(const fs::path & dir)
{
  const auto prepared = dir / "epoch.schedule";
  const auto start = dir / "start";
  write(prepared, schedule_text("Source", "instance", 100, "1", row("only", 0, 100)));
  const auto expect_bad_start = [&](const std::string & contents) {
    write(start, contents);
    nodes::SourceSchedule schedule(prepared, "Source", "boot", "instance", 100);
    assert(rejects([&] {schedule.due(1);}));
  };
  expect_bad_start("wrong session boot hash 1\n");
  expect_bad_start("LAME_SOURCE_START_V1 wrong-session boot hash 1\n");
  expect_bad_start("LAME_SOURCE_START_V1 session wrong-boot hash 1\n");
  expect_bad_start("LAME_SOURCE_START_V1 session boot wrong-hash 1\n");
  expect_bad_start("LAME_SOURCE_START_V1 session boot hash 0\n");
  expect_bad_start("LAME_SOURCE_START_V1 session boot hash -1\n");
  expect_bad_start("LAME_SOURCE_START_V1 session boot hash 1 extra\n");
  expect_bad_start("LAME_SOURCE_START_V1 session boot hash 9223372036854775808\n");
  expect_bad_start("LAME_SOURCE_START_V1 session boot hash 1x\n");

  const auto max = std::numeric_limits<int64_t>::max();
  write(start, "LAME_SOURCE_START_V1 session boot hash " + std::to_string(max - 100) + "\n");
  nodes::SourceSchedule boundary(prepared, "Source", "boot", "instance", 100);
  assert((boundary.due(max - 100) == std::vector<size_t>{0})); // duration end exactly representable.

  write(start, "LAME_SOURCE_START_V1 session boot hash " + std::to_string(max - 99) + "\n");
  nodes::SourceSchedule overflow(prepared, "Source", "boot", "instance", 100);
  assert(rejects([&] {overflow.due(max - 99);}));
}

void test_delayed_receipts_and_instance_skew(const fs::path & dir)
{
  const int64_t epoch = 1000;
  const std::string rows = row("phase-0", 0, 100) + row("phase-1", 10, 100) +
    row("phase-2", 20, 200) + row("phase-3", 30, 300);
  const auto make_instance = [&](const std::string & source, const std::string & instance) {
    const auto path = dir / instance;
    assert(fs::create_directory(path));
    const auto prepared = path / "source.schedule";
    write(prepared, schedule_text(source, instance, 40, "4", rows));
    write(path / "start", "LAME_SOURCE_START_V1 session boot hash " + std::to_string(epoch) + "\n");
    return prepared;
  };
  auto first_path = make_instance("SourceA", "instance-a");
  auto second_path = make_instance("SourceB", "instance-b");
  nodes::SourceSchedule first(first_path, "SourceA", "boot", "instance-a", 100);
  nodes::SourceSchedule second(second_path, "SourceB", "boot", "instance-b", 100);

  for (auto * schedule : {&first, &second}) {
    assert((schedule->due(epoch) == std::vector<size_t>{0}));
    schedule->applied(0);
  }

  const int64_t first_applied = epoch + 13;
  const int64_t second_applied = epoch + 15;
  const auto first_due = first.due(first_applied);
  const auto second_due = second.due(second_applied);
  assert((first_due == std::vector<size_t>{1}));
  assert((second_due == std::vector<size_t>{1}));
  std::ostringstream first_receipts;
  std::ostringstream second_receipts;
  first.record(first_receipts, 1, "applied", "SourceA", "boot", "instance-a", 100, 100, first_applied);
  second.record(second_receipts, 1, "applied", "SourceB", "boot", "instance-b", 100, 100, second_applied);
  first.applied(1);
  second.applied(1);
  assert(first_receipts.str().find("\"requested_ns\":1010,\"applied_ns\":1013") != std::string::npos);
  assert(second_receipts.str().find("\"requested_ns\":1010,\"applied_ns\":1015") != std::string::npos);
  assert(first_applied != second_applied); // Per-instance application skew remains visible.
  assert(first_receipts.str().find("\"old_period_ns\":100,\"new_period_ns\":100") != std::string::npos);

  const auto late_path = make_instance("SourceLate", "instance-late");
  nodes::SourceSchedule late(late_path, "SourceLate", "boot", "instance-late", 100);
  assert((late.due(epoch) == std::vector<size_t>{0}));
  late.applied(0);
  const int64_t callback_time = epoch + 25;
  const auto due = late.due(callback_time);
  assert((due == std::vector<size_t>{1, 2}));
  std::ostringstream delayed_receipts;
  delayed_receipts << "[";
  late.record(delayed_receipts, due[0], "skipped", "SourceLate", "boot", "instance-late",
    100, 100, callback_time);
  delayed_receipts << "\n";
  late.record(delayed_receipts, due.back(), "applied", "SourceLate", "boot", "instance-late",
    100, 200, callback_time);
  late.applied(due.back());
  assert(late.due(callback_time).empty());
  assert(delayed_receipts.str().find("\"requested_ns\":1010,\"applied_ns\":1025") != std::string::npos);
  assert(delayed_receipts.str().find("\"status\":\"skipped\"") != std::string::npos);
  assert(delayed_receipts.str().find("\"requested_ns\":1020,\"applied_ns\":1025") != std::string::npos);
  assert(delayed_receipts.str().find("\"old_period_ns\":100,\"new_period_ns\":200") != std::string::npos);
  assert((late.due(epoch + 30) == std::vector<size_t>{3}));
  std::ostringstream next_receipt;
  late.record(next_receipt, 3, "applied", "SourceLate", "boot", "instance-late", 200, 300, epoch + 40);
  assert(next_receipt.str().find("\"requested_ns\":1030,\"applied_ns\":1040") != std::string::npos);
  assert(next_receipt.str().find("\"old_period_ns\":200,\"new_period_ns\":300") != std::string::npos);
}
} // namespace

int main()
{
  TempDir temp;
  const auto legacy = temp.path / "legacy";
  const auto long_trace = temp.path / "long";
  const auto counts = temp.path / "counts";
  const auto malformed = temp.path / "malformed";
  const auto epochs = temp.path / "epochs";
  const auto receipts = temp.path / "receipts";
  for (const auto & dir : {legacy, long_trace, counts, malformed, epochs, receipts}) {
    assert(fs::create_directory(dir));
  }
  test_three_phase_regression(legacy);
  test_36_phases_boundaries_and_final_hold(long_trace);
  test_count_limits_and_single_phase(counts);
  test_malformed_schedule_records(malformed);
  test_start_identity_and_epoch_overflow(epochs);
  test_delayed_receipts_and_instance_skew(receipts);
}
