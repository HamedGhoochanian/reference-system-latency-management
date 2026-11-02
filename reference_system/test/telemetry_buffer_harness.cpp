#define REFERENCE_SYSTEM_TELEMETRY_TEST_HOOK
#include "reference_system/telemetry_buffer.hpp"

#include <cassert>
#include <array>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace reference_system_telemetry;

int main(int argc, char ** argv)
{
  constexpr size_t record_size = 40; // delivery: 32-byte record header + 8-byte payload
  alignas(64) std::array<uint64_t, (header_bytes + record_size + 63) / 8> storage{};
  auto * bytes = reinterpret_cast<uint8_t *>(storage.data());
  const Identity identity{"run-A", "FrontLidarDriver", "boot-A", "instance-A"};
  if (argc == 3 && std::string(argv[1]) == "--append") {
    MappedBuffer shared(argv[2], identity);
    RecordView event;
    event.kind = Kind::source;
    event.sequence = 0;
    event.timestamp_ns = 100;
    event.message_sequence = 0;
    event.source_kind = 1;
    assert(shared.writer().append(event) == AppendResult::written);
    RecordView report;
    report.kind = Kind::delivery;
    report.sequence = 1;
    report.timestamp_ns = 201;
    report.through_ns = 200;
    assert(shared.writer().append(report) == AppendResult::written);
    assert(shared.writer().complete());
    assert(shared.writer().append(report) == AppendResult::failed);
    return 0;
  }
  assert(initialize(bytes, header_bytes + record_size, identity));
  MemoryWriter writer(bytes, header_bytes + record_size, identity);
  RecordView delivery{};
  delivery.kind = Kind::delivery;
  delivery.sequence = UINT64_MAX - 1;
  delivery.timestamp_ns = INT64_MAX;
  delivery.through_ns = INT64_MAX - 1;
  assert(writer.append(delivery) == AppendResult::written);
  assert(committed(bytes) == record_size);
  assert(writer.append(delivery) == AppendResult::full);
  assert(failure(bytes) == 1);
  assert(committed(bytes) == record_size);
  assert(writer.append(delivery) == AppendResult::failed);

  alignas(64) std::array<uint64_t, (header_bytes + record_size + 63) / 8> short_storage{};
  auto * short_bytes = reinterpret_cast<uint8_t *>(short_storage.data());
  assert(initialize(short_bytes, header_bytes + record_size - 1, identity));
  MemoryWriter short_writer(short_bytes, header_bytes + record_size - 1, identity);
  assert(short_writer.append(delivery) == AppendResult::full);
  assert(committed(short_bytes) == 0);
  assert(failure(short_bytes) == 1);

  alignas(64) std::array<uint64_t, (header_bytes + 1024 + 63) / 8> other_storage{};
  auto * other = reinterpret_cast<uint8_t *>(other_storage.data());
  assert(initialize(other, header_bytes + 1024, identity));
  MemoryWriter other_writer(other, header_bytes + 1024, identity);
  RecordView source{};
  source.kind = Kind::source;
  source.sequence = 0;
  source.timestamp_ns = INT64_MAX - 2;
  source.message_sequence = 7;
  source.source_kind = 1;
  assert(other_writer.append(source) == AppendResult::written);
  RecordView empty{};
  assert(other_writer.append(empty) == AppendResult::failed);
  assert(failure(other) == 2);

  alignas(64) std::array<uint64_t, (header_bytes + 1024 + 63) / 8> oversized_storage{};
  auto * oversized = reinterpret_cast<uint8_t *>(oversized_storage.data());
  assert(initialize(oversized, header_bytes + 1024, identity));
  MemoryWriter oversized_writer(oversized, header_bytes + 1024, identity);
  std::string long_name(256, 'x');
  RecordView link{};
  link.kind = Kind::link;
  link.name = long_name;
  assert(oversized_writer.append(link) == AppendResult::failed);
  assert(committed(oversized) == 0);
  assert(failure(oversized) == 2);

  // One golden mapping contains all kinds, including a null source and two inputs.
  alignas(64) std::array<uint64_t, (header_bytes + 4096 + 63) / 8> golden_storage{};
  auto * golden = reinterpret_cast<uint8_t *>(golden_storage.data());
  assert(initialize(golden, header_bytes + 4096, identity));
  MemoryWriter golden_writer(golden, header_bytes + 4096, identity);
  assert(golden_writer.append(source) == AppendResult::written);
  InputView inputs[] = {{0, "FrontLidarDriver", 3, 1}, {1, "RearLidarDriver", 4, 2}};
  link.name = "PointCloudFusion";
  link.sequence = 1;
  link.timestamp_ns = 500;
  link.message_sequence = 9;
  link.channel = 2;
  link.inputs = inputs;
  link.input_count = 2;
  link.timer = true;
  link.source_kind = 1;
  link.source_sequence = 3;
  link.source_timestamp_ns = 100;
  assert(golden_writer.append(link) == AppendResult::written);
  RecordView sink{};
  sink.kind = Kind::sink;
  sink.sequence = 2;
  sink.timestamp_ns = 600;
  sink.message_sequence = 8;
  sink.input = {0, "VehicleInterface", 4, 3};
  assert(golden_writer.append(sink) == AppendResult::written);
  delivery.sequence = 3;
  assert(golden_writer.append(delivery) == AppendResult::written);
  RecordView timing{};
  timing.kind = Kind::timing;
  timing.name = "Sensor";
  timing.aux_name = "FrontLidarDriver";
  timing.value_ns = 123;
  assert(golden_writer.append(timing) == AppendResult::written);
  RecordView application{};
  application.kind = Kind::source_application;
  application.name = "phase-1";
  application.aux_name = "hash-A";
  application.session = "session-A";
  application.status = 1;
  application.requested_ns = 700;
  application.old_period_ns = 100;
  application.new_period_ns = 200;
  assert(golden_writer.append(application) == AppendResult::written);
  RecordView diagnostic{};
  diagnostic.kind = Kind::diagnostic;
  diagnostic.name = "completion";
  diagnostic.aux_name = "FrontLidarDriver";
  diagnostic.sink_name = "VehicleDBWSystem";
  diagnostic.message_sequence = 4;
  diagnostic.source_sequence = 3;
  diagnostic.source_timestamp_ns = 100;
  diagnostic.timestamp_ns = 600;
  diagnostic.status = 2;
  diagnostic.drop_count = 7;
  diagnostic.instrumentation_errors = 11;
  diagnostic.metadata_overflows = 13;
  RootView roots[] = {{"FrontLidarDriver", 3, 100}, {"RearLidarDriver", 4, 101}};
  std::string_view lineage[] = {"FrontLidarDriver", "VehicleDBWSystem"};
  diagnostic.roots = roots;
  diagnostic.root_count = 2;
  diagnostic.lineage = lineage;
  diagnostic.lineage_count = 2;
  diagnostic.value_ns = 5;
  assert(golden_writer.append(diagnostic) == AppendResult::written);
  assert(failure(golden) == 0);
  // MAP_SHARED and fork exercise publication between processes, not just threads.
  void * mapped = mmap(nullptr, header_bytes + 128, PROT_READ | PROT_WRITE,
      MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  assert(mapped != MAP_FAILED);
  auto * shared = static_cast<uint8_t *>(mapped);
  assert(initialize(shared, header_bytes + 128, identity));
  const pid_t child = fork();
  assert(child >= 0);
  if (child == 0) {
    MemoryWriter process_writer(shared, header_bytes + 128, identity);
    _exit(process_writer.append(source) == AppendResult::written ? 0 : 1);
  }
  for (size_t attempts = 0; !committed(shared) && attempts < 100000000; ++attempts) {
    // acquire load above must observe the complete child record when it changes.
  }
  int child_status = 0;
  assert(waitpid(child, &child_status, 0) == child);
  assert(WIFEXITED(child_status) && WEXITSTATUS(child_status) == 0);
  assert(committed(shared) == 40);
  assert(shared[header_bytes + 4] == static_cast<uint8_t>(Kind::source));
  assert(shared[header_bytes + 32] == 7);
  assert(munmap(mapped, header_bytes + 128) == 0);
  alignas(64) std::array<uint64_t, (header_bytes + 128) / 8> race_storage{};
  auto * racing = reinterpret_cast<uint8_t *>(race_storage.data());
  assert(initialize(racing, header_bytes + 128, identity));
  MemoryWriter race_writer(racing, header_bytes + 128, identity);
  struct Pause {std::mutex mutex; std::condition_variable condition; bool entered=false; bool release=false;};
  Pause pause;
  before_publish_context = &pause;
  before_publish = [](void * context) noexcept {
      auto & state = *static_cast<Pause *>(context);
      std::unique_lock<std::mutex> lock(state.mutex);
      state.entered = true;
      state.condition.notify_all();
      state.condition.wait(lock, [&state] {return state.release;});
    };
  std::thread publishing([&] {assert(race_writer.append(source) == AppendResult::written);});
  {
    std::unique_lock<std::mutex> lock(pause.mutex);
    pause.condition.wait(lock, [&] {return pause.entered;});
    assert(committed(racing) == 0); // Encoding is done but no record is published.
    pause.release = true;
  }
  pause.condition.notify_all();
  publishing.join();
  assert(committed(racing) == 40);
  before_publish = nullptr;
  before_publish_context = nullptr;
  alignas(64) std::array<uint64_t, (header_bytes + 128) / 8> wrong_version{};
  auto * wrong = reinterpret_cast<uint8_t *>(wrong_version.data());
  assert(initialize(wrong, header_bytes + 128, identity));
  put32(wrong + 8, 2);
  MemoryWriter rejected_writer(wrong, header_bytes + 128, identity);
  assert(rejected_writer.append(source) == AppendResult::failed);
  assert(committed(wrong) == 0);
  if (argc == 2) {
    std::ofstream output(argv[1], std::ios::binary);
    output.write(reinterpret_cast<const char *>(golden), header_bytes + 4096);
    assert(output.good());
  }
}
