// Copyright 2026 Eindhoven University of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REFERENCE_SYSTEM__EVENT_LOGGING_HPP_
#define REFERENCE_SYSTEM__EVENT_LOGGING_HPP_

#include <fcntl.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace reference_system
{
namespace events
{

struct SourceExecutionId
{
  std::string component;
  uint32_t sequence;
  uint64_t timestamp_ns;
};

struct CausalInput
{
  size_t input_index;
  SourceExecutionId callback_id;
};

struct EventResult
{
  uint64_t timestamp_ns = 0;
  bool queued = false;
};

inline uint64_t monotonic_now_ns()
{
  timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
    throw std::runtime_error("clock_gettime(CLOCK_MONOTONIC) failed");
  }
  return static_cast<uint64_t>(now.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(now.tv_nsec);
}

inline std::string escape_json(const std::string & value)
{
  std::ostringstream escaped;
  for (unsigned char c : value) {
    switch (c) {
      case '\\': escaped << "\\\\"; break;
      case '"': escaped << "\\\""; break;
      case '\b': escaped << "\\b"; break;
      case '\f': escaped << "\\f"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (c < 0x20U) {
          static constexpr char hex[] = "0123456789abcdef";
          escaped << "\\u00" << hex[c >> 4U] << hex[c & 0x0fU];
        } else {
          escaped << static_cast<char>(c);
        }
    }
  }
  return escaped.str();
}

inline std::string source_execution_id_json(const SourceExecutionId & id)
{
  std::ostringstream json;
  json << "{\"component\":\"" << escape_json(id.component) << "\"";
  json << ",\"sequence\":" << id.sequence;
  json << ",\"timestamp_ns\":" << id.timestamp_ns << "}";
  return json.str();
}

inline std::string source_identities_json(const std::vector<SourceExecutionId> & ids)
{
  std::ostringstream json;
  json << "[";
  for (size_t i = 0; i < ids.size(); ++i) {
    if (i != 0) {json << ",";}
    json << source_execution_id_json(ids[i]);
  }
  json << "]";
  return json.str();
}

inline std::string causal_inputs_json(const std::vector<CausalInput> & inputs)
{
  std::ostringstream json;
  json << "[";
  for (size_t i = 0; i < inputs.size(); ++i) {
    if (i != 0) {json << ",";}
    json << "{\"input_index\":" << inputs[i].input_index;
    json << ",\"callback_execution_id\":" << source_execution_id_json(inputs[i].callback_id);
    json << "}";
  }
  json << "]";
  return json.str();
}

inline std::string string_array_json(const std::vector<std::string> & values)
{
  std::ostringstream json;
  json << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {json << ",";}
    json << "\"" << escape_json(values[i]) << "\"";
  }
  json << "]";
  return json.str();
}

class ComponentEventLogger
{
public:
  using WriteFunction = ssize_t (*)(int, const void *, size_t);

  ComponentEventLogger(
    std::string component,
    std::string directory,
    std::string run_id,
    std::chrono::milliseconds progress_interval = std::chrono::milliseconds{100},
    size_t queue_capacity = 4096,
    WriteFunction write_function = ::write)
  : component_(std::move(component)),
    run_id_(std::move(run_id)),
    progress_interval_(progress_interval),
    queue_capacity_(queue_capacity),
    write_function_(write_function)
  {
    if (component_.empty() || run_id_.empty()) {
      throw std::invalid_argument("event component and run ID must not be empty");
    }
    if (progress_interval_.count() <= 0 || queue_capacity_ == 0 || write_function_ == nullptr) {
      throw std::invalid_argument("invalid event logger settings");
    }

    const std::filesystem::path directory_path{directory};
    if (!std::filesystem::is_directory(directory_path) ||
      access(directory_path.c_str(), W_OK | X_OK) != 0)
    {
      throw std::runtime_error("event directory must exist and be writable");
    }

    boot_id_ = read_boot_id();
    const auto start_time = monotonic_now_ns();
    producer_id_ = component_ + ":" + std::to_string(getpid()) + ":" +
      std::to_string(start_time);

    file_path_ = (directory_path / (safe_filename(component_) + ".jsonl")).string();
    file_descriptor_ = open(
      file_path_.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_APPEND | O_CLOEXEC, 0644);
    if (file_descriptor_ < 0) {
      throw std::runtime_error("could not create component event file: " + file_path_);
    }

    if (!enqueue_record("producer_start", start_time, "")) {
      close(file_descriptor_);
      file_descriptor_ = -1;
      throw std::runtime_error("could not queue component start record");
    }

    try {
      writer_ = std::thread(&ComponentEventLogger::writer_loop, this);
    } catch (...) {
      close(file_descriptor_);
      file_descriptor_ = -1;
      throw;
    }
  }

  ComponentEventLogger(const ComponentEventLogger &) = delete;
  ComponentEventLogger & operator=(const ComponentEventLogger &) = delete;

  ~ComponentEventLogger()
  {
    stop();
  }

  EventResult source_entry(
    uint32_t source_sequence,
    const std::vector<std::string> & chain_ids)
  {
    return emit("source_entry", [this, source_sequence, &chain_ids](uint64_t timestamp) {
        std::ostringstream fields;
        fields << "\"chain_ids\":" << string_array_json(chain_ids);
        fields << ",\"source_execution_id\":" << source_execution_id_json(
          {component_, source_sequence, timestamp});
        return fields.str();
      });
  }

  EventResult callback_entry(uint32_t callback_sequence)
  {
    return emit("callback_entry", [this, callback_sequence](uint64_t timestamp) {
        std::ostringstream fields;
        fields << "\"callback_execution_id\":" << source_execution_id_json(
          {component_, callback_sequence, timestamp});
        return fields.str();
      });
  }

  EventResult input_entry(
    uint32_t callback_sequence, size_t input_index,
    const std::vector<SourceExecutionId> & input_lineage)
  {
    return emit("input_entry", [this, callback_sequence, input_index, &input_lineage]
      (uint64_t timestamp) {
        std::ostringstream fields;
        fields << "\"callback_execution_id\":" << source_execution_id_json(
          {component_, callback_sequence, timestamp});
        fields << ",\"input_index\":" << input_index;
        // Raw input stats: do not collapse by component. Two branches can carry
        // different executions of the same source before the next merge.
        fields << ",\"input_lineage\":" << source_identities_json(input_lineage);
        return fields.str();
      });
  }

  EventResult output_dependencies(
    uint32_t output_sequence, uint64_t output_timestamp_ns,
    const std::vector<CausalInput> & inputs,
    const std::optional<SourceExecutionId> & timer_callback = std::nullopt)
  {
    return emit("output_dependencies", [this, output_sequence, output_timestamp_ns,
        &inputs, &timer_callback](uint64_t) {
        std::ostringstream fields;
        fields << "\"output_id\":" << source_execution_id_json(
          {component_, output_sequence, output_timestamp_ns});
        fields << ",\"causal_inputs\":" << causal_inputs_json(inputs);
        fields << ",\"timer_callback_execution_id\":";
        fields << (timer_callback ? source_execution_id_json(*timer_callback) : "null");
        return fields.str();
      });
  }

  EventResult sink_finish(
    const std::string & chain_id,
    const std::optional<SourceExecutionId> & source_id,
    uint32_t sink_sequence,
    const std::vector<SourceExecutionId> & source_candidates,
    const std::vector<SourceExecutionId> & lineage,
    const std::string & lineage_error = "",
    const std::string & source_selection = "configured",
    const std::vector<SourceExecutionId> & input_lineage = {})
  {
    return emit("sink_finish", [
        &chain_id, source_id, sink_sequence, &source_candidates, &lineage,
        &lineage_error, &source_selection, &input_lineage]
      (uint64_t timestamp) {
        std::ostringstream fields;
        fields << "\"chain_id\":\"" << escape_json(chain_id) << "\"";
        fields << ",\"source_execution_id\":";
        fields << (source_id ? source_execution_id_json(*source_id) : "null");
        fields << ",\"sink_sequence\":" << sink_sequence;
        fields << ",\"source_candidates\":" << source_identities_json(source_candidates);
        fields << ",\"lineage\":" << source_identities_json(lineage);
        fields << ",\"input_lineage\":" << source_identities_json(input_lineage);
        fields << ",\"source_selection\":\"" << escape_json(source_selection) << "\"";
        const bool lineage_complete = source_id.has_value() && lineage_error.empty() &&
          (source_selection == "configured" || source_selection == "earliest_contributor");
        fields << ",\"lineage_complete\":" << (lineage_complete ? "true" : "false");
        if (!lineage_error.empty()) {
          fields << ",\"lineage_error\":\"" << escape_json(lineage_error) << "\"";
        } else if (!lineage_complete) {
          fields << ",\"lineage_error\":\"source_match_unavailable\"";
        }
        fields << ",\"sink_timestamp_ns\":" << timestamp;
        return fields.str();
      });
  }

  EventResult diagnostic(
    const std::string & diagnostic_name,
    const std::string & fields_json)
  {
    return emit("diagnostic", [&diagnostic_name, &fields_json](uint64_t) {
        return "\"diagnostic_name\":\"" + escape_json(diagnostic_name) + "\"," + fields_json;
      });
  }

  EventResult behavior_planner_jitter(
    uint32_t callback_sequence,
    uint64_t callback_start_ns,
    uint64_t previous_start_ns,
    uint64_t expected_period_ns)
  {
    return emit("diagnostic", [=](uint64_t finish_ns) {
        std::ostringstream fields;
        fields << "\"diagnostic_name\":\"behavior_planner_timer_period\"";
        fields << ",\"callback_sequence\":" << callback_sequence;
        fields << ",\"callback_start_ns\":" << callback_start_ns;
        fields << ",\"callback_finish_ns\":" << finish_ns;
        fields << ",\"expected_period_ns\":" << expected_period_ns;
        if (previous_start_ns == 0 || callback_start_ns < previous_start_ns) {
          fields << ",\"period_ns\":null,\"period_error_ns\":null";
        } else {
          const uint64_t period = callback_start_ns - previous_start_ns;
          fields << ",\"period_ns\":" << period;
          fields << ",\"period_error_ns\":";
          if (period >= expected_period_ns) {
            fields << (period - expected_period_ns);
          } else {
            fields << "-" << (expected_period_ns - period);
          }
        }
        return fields.str();
      });
  }

  bool healthy() const
  {
    return !failed_.load();
  }

  const std::string & file_path() const
  {
    return file_path_;
  }

  const std::string & producer_id() const
  {
    return producer_id_;
  }

private:
  template<typename FieldsBuilder>
  EventResult emit(const std::string & record_type, FieldsBuilder build_fields)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stopping_.load() || failed_.load()) {
        throw std::runtime_error("component event logger is not accepting events");
      }
      active_emitters_.fetch_add(1);
    }
    struct EmitterGuard
    {
      std::atomic<uint32_t> & active;
      std::condition_variable & ready;
      ~EmitterGuard()
      {
        active.fetch_sub(1);
        ready.notify_one();
      }
    } guard{active_emitters_, queue_ready_};

    try {
      const uint64_t timestamp = monotonic_now_ns();
      const std::string fields = build_fields(timestamp);
      if (!enqueue_record(record_type, timestamp, fields)) {
        throw std::runtime_error("could not queue component event");
      }
      return {timestamp, true};
    } catch (...) {
      mark_failed("event_serialization_failed");
      queue_ready_.notify_one();
      throw;
    }
  }

  bool enqueue_record(
    const std::string & record_type,
    uint64_t timestamp_ns,
    const std::string & fields_json)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (failed_.load() || queue_.size() >= queue_capacity_) {
      if (!failed_.load()) {
        failure_reason_ = "event_queue_full";
      }
      if (queue_.size() >= queue_capacity_) {++lost_record_count_;}
      failed_.store(true);
      queue_ready_.notify_one();
      return false;
    }

    queue_.push_back(make_record(record_type, timestamp_ns, fields_json));
    queue_ready_.notify_one();
    return true;
  }

  void writer_loop() noexcept
  {
    try {
      writer_loop_impl();
    } catch (...) {
      write_failed_.store(true);
      failed_.store(true);
      queue_ready_.notify_all();
    }
  }

  void writer_loop_impl()
  {
    auto next_progress = std::chrono::steady_clock::now() + progress_interval_;
    std::optional<uint64_t> pending_watermark;
    bool final_watermark_requested = false;

    while (true) {
      if (stopping_.load() && !final_watermark_requested) {
        final_watermark_requested = true;
        const uint64_t observed = monotonic_now_ns();
        pending_watermark = observed == 0 ? 0 : observed - 1;
      }

      if (!failed_.load() && !pending_watermark &&
        std::chrono::steady_clock::now() >= next_progress)
      {
        const uint64_t observed = monotonic_now_ns();
        pending_watermark = observed == 0 ? 0 : observed - 1;
        next_progress = std::chrono::steady_clock::now() + progress_interval_;
      }

      if (!failed_.load() && pending_watermark && active_emitters_.load() == 0) {
        const uint64_t event_time = monotonic_now_ns();
        const std::string fields = "\"delivery_through_ns\":" +
          std::to_string(*pending_watermark);
        if (!enqueue_record("progress", event_time, fields)) {
          failed_.store(true);
        }
        pending_watermark.reset();
      }

      std::string record;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (!queue_.empty()) {
          record = std::move(queue_.front());
          queue_.pop_front();
        } else if (failed_.load() && !write_failed_.load() && !error_record_written_) {
          error_record_written_ = true;
          std::ostringstream fields;
          fields << "\"failure_reason\":\"" << escape_json(failure_reason_) << "\"";
          fields << ",\"lost_record_count\":" << lost_record_count_;
          record = make_record("producer_error", monotonic_now_ns(), fields.str());
        } else if (stopping_.load() && !pending_watermark && active_emitters_.load() == 0) {
          break;
        } else if (failed_.load()) {
          break;
        } else if (pending_watermark || stopping_.load()) {
          queue_ready_.wait_for(
            lock, std::chrono::milliseconds{1}, [this, &pending_watermark] {
              return !queue_.empty() || failed_.load() ||
                     (pending_watermark && active_emitters_.load() == 0);
            });
        } else {
          queue_ready_.wait_until(
            lock, next_progress, [this] {
              return !queue_.empty() || stopping_.load() || failed_.load();
            });
        }
      }
      if (!record.empty() && !write_all(record)) {
        write_failed_.store(true);
        failed_.store(true);
        queue_ready_.notify_one();
        break;
      }
    }
  }

  bool write_all(const std::string & record)
  {
    size_t written_total = 0;
    while (written_total < record.size()) {
      const ssize_t written = write_function_(
        file_descriptor_, record.data() + written_total, record.size() - written_total);
      if (written < 0 && errno == EINTR) {continue;}
      if (written <= 0) {return false;}
      written_total += static_cast<size_t>(written);
    }
    return true;
  }

  std::string make_record(
    const std::string & record_type,
    uint64_t timestamp_ns,
    const std::string & fields_json)
  {
    std::ostringstream record;
    record << "{\"schema_version\":1";
    record << ",\"record_type\":\"" << escape_json(record_type) << "\"";
    record << ",\"producer_component\":\"" << escape_json(component_) << "\"";
    record << ",\"producer_id\":\"" << escape_json(producer_id_) << "\"";
    record << ",\"run_id\":\"" << escape_json(run_id_) << "\"";
    record << ",\"host_boot_id\":\"" << escape_json(boot_id_) << "\"";
    record << ",\"event_seq\":" << next_event_sequence_++;
    record << ",\"timestamp_ns\":" << timestamp_ns;
    if (!fields_json.empty()) {record << "," << fields_json;}
    record << "}\n";
    return record.str();
  }

  void mark_failed(const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!failed_.load()) {failure_reason_ = reason;}
    failed_.store(true);
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stopping_.exchange(true)) {return;}
    }
    queue_ready_.notify_all();
    if (writer_.joinable()) {writer_.join();}
    if (file_descriptor_ >= 0) {
      close(file_descriptor_);
      file_descriptor_ = -1;
    }
  }

  static std::string read_boot_id()
  {
    std::ifstream input("/proc/sys/kernel/random/boot_id");
    std::string boot_id;
    if (!input || !std::getline(input, boot_id) || boot_id.empty()) {
      throw std::runtime_error("could not read host boot ID");
    }
    return boot_id;
  }

  static std::string safe_filename(const std::string & value)
  {
    std::string filename = value;
    for (char & c : filename) {
      const bool safe = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
      if (!safe) {c = '_';}
    }
    if (filename == "." || filename == "..") {
      throw std::invalid_argument("invalid component name for event file");
    }
    return filename;
  }

  std::string component_;
  std::string run_id_;
  std::string boot_id_;
  std::string producer_id_;
  std::string file_path_;
  std::chrono::milliseconds progress_interval_;
  size_t queue_capacity_;
  WriteFunction write_function_;
  int file_descriptor_ = -1;
  std::atomic<uint32_t> active_emitters_{0};
  std::atomic<bool> failed_{false};
  std::atomic<bool> write_failed_{false};
  std::mutex queue_mutex_;
  std::condition_variable queue_ready_;
  std::deque<std::string> queue_;
  uint64_t next_event_sequence_ = 1;
  uint64_t lost_record_count_ = 0;
  std::string failure_reason_;
  bool error_record_written_ = false;
  std::atomic<bool> stopping_{false};
  std::thread writer_;
};

inline std::unique_ptr<ComponentEventLogger> make_component_event_logger(
  const std::string & component)
{
  const char * required = std::getenv("LAME_REQUIRE_EVENT_LOGGING");
  const char * directory = std::getenv("LAME_EVENT_DIR");
  if (directory == nullptr) {
    if (required != nullptr && std::string{required} == "1") {
      throw std::runtime_error("LAME_EVENT_DIR is required for event logging");
    }
    return nullptr;
  }
  const char * run_id = std::getenv("LAME_RUN_ID");
  if (run_id == nullptr || std::string{run_id}.empty()) {
    throw std::runtime_error("LAME_RUN_ID is required when LAME_EVENT_DIR is set");
  }
  return std::make_unique<ComponentEventLogger>(component, directory, run_id);
}

}  // namespace events
}  // namespace reference_system

#endif  // REFERENCE_SYSTEM__EVENT_LOGGING_HPP_
