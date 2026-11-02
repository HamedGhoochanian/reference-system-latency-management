#ifndef REFERENCE_SYSTEM__TRACKING_HPP_
#define REFERENCE_SYSTEM__TRACKING_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <ctime>
#include <unistd.h>

#include "reference_system/stdout_mutex.hpp"
#include "reference_system/telemetry_buffer.hpp"

namespace reference_system_tracking
{

inline uint64_t monotonic_now_ns()
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<uint64_t>(now.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(now.tv_nsec);
}

inline std::string json_quote(const std::string & value)
{
  std::string result = "\"";
  for (const char character : value) {
    if (character == '\\' || character == '"') {result += '\\';}
    if (character == '\n') {result += "\\n"; continue;}
    if (character == '\r') {result += "\\r"; continue;}
    result += character;
  }
  result += '"';
  return result;
}

inline std::string with_identity(
  const std::string & event, const std::string & producer,
  const std::string & instance, const std::string & boot_id, uint64_t sequence)
{
  std::string result = event;
  const std::string schema = "\"schema\":1";
  const auto schema_at = result.find(schema);
  if (schema_at != std::string::npos) {result.replace(schema_at, schema.size(), "\"schema\":2");}
  const auto end = result.rfind('}');
  if (end == std::string::npos) {return result;}
  result.insert(end,
    ",\"producer\":" + json_quote(producer) +
    ",\"instance\":" + json_quote(instance) +
    ",\"boot_id\":" + json_quote(boot_id) +
    ",\"record_seq\":" + std::to_string(sequence));
  return result;
}

enum class WriterMode
{
  synchronous,
  asynchronous,
  memory
};

inline WriterMode parse_writer_mode(const std::string & value)
{
  if (value == "sync") {return WriterMode::synchronous;}
  if (value == "async") {return WriterMode::asynchronous;}
  throw std::invalid_argument(
          "REFERENCE_SYSTEM_TRACKING_WRITER must be 'sync' or 'async', got '" + value + "'");
}

inline WriterMode writer_mode_from_environment()
{
  const char * transport = std::getenv("REFERENCE_SYSTEM_TELEMETRY_TRANSPORT");
  if (transport && std::string(transport) == "memory") return WriterMode::memory;
  if (transport && std::string(transport) != "file") {
    throw std::invalid_argument("REFERENCE_SYSTEM_TELEMETRY_TRANSPORT must be 'file' or 'memory'");
  }
  const char * value = std::getenv("REFERENCE_SYSTEM_TRACKING_WRITER");
  return parse_writer_mode(value == nullptr ? "sync" : value);
}

struct TrackingSource
{
  uint32_t kind = 0;
  uint32_t sequence = 0;
  uint64_t timestamp_ns = 0;
  bool valid() const {return kind != 0;}
};

struct TrackingMessage
{
  std::string name;
  uint32_t sequence = 0;
  uint32_t channel = 0;
};

class TrackingWriter
{
public:
  using Write = std::function<void(const std::string &)>;
  using Clock = std::function<uint64_t()>;
  inline static constexpr size_t kDefaultQueueCapacity = 4096;

  class Callback
  {
  public:
    Callback() = default;
    Callback(TrackingWriter * writer, std::string producer, uint64_t token)
    : writer_(writer), producer_(std::move(producer)), token_(token) {}
    Callback(const Callback &) = delete;
    Callback & operator=(const Callback &) = delete;
    Callback(Callback && other) noexcept
    : writer_(other.writer_), producer_(std::move(other.producer_)), token_(other.token_)
    {other.writer_ = nullptr;}
    Callback & operator=(Callback && other) noexcept
    {
      if (this != &other) {
        finish(); writer_ = other.writer_; producer_ = std::move(other.producer_);
        token_ = other.token_; other.writer_ = nullptr;
      }
      return *this;
    }
    ~Callback() {finish();}
  private:
    void finish() {if (writer_ != nullptr) {writer_->end_callback(producer_, token_); writer_ = nullptr;}}
    TrackingWriter * writer_ = nullptr;
    std::string producer_;
    uint64_t token_ = 0;
  };

  TrackingWriter(std::string boot_id, std::string instance_id, Write write,
    Clock clock = monotonic_now_ns, WriterMode mode = WriterMode::synchronous,
    size_t queue_capacity = kDefaultQueueCapacity,
    reference_system_telemetry::MemoryWriter * memory_writer = nullptr)
  : boot_id_(std::move(boot_id)), instance_id_(std::move(instance_id)),
    write_(std::move(write)), clock_(std::move(clock)), mode_(mode),
    queue_capacity_(queue_capacity), memory_writer_(memory_writer)
  {
    if (mode_ == WriterMode::memory && !memory_writer_) {
      throw std::invalid_argument("memory tracking writer needs mapped buffer");
    }
    if (mode_ == WriterMode::asynchronous) {
      if (queue_capacity_ == 0) {throw std::invalid_argument("tracking queue capacity must be positive");}
      consumer_ = std::thread([this] {consume();});
    }
  }

  ~TrackingWriter()
  {
    try {shutdown();} catch (...) {}
  }

  TrackingWriter(const TrackingWriter &) = delete;
  TrackingWriter & operator=(const TrackingWriter &) = delete;

  WriterMode mode() const {return mode_;}

  Callback begin_callback(const std::string & producer)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == WriterMode::memory && stopping_) {
      throw std::runtime_error("memory telemetry writer is shutting down");
    }
    auto & state = producers_[producer];
    const uint64_t token = ++state.next_callback;
    const auto active = state.active.emplace(token, 0).first;
    try {
      active->second = clock_();
    } catch (...) {
      state.active.erase(active);
      throw;
    }
    return Callback(this, producer, token);
  }

  Callback begin_callback(const std::string & producer, uint64_t entry_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == WriterMode::memory && stopping_) {
      throw std::runtime_error("memory telemetry writer is shutting down");
    }
    auto & state = producers_[producer];
    const uint64_t token = ++state.next_callback;
    const auto active = state.active.emplace(token, 0).first;
    active->second = entry_ns;
    return Callback(this, producer, token);
  }

  void register_producer(const std::string & producer)
  {std::lock_guard<std::mutex> lock(mutex_); producers_[producer];}

  void write_event(const std::string & producer, uint64_t timestamp_ns, const std::string & event)
  {
    (void)timestamp_ns;
    if (mode_ == WriterMode::memory) {
      throw std::logic_error("raw JSON events are not accepted by the memory tracking writer");
    }
    if (mode_ == WriterMode::asynchronous) {
      throw std::logic_error("raw JSON events are not accepted by the asynchronous tracking writer");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    throw_if_unavailable_locked();
    auto & state = producers_[producer];
    write_(with_identity(event, producer, instance_id_, boot_id_, state.next_record++));
  }

  void write_source(
    const std::string & producer, uint32_t sequence, uint32_t kind, uint64_t timestamp_ns)
  {
    if (mode_ == WriterMode::memory) {
      reference_system_telemetry::RecordView record;
      record.kind = reference_system_telemetry::Kind::source;
      record.message_sequence = sequence;
      record.source_kind = kind;
      record.timestamp_ns = static_cast<int64_t>(timestamp_ns);
      write_memory(producer, record);
      return;
    }
    write_typed(SourceEvent{producer, sequence, kind, timestamp_ns});
  }

  void write_link(
    const std::string & producer, uint32_t sequence, uint32_t channel, uint64_t timestamp_ns,
    const std::vector<std::pair<uint32_t, TrackingMessage>> & inputs,
    const TrackingSource & source, bool timer)
  {
    if (mode_ == WriterMode::memory) {
      write_memory_link(producer, sequence, channel, timestamp_ns, inputs, source, timer);
      return;
    }
    write_typed(LinkEvent{producer, sequence, channel, timestamp_ns, inputs, source, timer});
  }

  template<typename Message, typename Source>
  void write_link(
    const std::string & producer, uint32_t sequence, uint32_t channel, uint64_t timestamp_ns,
    const std::vector<std::pair<uint32_t, Message>> & inputs,
    const Source & source, bool timer)
  {
    if (mode_ == WriterMode::memory) {
      write_memory_link(producer, sequence, channel, timestamp_ns, inputs, source, timer);
      return;
    }
    std::vector<std::pair<uint32_t, TrackingMessage>> copied_inputs;
    copied_inputs.reserve(inputs.size());
    for (const auto & input : inputs) {
      copied_inputs.emplace_back(
        input.first,
        TrackingMessage{input.second.name, input.second.sequence, input.second.channel});
    }
    const TrackingSource copied_source{source.kind, source.sequence, source.timestamp_ns};
    write_typed(LinkEvent{
      producer, sequence, channel, timestamp_ns, std::move(copied_inputs), copied_source, timer});
  }

  void write_sink(
    const std::string & producer, uint32_t sequence, uint64_t timestamp_ns,
    const TrackingMessage & input, const TrackingSource & source)
  {
    if (mode_ == WriterMode::memory) {
      write_memory_sink(producer, sequence, timestamp_ns, input, source);
      return;
    }
    write_typed(SinkEvent{producer, sequence, timestamp_ns, input, source});
  }

  template<typename Message, typename Source>
  void write_sink(
    const std::string & producer, uint32_t sequence, uint64_t timestamp_ns,
    const Message & input, const Source & source)
  {
    if (mode_ == WriterMode::memory) {
      write_memory_sink(producer, sequence, timestamp_ns, input, source);
      return;
    }
    const TrackingMessage copied_input{input.name, input.sequence, input.channel};
    const TrackingSource copied_source{source.kind, source.sequence, source.timestamp_ns};
    write_typed(SinkEvent{producer, sequence, timestamp_ns, copied_input, copied_source});
  }

  void report_delivery(const std::string & producer, uint64_t now_ns)
  {
    if (mode_ == WriterMode::memory) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) throw std::runtime_error("memory telemetry writer is shutting down");
      auto & state = producers_[producer];
      reference_system_telemetry::RecordView record;
      record.kind = reference_system_telemetry::Kind::delivery;
      record.timestamp_ns = static_cast<int64_t>(now_ns);
      record.through_ns = guard_limited_watermark(state, now_ns);
      append_memory(state, record);
      return;
    }
    if (mode_ == WriterMode::asynchronous) {
      enqueue_delivery(producer, now_ns);
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    throw_if_unavailable_locked();
    auto & state = producers_[producer];
    const uint64_t through_ns = guard_limited_watermark(state, now_ns);
    write_(delivery_json(producer, state.next_record++, through_ns));
  }

  void write_auxiliary(const std::string & producer, reference_system_telemetry::RecordView record)
  {
    if (mode_ != WriterMode::memory) throw std::logic_error("auxiliary memory record in file mode");
    std::lock_guard<std::mutex> lock(mutex_);
    auto & state = producers_[producer];
    record.sequence = state.next_auxiliary;
    if (memory_writer_->append(record) != reference_system_telemetry::AppendResult::written) {
      throw std::runtime_error("memory telemetry append failed");
    }
    ++state.next_auxiliary;
  }

  void shutdown()
  {
    if (mode_ == WriterMode::memory) {
      std::unique_lock<std::mutex> lock(mutex_);
      if (memory_closed_) return;
      stopping_ = true;
      const bool drained = callbacks_idle_.wait_for(lock, std::chrono::seconds(5), [this] {
          for (const auto & producer : producers_) {
            if (!producer.second.active.empty()) return false;
          }
          return true;
        });
      if (!drained) {
        memory_writer_->fail(3);
        throw std::runtime_error("memory telemetry callbacks did not drain");
      }
      for (auto & producer : producers_) {
        reference_system_telemetry::RecordView record;
        record.kind = reference_system_telemetry::Kind::delivery;
        record.timestamp_ns = static_cast<int64_t>(monotonic_now_ns());
        record.through_ns = static_cast<uint64_t>(record.timestamp_ns);
        append_memory(producer.second, record);
      }
      memory_closed_ = true;
      return;
    }
    if (mode_ != WriterMode::asynchronous) {return;}
    std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    not_empty_.notify_all();
    not_full_.notify_all();
    if (consumer_.joinable()) {consumer_.join();}
    std::lock_guard<std::mutex> lock(mutex_);
    throw_if_failed_locked();
  }

private:
  struct SourceEvent
  {
    std::string producer;
    uint32_t sequence;
    uint32_t kind;
    uint64_t timestamp_ns;
  };
  struct LinkEvent
  {
    std::string producer;
    uint32_t sequence;
    uint32_t channel;
    uint64_t timestamp_ns;
    std::vector<std::pair<uint32_t, TrackingMessage>> inputs;
    TrackingSource source;
    bool timer;
  };
  struct SinkEvent
  {
    std::string producer;
    uint32_t sequence;
    uint64_t timestamp_ns;
    TrackingMessage input;
    TrackingSource source;
  };
  struct DeliveryEvent
  {
    std::string producer;
    uint64_t through_ns;
  };
  using QueueEntry = std::variant<SourceEvent, LinkEvent, SinkEvent, DeliveryEvent>;

  struct ProducerState
  {
    uint64_t next_record = 0;
    uint64_t next_auxiliary = 0;
    uint64_t next_callback = 0;
    std::map<uint64_t, uint64_t> active;
  };

  void write_memory(const std::string & producer, reference_system_telemetry::RecordView record)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    append_memory(producers_[producer], record);
  }

  void append_memory(ProducerState & state, reference_system_telemetry::RecordView & record)
  {
    record.sequence = state.next_record;
    const auto result = memory_writer_->append(record);
    if (result != reference_system_telemetry::AppendResult::written) {
      throw std::runtime_error("memory telemetry append failed");
    }
    ++state.next_record;
  }

  template<typename Message, typename Source>
  void write_memory_link(
    const std::string & producer, uint32_t sequence, uint32_t channel, uint64_t timestamp_ns,
    const std::vector<std::pair<uint32_t, Message>> & inputs, const Source & source, bool timer)
  {
    std::array<reference_system_telemetry::InputView, 64> views{};
    for (size_t i = 0; i < std::min(inputs.size(), views.size()); ++i) {
      views[i] = {inputs[i].first, inputs[i].second.name,
        inputs[i].second.sequence, inputs[i].second.channel};
    }
    reference_system_telemetry::RecordView record;
    record.kind = reference_system_telemetry::Kind::link;
    record.name = producer;
    record.message_sequence = sequence;
    record.channel = channel;
    record.timestamp_ns = static_cast<int64_t>(timestamp_ns);
    record.source_kind = source.kind;
    record.source_sequence = source.sequence;
    record.source_timestamp_ns = source.timestamp_ns;
    record.timer = timer;
    record.inputs = views.data();
    record.input_count = inputs.size();
    write_memory(producer, record);
  }

  template<typename Message, typename Source>
  void write_memory_sink(
    const std::string & producer, uint32_t sequence, uint64_t timestamp_ns,
    const Message & input, const Source & source)
  {
    reference_system_telemetry::RecordView record;
    record.kind = reference_system_telemetry::Kind::sink;
    record.message_sequence = sequence;
    record.timestamp_ns = static_cast<int64_t>(timestamp_ns);
    record.source_kind = source.kind;
    record.source_sequence = source.sequence;
    record.source_timestamp_ns = source.timestamp_ns;
    record.input = {0, input.name, input.sequence, input.channel};
    write_memory(producer, record);
  }

  template<typename Event>
  void write_typed(Event event)
  {
    QueueEntry entry(std::move(event));
    if (mode_ == WriterMode::asynchronous) {
      enqueue(std::move(entry));
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    throw_if_unavailable_locked();
    const std::string producer = producer_of(entry);
    auto & state = producers_[producer];
    write_(serialize(entry, producer, state.next_record++));
  }

  void enqueue(QueueEntry entry)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_full_.wait(lock, [this] {
      return failed_ || stopping_ || outstanding_entries_ < queue_capacity_;
    });
    throw_if_unavailable_locked();
    queue_.push_back(std::move(entry));
    ++outstanding_entries_;
    lock.unlock();
    not_empty_.notify_one();
  }

  void enqueue_delivery(const std::string & producer, uint64_t now_ns)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_full_.wait(lock, [this] {
      return failed_ || stopping_ || outstanding_entries_ < queue_capacity_;
    });
    throw_if_unavailable_locked();
    auto & state = producers_[producer];
    const uint64_t through_ns = guard_limited_watermark(state, now_ns);
    queue_.emplace_back(DeliveryEvent{producer, through_ns});
    ++outstanding_entries_;
    lock.unlock();
    not_empty_.notify_one();
  }

  static uint64_t guard_limited_watermark(const ProducerState & state, uint64_t now_ns)
  {
    uint64_t through_ns = now_ns;
    for (const auto & callback : state.active) {
      if (callback.second == 0) {through_ns = 0;}
      else {through_ns = std::min(through_ns, callback.second - 1);}
    }
    return through_ns;
  }

  std::string delivery_json(
    const std::string & producer, uint64_t sequence, uint64_t through_ns)
  {
    return "{\"schema\":2,\"event\":\"delivery\",\"producer\":" + json_quote(producer) +
           ",\"instance\":" + json_quote(instance_id_) +
           ",\"boot_id\":" + json_quote(boot_id_) +
           ",\"record_seq\":" + std::to_string(sequence) +
           ",\"through_ns\":" + std::to_string(through_ns) + "}";
  }

  static std::string source_json(const SourceEvent & event)
  {
    return "{\"schema\":1,\"event\":\"source\",\"t_ns\":" +
           std::to_string(event.timestamp_ns) + ",\"node\":\"" + event.producer +
           "\",\"seq\":" + std::to_string(event.sequence) +
           ",\"kind\":" + std::to_string(event.kind) + "}";
  }

  static std::string source_identity_json(const TrackingSource & source)
  {
    if (!source.valid()) {return "null";}
    return "{\"kind\":" + std::to_string(source.kind) +
           ",\"seq\":" + std::to_string(source.sequence) +
           ",\"t_ns\":" + std::to_string(source.timestamp_ns) + "}";
  }

  static std::string link_json(const LinkEvent & event)
  {
    std::string record = "{\"schema\":1,\"event\":\"link\",\"t_ns\":" +
      std::to_string(event.timestamp_ns) + ",\"node\":\"" + event.producer +
      "\",\"msg\":{\"name\":\"" + event.producer + "\",\"seq\":" +
      std::to_string(event.sequence) + ",\"channel\":" +
      std::to_string(event.channel) + "},\"inputs\":[";
    for (size_t index = 0; index < event.inputs.size(); ++index) {
      if (index != 0) {record += ",";}
      const auto & input = event.inputs[index];
      record += "{\"index\":" + std::to_string(input.first) +
        ",\"msg\":{\"name\":\"" + input.second.name + "\",\"seq\":" +
        std::to_string(input.second.sequence) + ",\"channel\":" +
        std::to_string(input.second.channel) + "}}";
    }
    record += "],\"source\":" + source_identity_json(event.source) +
      ",\"timer\":" + std::string(event.timer ? "true" : "false") + "}";
    return record;
  }

  static std::string sink_json(const SinkEvent & event)
  {
    return "{\"schema\":1,\"event\":\"sink\",\"t_ns\":" +
      std::to_string(event.timestamp_ns) + ",\"node\":\"" + event.producer +
      "\",\"msg\":{\"name\":\"" + event.producer + "\",\"seq\":" +
      std::to_string(event.sequence) +
      ",\"channel\":0},\"input\":{\"name\":\"" + event.input.name +
      "\",\"seq\":" + std::to_string(event.input.sequence) +
      ",\"channel\":" + std::to_string(event.input.channel) +
      "},\"source\":" + source_identity_json(event.source) +
      ",\"sensor_started\":" + std::string(event.source.valid() ? "true" : "false") + "}";
  }

  static std::string producer_of(const QueueEntry & entry)
  {
    return std::visit([](const auto & record) {return record.producer;}, entry);
  }

  std::string serialize(
    const QueueEntry & entry, const std::string & producer, uint64_t sequence) const
  {
    return std::visit([this, &producer, sequence](const auto & record) -> std::string {
      using Record = std::decay_t<decltype(record)>;
      if constexpr (std::is_same_v<Record, DeliveryEvent>) {
        return "{\"schema\":2,\"event\":\"delivery\",\"producer\":" +
          json_quote(record.producer) + ",\"instance\":" + json_quote(instance_id_) +
          ",\"boot_id\":" + json_quote(boot_id_) +
          ",\"record_seq\":" + std::to_string(sequence) +
          ",\"through_ns\":" + std::to_string(record.through_ns) + "}";
      } else {
        std::string event;
        if constexpr (std::is_same_v<Record, SourceEvent>) {event = source_json(record);}
        else if constexpr (std::is_same_v<Record, LinkEvent>) {event = link_json(record);}
        else {event = sink_json(record);}
        return with_identity(event, producer, instance_id_, boot_id_, sequence);
      }
    }, entry);
  }

  void consume()
  {
    while (true) {
      QueueEntry entry;
      std::string producer;
      uint64_t sequence = 0;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait(lock, [this] {return stopping_ || failed_ || !queue_.empty();});
        if (failed_) {return;}
        if (queue_.empty()) {
          if (stopping_) {return;}
          continue;
        }
        entry = std::move(queue_.front());
        queue_.pop_front();
        producer = producer_of(entry);
        sequence = producers_[producer].next_record++;
      }
      try {
        write_(serialize(entry, producer, sequence));
      } catch (const std::exception & error) {
        fail(error.what());
        return;
      } catch (...) {
        fail("unknown exception while writing tracking record");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        --outstanding_entries_;
      }
      not_full_.notify_all();
    }
  }

  void fail(const std::string & message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!failed_) {
        failed_ = true;
        failure_message_ = message;
      }
    }
    not_full_.notify_all();
    not_empty_.notify_all();
  }

  void throw_if_unavailable_locked() const
  {
    throw_if_failed_locked();
    if (stopping_) {throw std::runtime_error("tracking writer is shutting down");}
  }

  void throw_if_failed_locked() const
  {
    if (failed_) {throw std::runtime_error("tracking writer failed: " + failure_message_);}
  }

  void end_callback(const std::string & producer, uint64_t token)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = producers_.find(producer);
    if (found != producers_.end()) {found->second.active.erase(token);}
    if (mode_ == WriterMode::memory) callbacks_idle_.notify_all();
  }
  std::string boot_id_;
  std::string instance_id_;
  Write write_;
  Clock clock_;
  WriterMode mode_;
  size_t queue_capacity_;
  reference_system_telemetry::MemoryWriter * memory_writer_;
  std::mutex mutex_;
  std::map<std::string, ProducerState> producers_;
  std::deque<QueueEntry> queue_;
  size_t outstanding_entries_ = 0;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;
  std::condition_variable callbacks_idle_;
  std::thread consumer_;
  std::mutex shutdown_mutex_;
  bool stopping_ = false;
  bool memory_closed_ = false;
  bool failed_ = false;
  std::string failure_message_;
};

inline bool enabled()
{
  if (const char * transport = std::getenv("REFERENCE_SYSTEM_TELEMETRY_TRANSPORT")) {
    if (std::string(transport) == "memory") return true;
  }
  const char * value = std::getenv("REFERENCE_SYSTEM_TRACKING");
  return value != nullptr && (std::string(value) == "1" || std::string(value) == "true");
}

inline std::string read_boot_id()
{
  if (const char * override_value = std::getenv("REFERENCE_SYSTEM_BOOT_ID")) {
    if (*override_value != '\0') {return override_value;}
  }
  std::ifstream file("/proc/sys/kernel/random/boot_id");
  std::string result;
  std::getline(file, result);
  return result;
}

inline std::string process_instance_id()
{
  if (const char * override_value = std::getenv("REFERENCE_SYSTEM_INSTANCE_ID")) {
    if (*override_value != '\0') {return override_value;}
  }
  std::ostringstream result;
  result << std::hex << monotonic_now_ns() << '-' << static_cast<unsigned long>(getpid());
  return result.str();
}

class Runtime
{
public:
  explicit Runtime(WriterMode mode = writer_mode_from_environment())
  : boot_id_(read_boot_id()), instance_id_(process_instance_id()),
    mapping_(mode == WriterMode::memory ?
      std::make_unique<reference_system_telemetry::MappedBuffer>(
        std::getenv("REFERENCE_SYSTEM_TELEMETRY_BUFFER_PATH"),
        reference_system_telemetry::Identity{
          std::getenv("REFERENCE_SYSTEM_RUN_ID") ? std::getenv("REFERENCE_SYSTEM_RUN_ID") : "",
          std::getenv("REFERENCE_SYSTEM_TELEMETRY_PRODUCER") ?
          std::getenv("REFERENCE_SYSTEM_TELEMETRY_PRODUCER") : "",
          boot_id_, instance_id_}) : nullptr),
    writer_(boot_id_, instance_id_, [mode](const std::string & record) {
      if (mode == WriterMode::synchronous) {
        reference_system_write_stdout_line(record);
        return;
      }
      std::lock_guard<std::mutex> lock(reference_system_cout_mutex());
      std::cout << record << std::endl;
      if (!std::cout) {throw std::runtime_error("tracking stdout write or flush failed");}
    }, monotonic_now_ns, mode, TrackingWriter::kDefaultQueueCapacity,
    mapping_ ? &mapping_->writer() : nullptr) {}
  ~Runtime()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    if (reporter_.joinable()) {reporter_.join();}
    if (writer_.mode() == WriterMode::memory) {
      try {writer_.shutdown();} catch (const std::exception & error) {
        std::cerr << "memory telemetry shutdown failed: " << error.what() << std::endl;
      }
    }
  }
  TrackingWriter & writer() {return writer_;}
  void register_producer(const std::string & producer)
  {
    if (mapping_) {
      const char * expected = std::getenv("REFERENCE_SYSTEM_TELEMETRY_PRODUCER");
      if (!expected || producer != expected) throw std::runtime_error("memory telemetry producer mismatch");
    }
    writer_.register_producer(producer);
    std::lock_guard<std::mutex> lock(mutex_);
    if (std::find(producers_.begin(), producers_.end(), producer) == producers_.end()) {
      producers_.push_back(producer);
    }
    if (!reporter_.joinable()) {reporter_ = std::thread([this] {report_loop();});}
  }
private:
  void report_loop()
  {
    unsigned long interval_ms = 100;
    if (const char * setting = std::getenv("REFERENCE_SYSTEM_DELIVERY_INTERVAL_MS")) {
      char * end = nullptr;
      const auto parsed = std::strtoul(setting, &end, 10);
      if (end != setting && *end == '\0' && parsed > 0 && parsed <= 60000) {interval_ms = parsed;}
    }
    std::unique_lock<std::mutex> lock(mutex_);
    while (!condition_.wait_for(lock, std::chrono::milliseconds(interval_ms), [this] {return stopping_;})) {
      lock.unlock();
      for (const auto & producer : registered_producers()) {
        try {
          writer_.report_delivery(producer, monotonic_now_ns());
        } catch (const std::exception & error) {
          std::cerr << "tracking delivery reporter failed: " << error.what() << std::endl;
          std::lock_guard<std::mutex> failure_lock(mutex_);
          stopping_ = true;
          condition_.notify_all();
          return;
        }
      }
      lock.lock();
    }
  }
  std::vector<std::string> registered_producers()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return producers_;
  }
  std::string boot_id_, instance_id_;
  std::unique_ptr<reference_system_telemetry::MappedBuffer> mapping_;
  TrackingWriter writer_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread reporter_;
  bool stopping_ = false;
  std::vector<std::string> producers_;
};

inline Runtime & runtime()
{
  static Runtime instance;
  return instance;
}

inline void register_producer(const std::string & producer)
{
  (void)writer_mode_from_environment();
  if (!enabled()) {return;}
  auto & instance = runtime();
  instance.writer().register_producer(producer);
  // Runtime owns a single reporter; retaining a list keeps its idle reports scoped to known nodes.
  instance.register_producer(producer);
}

inline TrackingWriter::Callback begin_callback(const std::string & producer)
{
  if (!enabled()) {return {};}
  auto & instance = runtime();
  return instance.writer().begin_callback(producer);
}

}  // namespace reference_system_tracking
#endif  // REFERENCE_SYSTEM__TRACKING_HPP_
