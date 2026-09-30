#ifndef REFERENCE_SYSTEM__TRACKING_HPP_
#define REFERENCE_SYSTEM__TRACKING_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ctime>
#include <unistd.h>

#include "reference_system/stdout_mutex.hpp"

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

class TrackingWriter
{
public:
  using Write = std::function<void(const std::string &)>;
  using Clock = std::function<uint64_t()>;

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
    Clock clock = monotonic_now_ns)
  : boot_id_(std::move(boot_id)), instance_id_(std::move(instance_id)),
    write_(std::move(write)), clock_(std::move(clock)) {}

  Callback begin_callback(const std::string & producer)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return begin_callback_locked(producer, clock_());
  }

  Callback begin_callback(const std::string & producer, uint64_t entry_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return begin_callback_locked(producer, entry_ns);
  }

  void register_producer(const std::string & producer)
  {std::lock_guard<std::mutex> lock(mutex_); producers_[producer];}

  void write_event(const std::string & producer, uint64_t timestamp_ns, const std::string & event)
  {
    (void)timestamp_ns;
    std::lock_guard<std::mutex> lock(mutex_);
    auto & state = producers_[producer];
    write_(with_identity(event, producer, instance_id_, boot_id_, state.next_record++));
  }

  void report_delivery(const std::string & producer, uint64_t now_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & state = producers_[producer];
    uint64_t through_ns = now_ns;
    for (const auto & callback : state.active) {
      if (callback.second == 0) {through_ns = 0;}
      else {through_ns = std::min(through_ns, callback.second - 1);}
    }
    write_("{\"schema\":2,\"event\":\"delivery\",\"producer\":" + json_quote(producer) +
      ",\"instance\":" + json_quote(instance_id_) + ",\"boot_id\":" + json_quote(boot_id_) +
      ",\"record_seq\":" + std::to_string(state.next_record++) +
      ",\"through_ns\":" + std::to_string(through_ns) + "}");
  }

private:
  Callback begin_callback_locked(const std::string & producer, uint64_t entry_ns)
  {
    auto & state = producers_[producer];
    const uint64_t token = ++state.next_callback;
    state.active.emplace(token, entry_ns);
    return Callback(this, producer, token);
  }
  struct ProducerState
  {
    uint64_t next_record = 0;
    uint64_t next_callback = 0;
    std::map<uint64_t, uint64_t> active;
  };
  void end_callback(const std::string & producer, uint64_t token)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = producers_.find(producer);
    if (found != producers_.end()) {found->second.active.erase(token);}
  }
  std::string boot_id_;
  std::string instance_id_;
  Write write_;
  Clock clock_;
  std::mutex mutex_;
  std::map<std::string, ProducerState> producers_;
};

inline bool enabled()
{
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
  Runtime()
  : writer_(read_boot_id(), process_instance_id(), [](const std::string & record) {
      reference_system_write_stdout_line(record);
    }) {}
  ~Runtime()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    if (reporter_.joinable()) {reporter_.join();}
  }
  TrackingWriter & writer() {return writer_;}
  void register_producer(const std::string & producer)
  {
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
      for (const auto & producer : registered_producers()) {writer_.report_delivery(producer, monotonic_now_ns());}
      lock.lock();
    }
  }
  std::vector<std::string> registered_producers()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return producers_;
  }
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
