#ifndef REFERENCE_SYSTEM__TELEMETRY_BUFFER_HPP_
#define REFERENCE_SYSTEM__TELEMETRY_BUFFER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string_view>
#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace reference_system_telemetry {

constexpr size_t header_bytes = 4096;
constexpr size_t max_record_bytes = 8192;
static_assert(__atomic_always_lock_free(8, nullptr), "shared telemetry needs lock-free 64-bit atomics");

enum class Kind : uint16_t {
  source = 1, link = 2, sink = 3, delivery = 4,
  timing = 5, source_application = 6, diagnostic = 7
};
enum class AppendResult { written, full, failed };

struct Identity {
  std::string_view run, producer, boot, instance;
};

struct InputView {
  uint32_t index = 0;
  std::string_view name;
  uint32_t sequence = 0;
  uint32_t channel = 0;
};

struct RootView {
  std::string_view name;
  uint32_t sequence = 0;
  uint64_t timestamp_ns = 0;
};

struct RecordView {
  Kind kind = static_cast<Kind>(0);
  uint64_t sequence = 0;
  int64_t timestamp_ns = 0;
  uint32_t message_sequence = 0;
  uint32_t channel = 0;
  uint32_t source_kind = 0;
  uint32_t source_sequence = 0;
  uint64_t source_timestamp_ns = 0;
  bool timer = false;
  const InputView * inputs = nullptr;
  size_t input_count = 0;
  InputView input;
  uint64_t through_ns = 0;
  std::string_view name;
  std::string_view aux_name;
  std::string_view session;
  std::string_view chain_id;
  std::string_view sink_name;
  const RootView * roots = nullptr;
  size_t root_count = 0;
  const std::string_view * lineage = nullptr;
  size_t lineage_count = 0;
  uint32_t drop_count = 0;
  uint64_t instrumentation_errors = 0;
  uint64_t metadata_overflows = 0;
  int64_t value_ns = 0;
  int64_t requested_ns = 0;
  int64_t old_period_ns = 0;
  int64_t new_period_ns = 0;
  uint8_t status = 0;
};

inline void put16(uint8_t * p, uint16_t n) noexcept
{p[0] = static_cast<uint8_t>(n); p[1] = static_cast<uint8_t>(n >> 8);}
inline void put32(uint8_t * p, uint32_t n) noexcept
{for (unsigned i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(n >> (i * 8));}
inline void put64(uint8_t * p, uint64_t n) noexcept
{for (unsigned i = 0; i < 8; ++i) p[i] = static_cast<uint8_t>(n >> (i * 8));}
inline uint64_t get64(const uint8_t * p) noexcept
{uint64_t n = 0; for (unsigned i = 0; i < 8; ++i) n |= uint64_t(p[i]) << (i * 8); return n;}
inline uint32_t get32(const uint8_t * p) noexcept
{uint32_t n = 0; for (unsigned i = 0; i < 4; ++i) n |= uint32_t(p[i]) << (i * 8); return n;}

inline uint64_t shared_load(const uint8_t * bytes, size_t offset) noexcept
{return __atomic_load_n(reinterpret_cast<const uint64_t *>(bytes + offset), __ATOMIC_ACQUIRE);}
inline void shared_store(uint8_t * bytes, size_t offset, uint64_t value) noexcept
{__atomic_store_n(reinterpret_cast<uint64_t *>(bytes + offset), value, __ATOMIC_RELEASE);}
inline uint64_t committed(const uint8_t * bytes) noexcept {return shared_load(bytes, 576);}
inline uint64_t failure(const uint8_t * bytes) noexcept {return shared_load(bytes, 640);}

#ifdef REFERENCE_SYSTEM_TELEMETRY_TEST_HOOK
// Only built into the standalone harness, never into workload processes.
inline void (*before_publish)(void *) noexcept = nullptr;
inline void * before_publish_context = nullptr;
#endif

inline bool valid_identity(std::string_view value) noexcept
{return !value.empty() && value.size() < 128 && value.find('\0') == std::string_view::npos;}

inline bool initialize(uint8_t * bytes, size_t capacity, const Identity & id) noexcept
{
  if (!bytes || (reinterpret_cast<uintptr_t>(bytes) & 63) || capacity < header_bytes ||
    !valid_identity(id.run) || !valid_identity(id.producer) ||
    !valid_identity(id.boot) || !valid_identity(id.instance)) return false;
  std::memset(bytes, 0, header_bytes);
  std::memcpy(bytes, "LAMETLM1", 8);
  put32(bytes + 8, 1);
  put32(bytes + 12, header_bytes);
  put64(bytes + 16, capacity);
  std::memcpy(bytes + 24, id.run.data(), id.run.size());
  std::memcpy(bytes + 152, id.producer.data(), id.producer.size());
  std::memcpy(bytes + 280, id.boot.data(), id.boot.size());
  std::memcpy(bytes + 408, id.instance.data(), id.instance.size());
  return true;
}

class MemoryWriter {
public:
  MemoryWriter(uint8_t * bytes, size_t size, const Identity & id) noexcept
  : bytes_(bytes), size_(size)
  {
    if (!bytes || (reinterpret_cast<uintptr_t>(bytes) & 63) || size < header_bytes ||
      std::memcmp(bytes, "LAMETLM1", 8) || get32(bytes + 8) != 1 ||
      get32(bytes + 12) != header_bytes || get64(bytes + 16) != size ||
      committed(bytes) > size - header_bytes || committed(bytes) % 8) return;
    const std::array<std::string_view, 4> fields{id.run, id.producer, id.boot, id.instance};
    for (size_t i = 0; i < fields.size(); ++i) {
      if (!valid_identity(fields[i]) ||
        std::memcmp(bytes + 24 + 128 * i, fields[i].data(), fields[i].size()) ||
        bytes[24 + 128 * i + fields[i].size()] != 0) return;
    }
    valid_ = true;
  }

  bool valid() const noexcept {return valid_;}

  void fail(uint64_t code) noexcept
  {
    std::lock_guard<std::mutex> guard(lock_);
    if (valid_ && !failure(bytes_)) shared_store(bytes_, 640, code);
  }

  AppendResult append(const RecordView & record) noexcept
  {
    std::lock_guard<std::mutex> guard(lock_);
    if (!valid_ || failure(bytes_)) return AppendResult::failed;
    std::array<uint8_t, max_record_bytes> encoded{};
    size_t used = 32;
    auto reserve = [&](size_t n) -> uint8_t * {
        if (n > encoded.size() - used) return nullptr;
        uint8_t * result = encoded.data() + used; used += n; return result;
      };
    auto u8 = [&](uint8_t value) -> bool {auto * p = reserve(1); if (!p) return false; *p = value; return true;};
    auto u16 = [&](uint16_t value) -> bool {auto * p = reserve(2); if (!p) return false; put16(p, value); return true;};
    auto u32 = [&](uint32_t value) -> bool {auto * p = reserve(4); if (!p) return false; put32(p, value); return true;};
    auto u64 = [&](uint64_t value) -> bool {auto * p = reserve(8); if (!p) return false; put64(p, value); return true;};
    auto text = [&](std::string_view value) -> bool {
        if (value.size() > 255) return false;
        if (!u16(static_cast<uint16_t>(value.size()))) return false;
        auto * p = reserve(value.size());
        if (!p) return false;
        std::memcpy(p, value.data(), value.size()); return true;
      };
    auto input = [&](const InputView & v) -> bool {
        return u32(v.index) && u32(v.sequence) && u32(v.channel) && text(v.name);
      };
    bool ok = record.timestamp_ns >= 0;
    switch (record.kind) {
      case Kind::source:
        ok = ok && u32(record.message_sequence) && u32(record.source_kind) && record.source_kind != 0;
        break;
      case Kind::link:
        ok = ok && record.input_count <= 64 && (record.input_count == 0 || record.inputs) &&
          u32(record.message_sequence) && u32(record.channel) && u32(record.source_kind) &&
          u32(record.source_sequence) && u64(record.source_timestamp_ns) &&
          u8(record.timer ? 1 : 0) && u8(static_cast<uint8_t>(record.input_count)) &&
          text(record.name);
        for (size_t i = 0; ok && i < record.input_count; ++i) ok = input(record.inputs[i]);
        break;
      case Kind::sink:
        ok = ok && u32(record.message_sequence) && u32(record.source_kind) &&
          u32(record.source_sequence) && u64(record.source_timestamp_ns) && input(record.input);
        break;
      case Kind::delivery: ok = ok && u64(record.through_ns); break;
      case Kind::timing: ok = ok && text(record.name) && text(record.aux_name) &&
          u64(static_cast<uint64_t>(record.value_ns)); break;
      case Kind::source_application:
        ok = ok && text(record.name) && text(record.aux_name) && text(record.session) &&
          u8(record.status) && u64(static_cast<uint64_t>(record.requested_ns)) &&
          u64(static_cast<uint64_t>(record.old_period_ns)) &&
          u64(static_cast<uint64_t>(record.new_period_ns)); break;
      case Kind::diagnostic:
        ok = ok && record.root_count <= 64 && record.lineage_count <= 64 &&
          (record.root_count == 0 || record.roots) &&
          (record.lineage_count == 0 || record.lineage) &&
          text(record.name) && text(record.chain_id) && text(record.aux_name) &&
          text(record.sink_name) && u32(record.source_sequence) &&
          u64(record.source_timestamp_ns) && u32(record.message_sequence) &&
          u64(static_cast<uint64_t>(record.value_ns)) && u8(record.status) &&
          u32(record.drop_count) && u64(record.instrumentation_errors) &&
          u64(record.metadata_overflows) && u8(static_cast<uint8_t>(record.root_count));
        for (size_t i = 0; ok && i < record.root_count; ++i) {
          ok = text(record.roots[i].name) && u32(record.roots[i].sequence) &&
            u64(record.roots[i].timestamp_ns);
        }
        ok = ok && u8(static_cast<uint8_t>(record.lineage_count));
        for (size_t i = 0; ok && i < record.lineage_count; ++i) ok = text(record.lineage[i]);
        break;
      default: ok = false;
    }
    if (!ok) {shared_store(bytes_, 640, 2); return AppendResult::failed;}
    const size_t length = (used + 7) & ~size_t(7);
    const uint64_t at = committed(bytes_);
    if (at > size_ - header_bytes || length > size_ - header_bytes - at) {
      shared_store(bytes_, 640, 1); return AppendResult::full;
    }
    put32(encoded.data(), static_cast<uint32_t>(length));
    put16(encoded.data() + 4, static_cast<uint16_t>(record.kind));
    put64(encoded.data() + 8, record.sequence);
    put64(encoded.data() + 16, static_cast<uint64_t>(record.timestamp_ns));
    put32(encoded.data() + 24, static_cast<uint32_t>(used - 32));
    std::memcpy(bytes_ + header_bytes + at, encoded.data(), length);
#ifdef REFERENCE_SYSTEM_TELEMETRY_TEST_HOOK
    if (before_publish) before_publish(before_publish_context);
#endif
    shared_store(bytes_, 576, at + length);
    shared_store(bytes_, 704, shared_load(bytes_, 704) + 1);
    return AppendResult::written;
  }
private:
  uint8_t * bytes_;
  size_t size_;
  bool valid_ = false;
  std::mutex lock_;
};

class MappedBuffer {
public:
  MappedBuffer(const char * path, const Identity & identity)
  {
#ifdef __linux__
    if (!path || !*path) throw std::invalid_argument("memory telemetry buffer path is missing");
    const int fd = open(path, O_RDWR | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) throw std::runtime_error("cannot open telemetry buffer");
    struct stat info{};
    if (fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) || info.st_size < static_cast<off_t>(header_bytes)) {
      close(fd);
      throw std::runtime_error("invalid telemetry buffer file");
    }
    size_ = static_cast<size_t>(info.st_size);
    void * address = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (address == MAP_FAILED) throw std::runtime_error("cannot map telemetry buffer");
    bytes_ = static_cast<uint8_t *>(address);
    writer_ = std::make_unique<MemoryWriter>(bytes_, size_, identity);
    if (!writer_->valid()) {
      writer_.reset(); munmap(bytes_, size_); bytes_ = nullptr;
      throw std::runtime_error("telemetry buffer header or identity mismatch");
    }
#else
    (void)path; (void)identity;
    throw std::runtime_error("memory telemetry requires Linux");
#endif
  }
  MappedBuffer(const MappedBuffer &) = delete;
  MappedBuffer & operator=(const MappedBuffer &) = delete;
  ~MappedBuffer() {
    writer_.reset();
#ifdef __linux__
    if (bytes_) munmap(bytes_, size_);
#endif
  }
  MemoryWriter & writer() noexcept {return *writer_;}
private:
  uint8_t * bytes_ = nullptr;
  size_t size_ = 0;
  std::unique_ptr<MemoryWriter> writer_;
};
} // namespace reference_system_telemetry
#endif
