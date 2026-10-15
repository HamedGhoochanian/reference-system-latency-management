// SPDX-License-Identifier: Apache-2.0
// Private, versioned source-control files. No executor or timer replacement.
#ifndef REFERENCE_SYSTEM__SOURCE_SCHEDULE_HPP_
#define REFERENCE_SYSTEM__SOURCE_SCHEDULE_HPP_
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace nodes {
class SourceSchedule {
public:
  struct Phase {std::string id; int64_t offset; int64_t period;};
  SourceSchedule(const std::string & path, const std::string & source,
    const std::string & boot, const std::string & instance, int64_t initial)
  : start_path_((std::filesystem::path(path).parent_path() / "start").string()) {
    std::ifstream file(path);
    std::string magic, declared_source, declared_instance;
    size_t count = 0;
    if (!(file >> magic >> session_ >> boot_ >> hash_ >> declared_source >> declared_instance >> duration_ >> count) ||
      magic != "LAME_SOURCE_SCHEDULE_V1" || declared_source != source || boot_ != boot ||
      declared_instance != instance || duration_ <= 0 || count != 3) {
      throw std::runtime_error("invalid or stale source schedule identity");
    }
    for (const auto & token : {session_, boot_, hash_, source, instance}) {check_token(token);}
    std::set<std::string> ids;
    for (size_t i=0; i<count; ++i) {
      Phase phase{};
      if (!(file >> phase.id >> phase.offset >> phase.period) || phase.period <= 0 ||
        phase.offset < 0 || phase.offset >= duration_ ||
        (i == 0 ? phase.offset != 0 : phase.offset <= phases_.back().offset) ||
        !ids.insert(phase.id).second) {throw std::runtime_error("invalid source schedule phase");}
      check_token(phase.id);
      phases_.push_back(phase);
    }
    std::string extra;
    if ((file >> extra) || phases_.front().period != initial ||
      phases_.front().period != phases_.back().period) {
      throw std::runtime_error("source schedule periods or trailing content invalid");
    }
  }

  std::vector<size_t> due(int64_t now) {
    if (!epoch_) {
      if (!std::filesystem::exists(start_path_)) {return {};}
      std::ifstream start(start_path_);
      std::string magic, session, boot, hash, extra;
      int64_t epoch=0;
      if (!(start >> magic >> session >> boot >> hash >> epoch) || (start >> extra) ||
        magic != "LAME_SOURCE_START_V1" || session != session_ || boot != boot_ || hash != hash_ ||
        epoch <= 0 || epoch > std::numeric_limits<int64_t>::max() - duration_) {
        throw std::runtime_error("invalid or stale source start record");
      }
      epoch_ = epoch;
    }
    std::vector<size_t> result;
    for (size_t i=next_; i<phases_.size() && now >= requested(i); ++i) {result.push_back(i);}
    return result;
  }
  void applied(size_t index) {next_=index+1;}
  int64_t requested(size_t index) const {return epoch_ + phases_.at(index).offset;}
  const Phase & phase(size_t index) const {return phases_.at(index);}
  const std::string & hash() const {return hash_;}
  const std::string & session() const {return session_;}
  void record(std::ostream & out, size_t index, const std::string & status,
    const std::string & source, const std::string & boot, const std::string & instance,
    int64_t old_period, int64_t new_period, int64_t application) const {
    out << "{\"version\":1,\"clock\":\"CLOCK_MONOTONIC\",\"session\":\"" << session_
      << "\",\"boot_id\":\"" << boot << "\",\"schedule_hash\":\"" << hash_
      << "\",\"source\":\"" << source << "\",\"instance\":\"" << instance
      << "\",\"phase_id\":\"" << phase(index).id << "\",\"status\":\"" << status
      << "\",\"requested_ns\":" << requested(index) << ",\"applied_ns\":" << application
      << ",\"old_period_ns\":" << old_period << ",\"new_period_ns\":" << new_period << "}\n";
    out.flush();
    if (!out) {throw std::runtime_error("source application record failed");}
  }
  static void check_token(const std::string & token) {
    if (token.empty() || token == "." || token == "..") {throw std::runtime_error("empty source identity");}
    for (auto c : token) {
      if (!((c>='a' && c<='z') || (c>='A' && c<='Z') || (c>='0' && c<='9') || c=='-' || c=='_' || c=='.')) {
        throw std::runtime_error("unsafe source identity");
      }
    }
  }
private:
  std::string start_path_, session_, boot_, hash_;
  int64_t duration_=0, epoch_=0;
  size_t next_=0;
  std::vector<Phase> phases_;
};
} // namespace nodes
#endif
