// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef REFERENCE_SYSTEM__SAMPLE_MANAGEMENT_HPP_
#define REFERENCE_SYSTEM__SAMPLE_MANAGEMENT_HPP_
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <limits>
#include <mutex>

#include "reference_system/msg_types.hpp"

inline std::mutex & reference_system_cout_mutex()
{
  static std::mutex mutex;
  return mutex;
}

template<typename DurationT>
void print_execution_time(
  const std::string & node_type,
  const std::string & node_name,
  const DurationT duration)
{
  std::lock_guard<std::mutex> lock(reference_system_cout_mutex());
  std::cout << node_type << " " << node_name << ": " << duration << std::endl;
}

bool set_benchmark_mode(const bool has_benchmark_mode, const bool set_value = true)
{
  static bool value{false};
  if (set_value) {value = has_benchmark_mode;}
  return value;
}

bool is_in_benchmark_mode()
{
  return set_benchmark_mode(false, false);
}

uint64_t now_as_int()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
    .count());
}

bool set_structured_output_enabled(const bool enabled, const bool set_value = true)
{
  static bool value{true};
  if (set_value) {value = enabled;}
  return value;
}

bool is_structured_output_enabled()
{
  return set_structured_output_enabled(false, false);
}

bool set_legacy_verbose_output_enabled(const bool enabled, const bool set_value = true)
{
  static bool value{false};
  if (set_value) {value = enabled;}
  return value;
}

bool is_legacy_verbose_output_enabled()
{
  return set_legacy_verbose_output_enabled(false, false);
}

template<typename SampleTypePointer>
std::string node_name_at(const SampleTypePointer & sample, uint64_t idx)
{
  return std::string(reinterpret_cast<const char *>(sample->stats[idx].node_name.data()));
}

template<typename SampleTypePointer>
std::set<std::string> extract_node_names(const SampleTypePointer & sample)
{
  std::set<std::string> names;
  for (uint64_t i = 0; i < sample->size; ++i) {
    names.insert(node_name_at(sample, i));
  }
  return names;
}

template<typename SampleTypePointer>
uint32_t sum_drops(const SampleTypePointer & sample, const std::set<std::string> & lineage)
{
  uint32_t total = 0;
  for (uint64_t i = 0; i < sample->size; ++i) {
    if (lineage.count(node_name_at(sample, i))) {
      total += sample->stats[i].dropped_samples;
    }
  }
  return total;
}

inline bool validate_hot_path_lineage(const std::set<std::string> & nodes)
{
  if (!nodes.count("ObjectCollisionEstimator")) return false;
  if (!nodes.count("EuclideanClusterDetector")) return false;
  if (!nodes.count("RayGroundFilter")) return false;
  if (!nodes.count("PointCloudFusion")) return false;
  bool has_lidar = nodes.count("FrontLidarDriver") || nodes.count("RearLidarDriver");
  if (!has_lidar) return false;
  return true;
}

inline bool validate_dbw_lineage(const std::set<std::string> & nodes)
{
  bool has_lidar = nodes.count("FrontLidarDriver") || nodes.count("RearLidarDriver");
  if (!has_lidar) return false;
  static const std::vector<std::string> required = {
    "PointsTransformerFront", "PointsTransformerRear", "PointCloudFusion",
    "VoxelGridDownsampler", "RayGroundFilter", "EuclideanClusterDetector",
    "ObjectCollisionEstimator", "PointCloudMap", "PointCloudMapLoader",
    "NDTLocalizer", "Visualizer", "Lanelet2GlobalPlanner", "Lanelet2Map",
    "Lanelet2MapLoader", "ParkingPlanner", "LanePlanner",
    "BehaviorPlanner", "MPCController", "VehicleInterface"
  };
  for (const auto & req : required) {
    if (!nodes.count(req)) return false;
  }
  return true;
}

inline bool validate_intersection_lineage(const std::set<std::string> & nodes)
{
  return nodes.count("EuclideanClusterSettings") &&
         nodes.count("EuclideanClusterDetector");
}

template<typename SampleTypePointer>
std::string extract_source_node(
  const SampleTypePointer & sample, bool use_earliest)
{
  std::string source_node;
  uint64_t source_ts = use_earliest ? std::numeric_limits<uint64_t>::max() : 0;
  for (uint64_t i = 0; i < sample->size; ++i) {
    std::string name = node_name_at(sample, i);
    if (name == "FrontLidarDriver" || name == "RearLidarDriver") {
      uint64_t ts = sample->stats[i].timestamp;
      if (use_earliest ? (ts < source_ts) : (ts > source_ts)) {
        source_ts = ts;
        source_node = name;
      }
    }
  }
  return source_node;
}

template<typename SampleTypePointer>
uint32_t extract_source_sequence(const SampleTypePointer & sample, const std::string & source_node)
{
  for (uint64_t i = 0; i < sample->size; ++i) {
    if (node_name_at(sample, i) == source_node) {
      return sample->stats[i].sequence_number;
    }
  }
  return 0;
}

template<typename SampleTypePointer>
uint64_t extract_source_timestamp(const SampleTypePointer & sample, const std::string & source_node)
{
  for (uint64_t i = 0; i < sample->size; ++i) {
    if (node_name_at(sample, i) == source_node) {
      return sample->stats[i].timestamp;
    }
  }
  return 0;
}

template<typename SampleTypePointer>
std::vector<std::string> extract_lineage(const SampleTypePointer & sample)
{
  std::vector<std::string> lineage;
  for (uint64_t i = 0; i < sample->size; ++i) {
    lineage.push_back(node_name_at(sample, i));
  }
  return lineage;
}

inline void emit_structured_chain_record(
  const std::string & chain_id,
  const std::string & source_node,
  uint32_t source_sequence,
  uint64_t source_timestamp_ns,
  const std::string & sink_node,
  uint32_t sink_sequence,
  uint64_t sink_timestamp_ns,
  uint64_t latency_ns,
  const std::vector<std::string> & lineage,
  const std::string & status,
  uint32_t drop_count)
{
  std::lock_guard<std::mutex> lock(reference_system_cout_mutex());
  std::ostringstream oss;
  oss << "{\"schema_version\":1";
  oss << ",\"chain_id\":\"" << chain_id << "\"";
  oss << ",\"source_node\":\"" << source_node << "\"";
  oss << ",\"source_sequence\":" << source_sequence;
  oss << ",\"source_timestamp_ns\":" << source_timestamp_ns;
  oss << ",\"sink_node\":\"" << sink_node << "\"";
  oss << ",\"sink_sequence\":" << sink_sequence;
  oss << ",\"sink_timestamp_ns\":" << sink_timestamp_ns;
  oss << ",\"latency_ns\":" << latency_ns;
  oss << ",\"lineage\":[";
  for (size_t i = 0U; i < lineage.size(); ++i) {
    if (i > 0U) oss << ",";
    oss << "\"" << lineage[i] << "\"";
  }
  oss << "]";
  oss << ",\"status\":\"" << status << "\"";
  oss << ",\"drop_count\":" << drop_count;
  oss << "}";
  std::cout << oss.str() << std::endl;
}

template<typename SampleTypePointer>
void set_sample(
  const std::string & node_name, const uint32_t sequence_number, const uint32_t dropped_samples,
  const uint64_t timestamp, SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() ) {return;}

  if (sample.size >= message_t::STATS_CAPACITY) {
    return;
  }

  uint64_t idx = sample.size;
  ++sample.size;

  memcpy(
    sample.stats[idx].node_name.data(), node_name.data(),
    std::min(
      node_name.size(),
      reference_interfaces::msg::TransmissionStats::NODE_NAME_LENGTH));

  sample.stats[idx].timestamp = timestamp;

  sample.stats[idx].sequence_number = sequence_number;
  sample.stats[idx].dropped_samples = dropped_samples;
}

template<typename SampleTypePointer>
uint64_t get_sample_timestamp(const SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() || sample->size == 0) {
    return 0;
  } else {
    return sample->stats[sample->size - 1].timestamp;
  }
}

template<typename SampleTypePointer>
uint32_t get_sample_sequence_number(const SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() || sample->size == 0) {
    return 0;
  } else {
    return sample->stats[sample->size - 1].sequence_number;
  }
}

template<typename SampleTypePointer>
uint32_t get_missed_samples_and_update_seq_nr(
  const SampleTypePointer & sample,
  uint32_t & sequence_number)
{
  uint32_t updated_seq_nr = get_sample_sequence_number(sample);
  uint32_t missed_samples =
    (updated_seq_nr > sequence_number) ? updated_seq_nr - sequence_number - 1 : 0;
  sequence_number = updated_seq_nr;
  return missed_samples;
}

template<typename SampleTypePointer, typename SourceType>
void merge_history_into_sample(SampleTypePointer & sample, const SourceType & source)
{
  if (is_in_benchmark_mode()) {return;}

  std::vector<uint64_t> entries_to_add;

  for (uint64_t i = 0; i < source->size; ++i) {
    bool entry_found = false;
    std::string source_name((const char *)source->stats[i].node_name.data());

    for (uint64_t k = 0; k < sample.size; ++k) {
      std::string sample_name((const char *)sample.stats[k].node_name.data());
      if (source_name == sample_name) {
        entry_found = true;
        break;
      }
    }

    if (!entry_found) {entries_to_add.emplace_back(i);}
  }

  for (auto i : entries_to_add) {
    memcpy(
      sample.stats.data() + sample.size, source->stats.data() + i,
      sizeof(reference_interfaces::msg::TransmissionStats));
    ++sample.size;
  }
}

struct statistic_value_t
{
  double average = 0.0;
  double deviation = 0.0;
  uint64_t min = std::numeric_limits<uint64_t>::max();
  uint64_t max = 0;
  uint64_t current = 0;
  uint64_t total_number = 0;
  std::string suffix;
  double adjustment = 0.0;

  void set(const uint64_t value)
  {
    ++total_number;
    current = value;
    double old_average = average;
    average = ((total_number - 1) * old_average + value) / total_number;
    double delta = static_cast<double>(value) - old_average;
    double delta2 = static_cast<double>(value) - average;
    deviation =
      std::sqrt(
      (deviation * deviation * (total_number - 1) + delta * delta2) / total_number);
    min = std::min(min, value);
    max = std::max(max, value);
  }
};

struct sample_statistic_t
{
  uint64_t timepoint_of_first_received_sample = 0;
  uint32_t previous_behavior_planner_sequence = 0;
  uint64_t previous_behavior_planner_time_stamp = 0;

  statistic_value_t latency;
  statistic_value_t hot_path_latency;
  statistic_value_t behavior_planner_period;
};

std::ostream & operator<<(std::ostream & output, const statistic_value_t & v)
{
  if (v.adjustment == 0.0) {
    output << v.current << v.suffix << " [min=" << v.min << v.suffix <<
      ", max=" << v.max << v.suffix << ", average=" << v.average <<
      v.suffix << ", deviation=" << v.deviation << "]";
  } else {
    output << static_cast<double>(v.current) / v.adjustment << v.suffix <<
      " [min=" << static_cast<double>(v.min) / v.adjustment << v.suffix <<
      ", max=" << static_cast<double>(v.max) / v.adjustment << v.suffix <<
      ", average=" << v.average / v.adjustment << v.suffix <<
      ", deviation=" << v.deviation / v.adjustment << v.suffix << "]";
  }
  return output;
}

template<typename SampleTypePointer>
void print_sample_path(
  const std::string & node_name,
  const uint32_t lost_samples,
  const SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() || sample->size <= 0) {return;}

  std::lock_guard<std::mutex> lock(reference_system_cout_mutex());

  static int benchmark_counter = 0;
  ++benchmark_counter;

  // benchmark_counter = dismissing first 100 samples to get rid of startup jitter
  if (benchmark_counter < 10) {return;}

  static std::map<std::string, sample_statistic_t> advanced_statistics;
  static std::map<std::string, std::map<std::string, statistic_value_t>> dropped_samples;

  auto iter = advanced_statistics.find(node_name);
  if (iter == advanced_statistics.end() ) {
    advanced_statistics[node_name].timepoint_of_first_received_sample =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    advanced_statistics[node_name].latency.suffix = "ms";
    advanced_statistics[node_name].latency.adjustment = 1000000.0;
    advanced_statistics[node_name].hot_path_latency.suffix = "ms";
    advanced_statistics[node_name].hot_path_latency.adjustment = 1000000.0;
    advanced_statistics[node_name].behavior_planner_period.suffix = "ms";
    advanced_statistics[node_name].behavior_planner_period.adjustment = 1000000.0;
  }

  const uint64_t timestamp_in_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
    .count());

  std::cout << "----------------------------------------------------------" <<
    std::endl;
  std::cout << "sample path: " << std::endl;
  std::cout <<
    "  order timepoint           sequence nr.                  node name     dropped samples" <<
    std::endl;

  std::map<uint64_t, uint64_t> timestamp2Order;
  uint64_t min_time_stamp = std::numeric_limits<uint64_t>::max();

  for (uint64_t i = 0; i < sample->size; ++i) {
    timestamp2Order[sample->stats[i].timestamp] = 0;
    min_time_stamp = std::min(min_time_stamp, sample->stats[i].timestamp);
  }
  uint64_t i = 0;
  for (auto & e : timestamp2Order) {
    e.second = i++;
  }

  for (uint64_t i = 0; i < sample->size; ++i) {
    std::string name((const char *)sample->stats[i].node_name.data());
    std::cout << "  [";
    std::cout.width(2);
    std::cout << timestamp2Order[sample->stats[i].timestamp];
    std::cout << "]  " << sample->stats[i].timestamp << "  ";
    std::cout.width(10);
    std::cout << sample->stats[i].sequence_number;
    std::cout << "    ";
    std::cout.width(24);
    std::cout << name;
    std::cout << "     ";
    dropped_samples[node_name][name].set(sample->stats[i].dropped_samples);
    std::cout << dropped_samples[node_name][name];
    std::cout << std::endl;
  }

  std::cout << "  [";
  std::cout.width(2);
  std::cout << timestamp2Order.size();
  std::cout << "]  " << timestamp_in_ns << "  ";
  std::cout.width(10);
  std::cout << "endpoint";
  std::cout << "    ";
  std::cout.width(24);
  std::cout << node_name;
  std::cout << "     ";
  dropped_samples[node_name][node_name].set(lost_samples);
  std::cout << dropped_samples[node_name][node_name];
  std::cout << std::endl;

  // hot path latency
  uint64_t hot_path_latency_in_ns = 0;
  bool does_contain_hot_path = false;
  uint64_t lidar_timestamp = 0;
  for (uint64_t i = 0; i < sample->size; ++i) {
    uint64_t idx = sample->size - i - 1;
    std::string current_node_name(
      reinterpret_cast<const char *>(sample->stats[idx].node_name.data()));

    if (current_node_name == "ObjectCollisionEstimator") {
      hot_path_latency_in_ns = sample->stats[idx].timestamp;
      does_contain_hot_path = true;
    } else if (current_node_name == "FrontLidarDriver") {
      lidar_timestamp = std::max(lidar_timestamp, sample->stats[idx].timestamp);
    } else if (current_node_name == "RearLidarDriver") {
      lidar_timestamp = std::max(lidar_timestamp, sample->stats[idx].timestamp);
    }
  }
  hot_path_latency_in_ns -= lidar_timestamp;

  // hot path drops
  uint64_t hot_path_drops = 0;
  if (does_contain_hot_path) {
    for (uint64_t i = 0; i < sample->size; ++i) {
      uint64_t idx = sample->size - i - 1;
      std::string current_node_name(
        reinterpret_cast<const char *>(sample->stats[idx].node_name.data()));
      if (current_node_name == "ObjectCollisionEstimator" ||
        current_node_name == "FrontLidarDriver" ||
        current_node_name == "PointsTransformerFront" ||
        current_node_name == "PointCloudFusion" ||
        current_node_name == "RayGroundFilter" ||
        current_node_name == "EuclideanClusterDetector")
      {
        hot_path_drops += sample->stats[idx].dropped_samples;
      }
    }
  }

  // behavior planner cycle time
  bool does_contain_behavior_planner = false;
  for (uint64_t i = 0; i < sample->size; ++i) {
    std::string current_node_name(
      reinterpret_cast<const char *>(sample->stats[i].node_name.data()));
    if (current_node_name == "BehaviorPlanner") {
      does_contain_behavior_planner = true;
      auto seq_nr = sample->stats[i].sequence_number;
      auto timestamp = sample->stats[i].timestamp;
      auto prev_seq_nr = advanced_statistics[node_name].previous_behavior_planner_sequence;
      auto prev_timestamp = advanced_statistics[node_name].previous_behavior_planner_time_stamp;
      if (prev_timestamp != 0) {
        advanced_statistics[node_name].behavior_planner_period.set(
          static_cast<double>(timestamp - prev_timestamp) /
          static_cast<double>(seq_nr - prev_seq_nr));
      }
      advanced_statistics[node_name].previous_behavior_planner_sequence = seq_nr;
      advanced_statistics[node_name].previous_behavior_planner_time_stamp = timestamp;
    }
  }

  advanced_statistics[node_name].latency.set(timestamp_in_ns - min_time_stamp);

  std::cout << std::endl;
  std::cout << "Statistics:" << std::endl;
  std::cout << "  latency:                  " << advanced_statistics[node_name].latency <<
    std::endl;

  if (does_contain_hot_path) {
    dropped_samples[node_name]["hotpath"].set(hot_path_drops);
    advanced_statistics[node_name].hot_path_latency.set(hot_path_latency_in_ns);
    std::cout <<
      "  hot path:                 " << \
      "FrontLidarDriver/RearLidarDriver (latest) -> ObjectCollisionEstimator"
              <<
      std::endl;
    std::cout << "  hot path latency:         " <<
      advanced_statistics[node_name].hot_path_latency << std::endl;
    std::cout << "  hot path drops:           " << dropped_samples[node_name]["hotpath"] <<
      std::endl;
  }

  if (does_contain_behavior_planner) {
    std::cout << "  behavior planner period:  " <<
      advanced_statistics[node_name].behavior_planner_period << std::endl;
  }

  std::cout << "----------------------------------------------------------" <<
    std::endl;
  std::cout << std::endl;
}

#endif  // REFERENCE_SYSTEM__SAMPLE_MANAGEMENT_HPP_
