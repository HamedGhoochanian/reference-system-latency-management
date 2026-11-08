// Copyright 2026 Eindhoven University of Technology
// Licensed under the Apache License 2.0

#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "reference_system/nodes/rclcpp/command.hpp"
#include "reference_system/nodes/rclcpp/cyclic.hpp"
#include "reference_system/nodes/rclcpp/fusion.hpp"
#include "reference_system/nodes/rclcpp/intersection.hpp"
#include "reference_system/nodes/rclcpp/sensor.hpp"
#include "reference_system/nodes/rclcpp/transform.hpp"

namespace
{
// Dispatch through the real subscription boundary without introducing DDS arrival
// races. Node callbacks, computation, publication and provenance remain real.
std::string deliver(rclcpp::Node & node, const std::string & topic, uint32_t sequence)
{
  const auto subscription = node.get_node_base_interface()->get_default_callback_group()->
    find_subscription_ptrs_if([&](const auto & sub) {return topic == sub->get_topic_name();});
  if (!subscription) {throw std::runtime_error("missing test subscription: " + topic);}
  auto message = std::make_shared<message_t>();
  set_sample(topic.substr(1), sequence, 0, 1, *message);
  set_provenance(*message, topic.substr(1), sequence, 0, {1, sequence, 1});
  std::shared_ptr<void> erased = message;
  testing::internal::CaptureStdout();
  subscription->handle_message(erased, rclcpp::MessageInfo{});
  return testing::internal::GetCapturedStdout();
}

bool published(const std::string & events)
{
  return events.find("\"event\":\"link\"") != std::string::npos;
}

class InputConsumption : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(InputConsumption, FusionConsumesBothInputsAndKeepsInputZeroTrigger)
{
  nodes::FusionSettings settings{};
  settings.node_name = "TestFusion";
  settings.input_0 = "input_zero";
  settings.input_1 = "input_one";
  settings.output_topic = "fusion_output";
  settings.number_crunch_limit = 0;
  nodes::rclcpp_system::Fusion node(settings);

  EXPECT_FALSE(published(deliver(node, "/input_zero", 1)));
  EXPECT_FALSE(published(deliver(node, "/input_one", 1)));
  const auto first = deliver(node, "/input_zero", 2);
  EXPECT_TRUE(published(first));
  EXPECT_NE(first.find("\"name\":\"input_one\",\"seq\":1"), std::string::npos);
  // A new trigger must not reuse input one from the preceding publication.
  EXPECT_FALSE(published(deliver(node, "/input_zero", 3)));
  EXPECT_FALSE(published(deliver(node, "/input_one", 2)));
  const auto second = deliver(node, "/input_zero", 4);
  EXPECT_TRUE(published(second));
  EXPECT_NE(second.find("\"name\":\"input_zero\",\"seq\":4"), std::string::npos);
  EXPECT_NE(second.find("\"name\":\"input_one\",\"seq\":2"), std::string::npos);
}

TEST_F(InputConsumption, CyclicStillConsumesInputsOncePerTimerPublication)
{
  nodes::CyclicSettings settings{};
  settings.node_name = "TestCyclic";
  settings.inputs = {"cyclic_a", "cyclic_b", "cyclic_c", "cyclic_d", "cyclic_e", "cyclic_f"};
  settings.output_topic = "cyclic_output";
  settings.cycle_time = std::chrono::hours(1);
  settings.number_crunch_limit = 0;
  nodes::rclcpp_system::Cyclic node(settings);
  const auto timer = node.get_node_base_interface()->get_default_callback_group()->
    find_timer_ptrs_if([](const auto &) {return true;});
  ASSERT_NE(timer, nullptr);
  EXPECT_FALSE(published(deliver(node, "/cyclic_a", 1)));
  testing::internal::CaptureStdout();
  timer->execute_callback();
  const auto first = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(published(first));
  EXPECT_NE(first.find("\"name\":\"cyclic_a\",\"seq\":1"), std::string::npos);
  testing::internal::CaptureStdout();
  timer->execute_callback();
  const auto second = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(published(second));
  EXPECT_NE(second.find("\"inputs\":[]"), std::string::npos);
}

TEST_F(InputConsumption, WorkloadSubscriptionsUseDoubledMiddlewareHistories)
{
  using namespace nodes::rclcpp_system;
  nodes::SensorSettings sensor{};
  sensor.node_name = "QueueSensor";
  sensor.topic_name = "queue_sensor";
  sensor.cycle_time = std::chrono::hours(1);
  nodes::TransformSettings transform{};
  transform.node_name = "QueueTransform";
  transform.input_topic = "queue_sensor";
  transform.output_topic = "queue_transform";
  nodes::FusionSettings fusion{};
  fusion.node_name = "QueueFusion";
  fusion.input_0 = "queue_sensor";
  fusion.input_1 = "queue_transform";
  fusion.output_topic = "queue_fusion";
  nodes::CyclicSettings cyclic{};
  cyclic.node_name = "QueueCyclic";
  cyclic.inputs = {"qa", "qb", "qc", "qd", "qe", "qf"};
  cyclic.output_topic = "queue_cyclic";
  cyclic.cycle_time = std::chrono::hours(1);
  nodes::IntersectionSettings intersection{};
  intersection.node_name = "QueueIntersection";
  intersection.connections = {{"queue_fusion", "queue_intersection_a", 0},
    {"queue_cyclic", "queue_intersection_b", 0}};
  nodes::CommandSettings command{};
  command.node_name = "QueueCommand";
  command.input_topic = "queue_intersection_a";
  const std::vector<std::shared_ptr<rclcpp::Node>> nodes{
    std::make_shared<Sensor>(sensor), std::make_shared<Transform>(transform),
    std::make_shared<Fusion>(fusion), std::make_shared<Cyclic>(cyclic),
    std::make_shared<Intersection>(intersection), std::make_shared<Command>(command)};
  size_t subscriptions = 0;
  for (const auto & node : nodes) {
    node->get_node_base_interface()->get_default_callback_group()->find_subscription_ptrs_if(
      [&](const auto & sub) {
        if (std::string(sub->get_topic_name()) == "/parameter_events") {return false;}
        ++subscriptions;
        const size_t expected = std::string(node->get_name()) == "QueueCommand" ? 20U : 2U;
        EXPECT_EQ(sub->get_actual_qos().depth(), expected) << sub->get_topic_name();
        return false;
      });
  }
  EXPECT_EQ(subscriptions, 12U);
}
}  // namespace
