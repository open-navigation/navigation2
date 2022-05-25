// Copyright (c) 2022 Samsung Research
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_velocity_smoother/velocity_smoother.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class VelSmootherShim : public nav2_velocity_smoother::VelocitySmoother
{
public:
  VelSmootherShim()
  : VelocitySmoother() {}
  void configure(const rclcpp_lifecycle::State & state) {this->on_configure(state);}
  void activate(const rclcpp_lifecycle::State & state) {this->on_activate(state);}
  void deactivate(const rclcpp_lifecycle::State & state) {this->on_deactivate(state);}
  void cleanup(const rclcpp_lifecycle::State & state) {this->on_cleanup(state);}
  void shutdown(const rclcpp_lifecycle::State & state) {this->on_shutdown(state);}

  bool isOdomSmoother() {return odom_smoother_ ? true : false;}
  bool hasCommandMsg() {return last_command_time_.nanoseconds() != 0;}
  geometry_msgs::msg::Twist::SharedPtr lastCommandMsg() {return command_;}

  void sendCommandMsg(geometry_msgs::msg::Twist::SharedPtr msg) {inputCommandCallback(msg);}
};

TEST(VelocitySmootherTest, testfindEtaConstraint)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);

  // In range
  EXPECT_EQ(smoother->findEtaConstraint(1.0, 1.0, 1.5, -2.0), -1);
  EXPECT_EQ(smoother->findEtaConstraint(0.5, 0.55, 1.5, -2.0), -1);
  EXPECT_EQ(smoother->findEtaConstraint(0.5, 0.45, 1.5, -2.0), -1);
  // Too high
  EXPECT_EQ(smoother->findEtaConstraint(1.0, 2.0, 1.5, -2.0), 0.075);
  // Too low
  EXPECT_EQ(smoother->findEtaConstraint(1.0, 0.0, 1.5, -2.0), 0.1);

  // In a more realistic situation accelerating linear axis
  EXPECT_NEAR(smoother->findEtaConstraint(0.40, 0.50, 1.5, -2.0), 0.75, 0.001);
}

TEST(VelocitySmootherTest, testapplyConstraints)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;

  // Apply examples from testfindEtaConstraint
  // In range, so no eta or acceleration limit impact
  EXPECT_EQ(smoother->applyConstraints(1.0, 1.0, 1.5, -2.0, no_eta), 1.0);
  EXPECT_EQ(smoother->applyConstraints(0.5, 0.55, 1.5, -2.0, no_eta), 0.55);
  EXPECT_EQ(smoother->applyConstraints(0.5, 0.45, 1.5, -2.0, no_eta), 0.45);
  // Too high, without eta
  EXPECT_NEAR(smoother->applyConstraints(1.0, 2.0, 1.5, -2.0, no_eta), 1.075, 0.01);
  // Too high, with eta applied on its own axis
  EXPECT_NEAR(smoother->applyConstraints(1.0, 2.0, 1.5, -2.0, 0.075), 1.075, 0.01);
  // On another virtual axis that is OK
  EXPECT_NEAR(smoother->applyConstraints(0.5, 0.55, 1.5, -2.0, 0.075), 0.503, 0.01);

  // In a more realistic situation, applied to angular
  EXPECT_NEAR(smoother->applyConstraints(0.8, 1.0, 3.2, -3.2, 0.75), 1.075, 0.95);
}

TEST(VelocitySmootherTest, testCommandCallback)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  smoother->configure(state);
  smoother->activate(state);

  auto pub = smoother->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  pub->on_activate();
  auto msg = std::make_unique<geometry_msgs::msg::Twist>();
  msg->linear.x = 100.0;
  pub->publish(std::move(msg));
  rclcpp::spin_some(smoother->get_node_base_interface());

  EXPECT_TRUE(smoother->hasCommandMsg());
  EXPECT_EQ(smoother->lastCommandMsg()->linear.x, 100.0);
}

TEST(VelocitySmootherTest, testClosedLoopSub)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  smoother->declare_parameter("feedback", rclcpp::ParameterValue(std::string("OPEN_LOOP")));
  smoother->set_parameter(rclcpp::Parameter("feedback", std::string("CLOSED_LOOP")));
  rclcpp_lifecycle::State state;
  smoother->configure(state);
  EXPECT_TRUE(smoother->isOdomSmoother());
}

TEST(VelocitySmootherTest, testInvalidParams)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  std::vector<double> max_vels{0.0, 0.0};  // invalid size
  smoother->declare_parameter("max_velocity", rclcpp::ParameterValue(max_vels));
  rclcpp_lifecycle::State state;
  EXPECT_THROW(smoother->configure(state), std::runtime_error);
}

TEST(VelocitySmootherTest, testDynamicParameter)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  smoother->configure(state);
  smoother->activate(state);
  EXPECT_FALSE(smoother->isOdomSmoother());

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    smoother->get_node_base_interface(), smoother->get_node_topics_interface(),
    smoother->get_node_graph_interface(),
    smoother->get_node_services_interface());

  std::vector<double> max_vel{10.0, 10.0, 10.0};
  std::vector<double> min_vel{0.0, 0.0, 0.0};
  std::vector<double> max_accel{10.0, 10.0, 10.0};
  std::vector<double> min_accel{0.0, 0.0, 0.0};
  std::vector<double> deadband{0.0, 0.0, 0.0};

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("smoothing_frequency", 100.0),
      rclcpp::Parameter("feedback", std::string("CLOSED_LOOP")),
      rclcpp::Parameter("scale_velocities", true),
      rclcpp::Parameter("max_velocity", max_vel),
      rclcpp::Parameter("min_velocity", min_vel),
      rclcpp::Parameter("max_accel", max_accel),
      rclcpp::Parameter("max_decel", min_accel),
      rclcpp::Parameter("odom_topic", std::string("TEST")),
      rclcpp::Parameter("odom_duration", 2.0),
      rclcpp::Parameter("velocity_timeout", 4.0),
      rclcpp::Parameter("deadband_velocity", deadband)});

  rclcpp::spin_until_future_complete(
    smoother->get_node_base_interface(),
    results);

  EXPECT_EQ(smoother->get_parameter("smoothing_frequency").as_double(), 100.0);
  EXPECT_EQ(smoother->get_parameter("feedback").as_string(), std::string("CLOSED_LOOP"));
  EXPECT_EQ(smoother->get_parameter("scale_velocities").as_bool(), true);
  EXPECT_EQ(smoother->get_parameter("max_velocity").as_double_array(), max_vel);
  EXPECT_EQ(smoother->get_parameter("min_velocity").as_double_array(), min_vel);
  EXPECT_EQ(smoother->get_parameter("max_accel").as_double_array(), max_accel);
  EXPECT_EQ(smoother->get_parameter("max_decel").as_double_array(), min_accel);
  EXPECT_EQ(smoother->get_parameter("odom_topic").as_string(), std::string("TEST"));
  EXPECT_EQ(smoother->get_parameter("odom_duration").as_double(), 2.0);
  EXPECT_EQ(smoother->get_parameter("velocity_timeout").as_double(), 4.0);
  EXPECT_EQ(smoother->get_parameter("deadband_velocity").as_double_array(), deadband);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("feedback", std::string("OPEN_LOOP"))});
  rclcpp::spin_until_future_complete(
    smoother->get_node_base_interface(), results);
  EXPECT_EQ(smoother->get_parameter("feedback").as_string(), std::string("OPEN_LOOP"));

  // test full state after major changes
  smoother->deactivate(state);
  smoother->cleanup(state);
  smoother->shutdown(state);
  smoother.reset();
}
