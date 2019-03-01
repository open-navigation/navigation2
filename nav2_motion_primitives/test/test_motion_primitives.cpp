// Copyright (c) 2019 Intel Corporation
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <chrono>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_tasks/task_client.hpp"
#include "nav2_tasks/task_status.hpp"
#include "nav2_motion_primitives/motion_primitive.hpp"

using nav2_tasks::TaskStatus;
using nav2_motion_primitives::MotionPrimitive;
using namespace std::chrono_literals;

// A global object to initialize and shutdown ROS

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

// Let's create a motion primitive for testing the base class

using DummyPrimitiveCommand = std_msgs::msg::String;
using DummyPrimitiveResult = std_msgs::msg::String;

using DummyPrimitiveClient = nav2_tasks::TaskClient<DummyPrimitiveCommand, DummyPrimitiveResult>;

class DummyPrimitive : public MotionPrimitive<DummyPrimitiveCommand, DummyPrimitiveResult>
{
public:
  explicit DummyPrimitive(rclcpp::Node::SharedPtr & node)
  : MotionPrimitive<DummyPrimitiveCommand, DummyPrimitiveResult>(node),
    initialized_(false) {}

  ~DummyPrimitive() {}

  TaskStatus onRun(const DummyPrimitiveCommand::SharedPtr command) override
  {
    // A normal primitive would catch the command and initialize
    initialized_ = false;
    command_ = command;
    start_time_ = std::chrono::system_clock::now();

    // onRun method can have various possible outcomes (success, failure, cancelled)
    // The output is defined by the tester class on the command string.

    if (command_->data == "Testing failure on init") {
      // A real primitive we would return failed if initialization failed or command is invalid
      return TaskStatus::FAILED;
    } else if (command_->data == "Testing success") {
      initialized_ = true;
      return TaskStatus::SUCCEEDED;
    } else {
      return TaskStatus::FAILED;
    }
  }

  TaskStatus onCycleUpdate(DummyPrimitiveResult & /*result*/) override
  {
    // A normal primitive would set the robot in motion in the first call
    // and check for robot states on subsequent calls to check if the movement
    // was completed.

    if (command_->data != "Testing success" || !initialized_) {
      return TaskStatus::FAILED;
    }

    // Fake getting the robot state, calculate and send control output
    std::this_thread::sleep_for(2ms);

    // For testing, pretend the robot takes some fixed
    // amount of time to complete the motion.
    auto current_time = std::chrono::system_clock::now();
    auto motion_duration = 5s;

    if (current_time - start_time_ >= motion_duration) {
      // Movement was completed
      return TaskStatus::SUCCEEDED;
    }

    return TaskStatus::RUNNING;
  }

private:
  bool initialized_;
  std::shared_ptr<DummyPrimitiveCommand> command_;
  std::chrono::system_clock::time_point start_time_;
};

// Define a test class to hold the context for the tests

class MotionPrimitivesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("MotionPrimitivesTestNode");

    primitive_ = std::make_unique<DummyPrimitive>(node_);

    client_ = std::make_unique<DummyPrimitiveClient>(node_);

    // Launch a thread to spin the node
    spin_thread_ = new std::thread(&MotionPrimitivesTest::spin, this);
  }

  void TearDown() override
  {
    spin_thread_->join();
    delete spin_thread_;
  }

  void spin()
  {
    rclcpp::spin(node_);
  }

  void sendStringCommand(const std::string & string_command)
  {
    auto command = std::make_shared<DummyPrimitiveCommand>();
    command->data = string_command;
    client_->sendCommand(command);
  }

  TaskStatus waitForPrimitive()
  {
    auto result = std::make_shared<DummyPrimitiveResult>();

    // Loop until primitive is completed
    while (true) {
      TaskStatus status = client_->waitForResult(result, 100ms);

      if (status != TaskStatus::RUNNING) {
        return status;
      }
    }
  }


  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<DummyPrimitive> primitive_;
  std::unique_ptr<DummyPrimitiveClient> client_;
  std::thread * spin_thread_;
};

// Define the tests

TEST_F(MotionPrimitivesTest, testingCorrectSuccess)
{
  sendStringCommand("Test success");
  EXPECT_EQ(waitForPrimitive(), TaskStatus::SUCCEEDED);
}

TEST_F(MotionPrimitivesTest, testingCorrecFailure)
{
  sendStringCommand("Testing failure on init");
  EXPECT_EQ(waitForPrimitive(), TaskStatus::FAILED);

  sendStringCommand("Testing failure");
  EXPECT_EQ(waitForPrimitive(), TaskStatus::FAILED);
}

TEST_F(MotionPrimitivesTest, testCancel)
{
  sendStringCommand("Testing success");
  client_->cancel();
  EXPECT_EQ(waitForPrimitive(), TaskStatus::CANCELED);
}
