#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <limits>
#include <memory>
#include <new>
#include <shared_mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "swervebot_controllers/swerve_controller.hpp"
#include "swervebot_controllers/swerve_kinematics.hpp"

namespace
{
thread_local bool track_allocations = false;
thread_local size_t allocations = 0;

// Hold a handle from a different thread to model real contention without recursive-lock UB.
class BusyInterface
{
public:
  explicit BusyInterface(std::shared_mutex & mutex)
  {
    auto release = release_.get_future();
    thread_ = std::thread([this, &mutex, release = std::move(release)]() {
          std::unique_lock<std::shared_mutex> lock(mutex);
          acquired_.set_value();
          release.wait();
    });
    acquired_.get_future().wait();
  }
  ~BusyInterface()
  {
    release_.set_value();
    thread_.join();
  }

private:
  std::promise<void> acquired_, release_;
  std::thread thread_;
};
}

// Count allocations in the update thread, excluding ROS executor/DDS background threads.
void * operator new(std::size_t size)
{
  if (track_allocations) {++allocations;}
  if (void * memory = std::malloc(size == 0 ? 1 : size)) {return memory;}
  throw std::bad_alloc();
}
void * operator new[](std::size_t size) {return ::operator new(size);}
void operator delete(void * memory) noexcept {std::free(memory);}
void operator delete[](void * memory) noexcept {std::free(memory);}
void operator delete(void * memory, std::size_t) noexcept {std::free(memory);}
void operator delete[](void * memory, std::size_t) noexcept {std::free(memory);}
void * operator new(std::size_t size, std::align_val_t alignment)
{
  if (track_allocations) {++allocations;}
  void * memory = nullptr;
  if (posix_memalign(&memory, static_cast<size_t>(alignment), size == 0 ? 1 : size) != 0) {
    throw std::bad_alloc();
  }
  return memory;
}
void * operator new[](std::size_t size, std::align_val_t alignment)
{
  return ::operator new(size, alignment);
}
void operator delete(void * memory, std::align_val_t) noexcept {std::free(memory);}
void operator delete[](void * memory, std::align_val_t) noexcept {std::free(memory);}
void operator delete(void * memory, std::size_t, std::align_val_t) noexcept {std::free(memory);}
void operator delete[](void * memory, std::size_t, std::align_val_t) noexcept {std::free(memory);}

using controller_interface::CallbackReturn;
using controller_interface::return_type;
using swervebot_controllers::SwerveController;

class SwerveControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("command_source");
    publisher = node->create_publisher<swervebot_msgs::msg::RobotCommand>("/swervebot/cmd_raw", 1);
    executor.add_node(node);
  }
  void TearDown() override
  {
    if (controller) {
      if (activated) {EXPECT_EQ(controller->on_deactivate(state), CallbackReturn::SUCCESS);}
      executor.remove_node(controller->get_node()->get_node_base_interface());
      controller->release_interfaces();
      controller.reset();
    }
    executor.remove_node(node);
  }

  void initialize(const std::vector<rclcpp::Parameter> & extra = {})
  {
    std::vector<rclcpp::Parameter> parameters{
      {"wheel_names", std::vector<std::string>{"w0", "w1", "w2", "w3"}},
      {"actuator_names", std::vector<std::string>{"a0", "a1", "a2", "a3"}},
      {"wheel_base", 0.26}, {"track_width", 0.24}, {"wheel_offset", 0.045},
      {"wheel_radius", 0.075}, {"cmd_vel_timeout", 0.15}};
    for (const auto & parameter : extra) {
      const auto old = std::find_if(parameters.begin(), parameters.end(), [&](const auto & item) {
            return item.get_name() == parameter.get_name();
      });
      if (old != parameters.end()) {*old = parameter;} else {parameters.push_back(parameter);}
    }
    controller = std::make_unique<SwerveController>();
    controller_interface::ControllerInterfaceParams options;
    options.controller_name = "swerve_test";
    options.controller_manager_update_rate = 100;
    options.node_options.parameter_overrides(parameters);
    ASSERT_EQ(controller->init(options), return_type::OK);
    executor.add_node(controller->get_node()->get_node_base_interface());
  }

  void bind(bool missing = false)
  {
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    // Deliberately loan in reverse module order to exercise name-based binding.
    for (int i = 3; i >= 0; --i) {
      for (int kind = 0; kind < 2; ++kind) {
        const std::string joint = (kind == 0 ? "w" : "a") + std::to_string(i);
        const std::string interface = kind == 0 ? "velocity" : "position";
        const size_t index = 2 * i + kind;
        commands[index] = std::make_shared<hardware_interface::CommandInterface>(
          joint, interface, "double", "0");
        states[index] = std::make_shared<hardware_interface::StateInterface>(
          joint, interface, "double", "0");
        if (!(missing && i == 2 && kind == 1)) {command_loans.emplace_back(commands[index]);}
        state_loans.emplace_back(states[index]);
      }
    }
    for (size_t i = 0; i < 4; ++i) {
      steering_rates[i] = std::make_shared<hardware_interface::StateInterface>(
        "a" + std::to_string(i), "velocity", "double", "0");
      state_loans.emplace_back(steering_rates[i]);
    }
    controller->assign_interfaces(std::move(command_loans), std::move(state_loans));
  }

  void activate()
  {
    ASSERT_EQ(controller->on_activate(state), CallbackReturn::SUCCESS);
    activated = true;
  }

  return_type update()
  {
    return controller->update(rclcpp::Time(1, 0), rclcpp::Duration::from_seconds(0.01));
  }

  void send(
    double vx, double vy = 0.0, double omega = 0.0,
    uint8_t mode = swervebot_msgs::msg::RobotCommand::ACKERMANN)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (publisher->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_GT(publisher->get_subscription_count(), 0u);
    swervebot_msgs::msg::RobotCommand command;
    command.mode = mode;
    command.twist.linear.x = vx;
    command.twist.linear.y = vy;
    command.twist.angular.z = omega;
    // Spin repeatedly so delivery is observed before inspecting the command interfaces.
    for (int i = 0; i < 10; ++i) {
      publisher->publish(command);
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    executor.spin_some();
    ASSERT_EQ(update(), return_type::OK);
  }

  double command(size_t index) {return commands[index]->get_optional().value();}
  rclcpp_lifecycle::State state;
  bool activated = false;
  std::unique_ptr<SwerveController> controller;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<swervebot_msgs::msg::RobotCommand>::SharedPtr publisher;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::array<hardware_interface::CommandInterface::SharedPtr, 8> commands;
  std::array<hardware_interface::StateInterface::SharedPtr, 8> states;
  std::array<hardware_interface::StateInterface::SharedPtr, 4> steering_rates;
};

TEST_F(SwerveControllerTest, BindsInterfacesAndWritesCommands)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  EXPECT_EQ(controller->command_interface_configuration().names.size(), 8u);
  EXPECT_EQ(controller->state_interface_configuration().names.size(), 12u);
  bind();
  activate();
  send(0.075);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(command(2 * i), 1.0, 1e-9);
    EXPECT_NEAR(command(2 * i + 1), 0.0, 1e-9);
  }
  send(0.075, 0.075);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
    EXPECT_DOUBLE_EQ(command(2 * i + 1), 0.0);
  }
}

TEST_F(SwerveControllerTest, StopsOnTimeoutAndHoldsSteering)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  std::this_thread::sleep_for(std::chrono::milliseconds(170));
  ASSERT_EQ(update(), return_type::OK);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
    EXPECT_DOUBLE_EQ(command(2 * i + 1), 0.0);
  }
}

TEST_F(SwerveControllerTest, DeactivationStopsAndReactivationRequiresFreshCommand)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  ASSERT_EQ(controller->on_deactivate(state), CallbackReturn::SUCCESS);
  activated = false;
  send(0.15);  // Commands received while inactive must also be discarded.
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
                                                                        }
  activate();
  ASSERT_EQ(update(), return_type::OK);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
                                                                        }
  send(0.075);
  EXPECT_DOUBLE_EQ(command(0), 1.0);
}

TEST_F(SwerveControllerTest, RejectsNonfiniteCommandAndStops)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  send(std::numeric_limits<double>::quiet_NaN());
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
                                                                        }
  send(0.0);
  EXPECT_DOUBLE_EQ(command(1), 0.0);
}

TEST_F(SwerveControllerTest, MissingInterfaceFailsActivationAndCanRetry)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind(true);
  EXPECT_EQ(controller->on_activate(state), CallbackReturn::ERROR);
  controller->release_interfaces();
  bind();
  activate();
  EXPECT_EQ(update(), return_type::OK);
}

TEST_F(SwerveControllerTest, NonfiniteFeedbackStopsEveryWheel)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  ASSERT_TRUE(states[5]->set_value(std::numeric_limits<double>::infinity()));
  EXPECT_EQ(update(), return_type::ERROR);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
                                                                        }
}

TEST_F(SwerveControllerTest, UnavailableFeedbackReturnsErrorWithoutThrowing)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  BusyInterface busy(states[3]->get_mutex());
  EXPECT_EQ(update(), return_type::ERROR);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
                                                                        }
}

TEST_F(SwerveControllerTest, FailedCommandWriteStopsRemainingWheels)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  // Simulate a busy command interface; the controller must still stop the other wheels.
  BusyInterface busy(commands[2]->get_mutex());
  EXPECT_EQ(update(), return_type::ERROR);
  for (size_t i : {0u, 4u, 6u}) {
    EXPECT_DOUBLE_EQ(command(i), 0.0);
                                                                   }
}

TEST_F(SwerveControllerTest, RejectsDuplicateNames)
{
  initialize({{"wheel_names", std::vector<std::string>{"w0", "w0", "w2", "w3"}}});
  EXPECT_EQ(controller->on_configure(state), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, RejectsWrongJointCount)
{
  initialize({{"actuator_names", std::vector<std::string>{"a0", "a1", "a2"}}});
  EXPECT_EQ(controller->on_configure(state), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, RejectsUnsupportedInterfaces)
{
  initialize({{"wheel_command_interfaces", std::vector<std::string>{"effort"}}});
  EXPECT_EQ(controller->on_configure(state), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, ParametersAreImmutable)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  const auto result = controller->get_node()->set_parameters_atomically({{"wheel_radius", 0.2}});
  EXPECT_FALSE(result.successful);
}

TEST_F(SwerveControllerTest, WarmUpdateDoesNotAllocate)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  ASSERT_EQ(update(), return_type::OK);
  allocations = 0;
  track_allocations = true;
  auto result = return_type::OK;
  for (size_t i = 0; i < 100; ++i) {
    result = update();
                                                      }
  track_allocations = false;
  EXPECT_EQ(result, return_type::OK);
  EXPECT_EQ(allocations, 0u);
}

TEST_F(SwerveControllerTest, InvalidInitialFeedbackPreventsActivation)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  ASSERT_TRUE(states[1]->set_value(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(controller->on_activate(state), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, ThrowingLimiterReturnsErrorAndStopsOtherWheels)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  commands[2]->set_on_set_command_limiter([](double, bool &) -> double {
      throw std::runtime_error("Hardware limit failure");
  });
  EXPECT_EQ(update(), return_type::ERROR);
  for (size_t i : {0u, 4u, 6u}) {
    EXPECT_DOUBLE_EQ(command(i), 0.0);
                                                                   }
  commands[2]->set_on_set_command_limiter([](double value, bool &) {return value;});
}

TEST_F(SwerveControllerTest, CoordinatedArcsRespectOffsetsAndReverse)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  for (const double vx : {0.3, -0.3}) {
    for (const double omega : {0.4, -0.4}) {
      send(vx, 0.0, omega);
      for (size_t tick = 0; tick < 35; ++tick) {
        for (size_t i = 0; i < 4; ++i) {
          ASSERT_TRUE(states[2 * i + 1]->set_value(command(2 * i + 1)));
        }
        ASSERT_EQ(update(), return_type::OK);
      }
      for (size_t i = 0; i < 4; ++i) {
        const double x = i < 2 ? 0.13 : -0.13;
        const double y = i % 2 == 0 ? 0.12 : -0.12;
        const double offset = i % 2 == 0 ? 0.045 : -0.045;
        const double angle = command(2 * i + 1);
        EXPECT_LE(std::abs(angle), M_PI_2);
        // Once steering reaches its target, each contact rolls around the common turn center.
        ASSERT_TRUE(states[2 * i + 1]->set_value(angle));
        ASSERT_TRUE(steering_rates[i]->set_value(0.2));
        const double expected = ((vx - omega * y) * std::cos(angle) +
          omega * x * std::sin(angle) - offset * (omega + 0.2)) / 0.075;
        ASSERT_EQ(update(), return_type::OK);
        EXPECT_NEAR(command(2 * i), expected, 1e-10);
      }
      EXPECT_NEAR(command(1), -command(5), 1e-10);
      EXPECT_NEAR(command(3), -command(7), 1e-10);
    }
  }
}

TEST_F(SwerveControllerTest, LivePolicyChangesRequireNewCommands)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  EXPECT_TRUE(controller->get_node()->set_parameters_atomically(
      {{"unsupported_command_policy", "ignore"}}).successful);
  ASSERT_EQ(update(), return_type::OK);
  EXPECT_DOUBLE_EQ(command(0), 0.0);
  send(0.075, 0.3);
  EXPECT_NEAR(command(0), 1.0, 1e-9);
  EXPECT_TRUE(controller->get_node()->set_parameters_atomically(
      {{"unsupported_command_policy", "stop"}}).successful);
  ASSERT_EQ(update(), return_type::OK);
  EXPECT_DOUBLE_EQ(command(0), 0.0);
  send(0.075, 0.3);
  EXPECT_DOUBLE_EQ(command(0), 0.0);
  EXPECT_FALSE(controller->get_node()->set_parameters_atomically(
      {{"unsupported_command_policy", "invalid"}}).successful);
  // A rejected atomic transaction must never change the active RT policy.
  EXPECT_FALSE(controller->get_node()->set_parameters_atomically(
      {{"unsupported_command_policy", "ignore"}, {"wheel_radius", 0.2}}).successful);
  send(0.075, 0.3);
  EXPECT_DOUBLE_EQ(command(0), 0.0);
  send(0.0, 0.0, 0.4);
  EXPECT_DOUBLE_EQ(command(0), 0.0);
}

TEST_F(SwerveControllerTest, IgnorePolicyStillStopsYawOnlyAndNonfiniteCommands)
{
  initialize({{"unsupported_command_policy", "ignore"}});
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.0, 1.0, 0.5);
  EXPECT_DOUBLE_EQ(command(0), 0.0);
  send(0.075, std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(command(0), 0.0);
}

TEST(SwerveKinematicsTest, RecoversTwistWithOffsetAndSteeringMotion)
{
  const double x[4]{0.13, 0.13, -0.13, -0.13};
  const double y[4]{0.12, -0.12, 0.12, -0.12};
  for (const double vx : {0.3, -0.3}) {
    for (const double yaw : {0.4, -0.4}) {
      std::array<double, 4> angles{}, rates{}, steering{};
      for (size_t i = 0; i < 4; ++i) {
        const double px = vx - yaw * y[i], py = yaw * x[i];
        angles[i] = std::atan2(py, px);
        steering[i] = 0.1 * static_cast<double>(i);
        const double offset = i % 2 == 0 ? 0.045 : -0.045;
        rates[i] = (px * std::cos(angles[i]) + py * std::sin(angles[i]) -
          offset * (yaw + steering[i])) / 0.075;
      }
      std::array<double, 3> recovered{};
      ASSERT_TRUE(swervebot_controllers::estimate_chassis_velocity(
        x, y, 0.075, 0.045, angles, rates, steering, recovered));
      EXPECT_NEAR(recovered[0], vx, 1e-10);
      EXPECT_NEAR(recovered[1], 0.0, 1e-10);
      EXPECT_NEAR(recovered[2], yaw, 1e-10);
    }
  }
}

TEST_F(SwerveControllerTest, SteeringCommandsRespectRateLimit)
{
  initialize({{"max_steering_velocity", 1.0}});
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.2, 0.0, 0.3);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_LE(std::abs(command(2 * i + 1)), 0.01 + 1e-12);
                                                                                        }
}

TEST_F(SwerveControllerTest, ModesRecoverRequestedMotionAfterAlignment)
{
  using Message = swervebot_msgs::msg::RobotCommand;
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  const double x[4]{0.13, 0.13, -0.13, -0.13};
  const double y[4]{0.12, -0.12, 0.12, -0.12};
  // Deliberately change modes without resetting steering to cover transitions as well.
  for (const auto & motion : std::array<std::array<double, 4>, 10>{{
      {0.2, 0.0, 0.0, Message::CRAB_WALK}, {-0.2, 0.0, 0.0, Message::CRAB_WALK},
      {0.0, 0.2, 0.0, Message::CRAB_WALK}, {0.0, -0.2, 0.0, Message::CRAB_WALK},
      {0.2, 0.15, 0.0, Message::CRAB_WALK}, {-0.2, -0.15, 0.0, Message::CRAB_WALK},
      {0.0, 0.0, 0.4, Message::ZERO_TURN}, {0.0, 0.0, -0.4, Message::ZERO_TURN},
      {0.2, 0.0, 0.3, Message::ACKERMANN}, {0.0, 0.15, 0.0, Message::CRAB_WALK}}})
  {
    const auto mode = static_cast<uint8_t>(motion[3]);
    // Forbidden planar components must be discarded even under the default stop policy.
    send(mode == Message::ZERO_TURN ? 0.7 : motion[0],
      mode == Message::ZERO_TURN ? 0.6 : motion[1],
      mode == Message::CRAB_WALK ? 0.8 : motion[2], mode);
    for (size_t tick = 0; tick < 40; ++tick) {
      for (size_t i = 0; i < 4; ++i) {
        const double previous = states[2 * i + 1]->get_optional().value();
        EXPECT_LE(std::abs(command(2 * i + 1) - previous), 0.1 + 1e-12);
        ASSERT_TRUE(states[2 * i + 1]->set_value(command(2 * i + 1)));
      }
      ASSERT_EQ(update(), return_type::OK);
    }
    std::array<double, 4> angles{}, rates{}, steering{};
    for (size_t i = 0; i < 4; ++i) {
      angles[i] = command(2 * i + 1);
      rates[i] = command(2 * i);
      EXPECT_LE(std::abs(angles[i]), M_PI_2);
    }
    std::array<double, 3> recovered{};
    ASSERT_TRUE(swervebot_controllers::estimate_chassis_velocity(
      x, y, 0.075, 0.045, angles, rates, steering, recovered));
    for (size_t axis = 0; axis < 3; ++axis) {
      EXPECT_NEAR(recovered[axis], motion[axis], 1e-9);
    }
    allocations = 0;
    track_allocations = true;
    const auto result = update();
    track_allocations = false;
    EXPECT_EQ(result, return_type::OK);
    EXPECT_EQ(allocations, 0u);
  }
}

TEST_F(SwerveControllerTest, InvalidModesAndNonfiniteDiscardedFieldsAlwaysStop)
{
  using Message = swervebot_msgs::msg::RobotCommand;
  initialize({{"unsupported_command_policy", "ignore"}});
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  for (const auto mode : {Message::CRAB_WALK, Message::ZERO_TURN}) {
    send(0.2, 0.1, 0.4, mode);
    send(0.2, 0.1, 0.4, 255);
    for (size_t i = 0; i < 4; ++i) {
      EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
    }
    send(0.2, 0.1, std::numeric_limits<double>::quiet_NaN(), mode);
    for (size_t i = 0; i < 4; ++i) {
      EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
    }
    send(std::numeric_limits<double>::infinity(), 0.1, 0.4, mode);
    for (size_t i = 0; i < 4; ++i) {
      EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
    }
    send(0.2, 0.1, 0.4, mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(170));
    ASSERT_EQ(update(), return_type::OK);
    for (size_t i = 0; i < 4; ++i) {
      EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
      EXPECT_DOUBLE_EQ(command(2 * i + 1), states[2 * i + 1]->get_optional().value());
    }
  }
}

TEST_F(SwerveControllerTest, CrabToleratesSmallHardStopOvershootAndBoundsCommands)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  // Gazebo can report a small overshoot while settling against the mechanical stop.
  for (const double sign : {-1.0, 1.0}) {
    for (size_t i = 0; i < 4; ++i) {
      ASSERT_TRUE(states[2 * i + 1]->set_value(sign * (M_PI_2 + 5e-4)));
    }
    activate();
    send(0.0, sign * 0.2, 0.0, swervebot_msgs::msg::RobotCommand::CRAB_WALK);
    for (size_t i = 0; i < 4; ++i) {
      EXPECT_LE(std::abs(command(2 * i + 1)), M_PI_2);
      EXPECT_GT(std::abs(command(2 * i)), 2.0);
    }
    send(0.0, 0.0, 0.0, swervebot_msgs::msg::RobotCommand::CRAB_WALK);
    for (size_t i = 0; i < 4; ++i) {
      EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
      EXPECT_LE(std::abs(command(2 * i + 1)), M_PI_2);
    }
    ASSERT_EQ(controller->on_deactivate(state), CallbackReturn::SUCCESS);
    activated = false;
  }
}

TEST_F(SwerveControllerTest, RejectsSteeringBeyondHardStopTolerance)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.0, 0.2, 0.0, swervebot_msgs::msg::RobotCommand::CRAB_WALK);
  ASSERT_TRUE(states[1]->set_value(M_PI_2 + 0.01));
  EXPECT_EQ(update(), return_type::ERROR);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
                                                                        }
}
