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
#include <unordered_map>
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
    publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
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

  void initialize(
    const std::vector<rclcpp::Parameter> & extra = {}, size_t count = 4,
    const std::unordered_map<std::string, joint_limits::JointLimits> & limits = {})
  {
    count_ = count;
    commands.resize(2 * count);
    states.resize(2 * count);
    steering_rates.resize(count);
    std::vector<std::string> wheels, actuators;
    std::vector<double> x, y, offsets;
    for (size_t i = 0; i < count; ++i) {
      wheels.push_back("w" + std::to_string(i));
      actuators.push_back("a" + std::to_string(i));
      x.push_back(count == 4 ? (i < 2 ? 0.13 : -0.13) :
        0.2 * std::cos(2 * M_PI * i / count));
      y.push_back(count == 4 ? (i % 2 == 0 ? 0.12 : -0.12) :
        0.15 * std::sin(2 * M_PI * i / count));
      offsets.push_back(i % 2 == 0 ? 0.045 : -0.045);
    }
    std::vector<rclcpp::Parameter> parameters{
      {"wheel_names", wheels}, {"actuator_names", actuators},
      {"module_x", x}, {"module_y", y}, {"wheel_offsets", offsets},
      {"wheel_radii", std::vector<double>(count, 0.075)},
      {"steering_continuous", std::vector<bool>(count, false)},
      {"steering_min", std::vector<double>(count, -M_PI_2)},
      {"steering_max", std::vector<double>(count, M_PI_2)},
      {"steering_velocity_limits", std::vector<double>(count, 10.0)},
      {"cmd_vel_timeout", 0.15}};
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
    options.hard_joint_limits = limits;
    options.node_options.parameter_overrides(parameters);
    ASSERT_EQ(controller->init(options), return_type::OK);
    executor.add_node(controller->get_node()->get_node_base_interface());
  }

  void bind(bool missing = false)
  {
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    // Deliberately loan in reverse module order to exercise name-based binding.
    for (int i = static_cast<int>(count_) - 1; i >= 0; --i) {
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
    for (size_t i = 0; i < count_; ++i) {
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
    double nonplanar = 0.0)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (publisher->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_GT(publisher->get_subscription_count(), 0u);
    geometry_msgs::msg::Twist command;
    command.linear.z = nonplanar;
    command.linear.x = vx;
    command.linear.y = vy;
    command.angular.z = omega;
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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
  rclcpp::executors::SingleThreadedExecutor executor;
  size_t count_ = 4;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
  std::vector<hardware_interface::StateInterface::SharedPtr> states;
  std::vector<hardware_interface::StateInterface::SharedPtr> steering_rates;
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
  for (size_t i = 0; i < count_; ++i) {
    EXPECT_NEAR(command(2 * i), 1.0, 1e-9);
    EXPECT_NEAR(command(2 * i + 1), 0.0, 1e-9);
  }
  send(0.075, 0.0, 0.0, 0.1);
  for (size_t i = 0; i < count_; ++i) {
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
  for (size_t i = 0; i < count_; ++i) {
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
  for (size_t i = 0; i < count_; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
  }
  activate();
  ASSERT_EQ(update(), return_type::OK);
  for (size_t i = 0; i < count_; ++i) {
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
  for (size_t i = 0; i < count_; ++i) {
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
  for (size_t i = 0; i < count_; ++i) {
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
  for (size_t i = 0; i < count_; ++i) {
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
  const auto result = controller->get_node()->set_parameters_atomically({{"wheel_radii",
        std::vector<double>(4, 0.2)}});
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
        for (size_t i = 0; i < count_; ++i) {
          ASSERT_TRUE(states[2 * i + 1]->set_value(command(2 * i + 1)));
        }
        ASSERT_EQ(update(), return_type::OK);
      }
      for (size_t i = 0; i < count_; ++i) {
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
  send(0.075, 0.0, 0.0, 0.3);
  EXPECT_NEAR(command(0), 1.0, 1e-9);
  EXPECT_TRUE(controller->get_node()->set_parameters_atomically(
      {{"unsupported_command_policy", "stop"}}).successful);
  ASSERT_EQ(update(), return_type::OK);
  EXPECT_DOUBLE_EQ(command(0), 0.0);
  send(0.075, 0.0, 0.0, 0.3);
  EXPECT_DOUBLE_EQ(command(0), 0.0);
  EXPECT_FALSE(controller->get_node()->set_parameters_atomically(
      {{"unsupported_command_policy", "invalid"}}).successful);
  // A rejected atomic transaction must never change the active RT policy.
  EXPECT_FALSE(controller->get_node()->set_parameters_atomically(
      {{"unsupported_command_policy", "ignore"},
        {"wheel_radii", std::vector<double>(4, 0.2)}}).successful);
  send(0.075, 0.0, 0.0, 0.3);
  EXPECT_DOUBLE_EQ(command(0), 0.0);
  send(0.0, 0.0, 0.4);
  EXPECT_NE(command(0), 0.0);
}

TEST_F(SwerveControllerTest, CrabToleratesSmallHardStopOvershootAndBoundsCommands)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  // Gazebo can report a small overshoot while settling against the mechanical stop.
  for (const double sign : {-1.0, 1.0}) {
    for (size_t i = 0; i < count_; ++i) {
      ASSERT_TRUE(states[2 * i + 1]->set_value(sign * (M_PI_2 + 5e-4)));
    }
    activate();
    send(0.0, sign * 0.2, 0.0);
    for (size_t i = 0; i < count_; ++i) {
      EXPECT_LE(std::abs(command(2 * i + 1)), M_PI_2);
      EXPECT_GT(std::abs(command(2 * i)), 2.0);
    }
    send(0.0, 0.0, 0.0);
    for (size_t i = 0; i < count_; ++i) {
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
  send(0.0, 0.2, 0.0);
  ASSERT_TRUE(states[1]->set_value(M_PI_2 + 0.01));
  EXPECT_EQ(update(), return_type::ERROR);
  for (size_t i = 0; i < count_; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
  }
}

TEST_F(SwerveControllerTest, ConfigurableCountsBindAndUpdateWithoutAllocation)
{
  // Each fixture invocation owns one controller; exercise non-four counts via scoped teardown.
  for (const size_t count : {3u, 6u}) {
    initialize({}, count);
    ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
    EXPECT_EQ(controller->command_interface_configuration().names.size(), 2 * count);
    EXPECT_EQ(controller->state_interface_configuration().names.size(), 3 * count);
    bind();
    activate();
    send(0.075);
    for (size_t i = 0; i < count; ++i) {
      EXPECT_NEAR(command(2 * i), 1.0, 1e-9);
    }
    allocations = 0;
    track_allocations = true;
    const auto result = update();
    track_allocations = false;
    EXPECT_EQ(result, return_type::OK);
    EXPECT_EQ(allocations, 0u);
    ASSERT_EQ(controller->on_deactivate(state), CallbackReturn::SUCCESS);
    activated = false;
    executor.remove_node(controller->get_node()->get_node_base_interface());
    controller->release_interfaces();
    controller.reset();
  }
}

TEST_F(SwerveControllerTest, RejectsMismatchedGeometry)
{
  initialize({{"module_x", std::vector<double>{0.1, 0.2, 0.3}}});
  EXPECT_EQ(controller->on_configure(state), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, RejectsNonfiniteGeometry)
{
  initialize({{"wheel_offsets", std::vector<double>(4,
      std::numeric_limits<double>::infinity())}});
  EXPECT_EQ(controller->on_configure(state), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, RejectsInvalidRadius)
{
  initialize({{"wheel_radii", std::vector<double>(4, 0.0)}});
  EXPECT_EQ(controller->on_configure(state), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, RejectsDegenerateGeometry)
{
  initialize({{"module_x", std::vector<double>(4, 0.0)},
      {"module_y", std::vector<double>(4, 0.0)}});
  EXPECT_EQ(controller->on_configure(state), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, UnreachableHeadingStopsAllModulesAndRecovers)
{
  initialize({{"steering_min", std::vector<double>(4, -0.2)},
      {"steering_max", std::vector<double>(4, 0.2)}});
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  EXPECT_DOUBLE_EQ(command(0), 1.0);
  send(0.0, 0.2);
  for (size_t i = 0; i < count_; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
    EXPECT_DOUBLE_EQ(command(2 * i + 1), 0.0);
  }
  send(0.075);
  EXPECT_DOUBLE_EQ(command(0), 1.0);
}

TEST_F(SwerveControllerTest, ContinuousSteeringPreservesUnwrappedFeedback)
{
  initialize({{"steering_continuous", std::vector<bool>(4, true)}});
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  for (size_t i = 0; i < count_; ++i) {
    ASSERT_TRUE(states[2 * i + 1]->set_value(4 * M_PI));
  }
  activate();
  send(0.075);
  for (size_t i = 0; i < count_; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i + 1), 4 * M_PI);
    EXPECT_NEAR(command(2 * i), 1.0, 1e-9);
  }
}

TEST_F(SwerveControllerTest, PlanarTwistsRecoverRequestedMotion)
{
  using namespace swervebot_controllers;
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  std::array<ModuleGeometry, 4> modules;
  for (size_t i = 0; i < 4; ++i) {
    modules[i] = {i < 2 ? 0.13 : -0.13, i % 2 == 0 ? 0.12 : -0.12,
      0.075, i % 2 == 0 ? 0.045 : -0.045, false, -M_PI_2, M_PI_2, 10.0};
  }
  for (const auto & motion : std::array<std::array<double, 3>, 8>{{
      {0.2, 0.0, 0.0}, {-0.2, 0.0, 0.0}, {0.0, 0.2, 0.0}, {0.0, -0.2, 0.0},
      {0.2, 0.15, 0.0}, {-0.2, -0.15, 0.0}, {0.0, 0.0, 0.4}, {0.2, 0.1, -0.3}}})
  {
    send(motion[0], motion[1], motion[2]);
    for (size_t tick = 0; tick < 40; ++tick) {
      for (size_t i = 0; i < count_; ++i) {
        const double previous = states[2 * i + 1]->get_optional().value();
        EXPECT_LE(std::abs(command(2 * i + 1) - previous), 0.1 + 1e-12);
        ASSERT_TRUE(states[2 * i + 1]->set_value(command(2 * i + 1)));
      }
      ASSERT_EQ(update(), return_type::OK);
    }
    std::array<ModuleState, 4> feedback;
    for (size_t i = 0; i < 4; ++i) {
      feedback[i] = {command(2 * i + 1), command(2 * i), 0.0};
    }
    std::array<double, 3> recovered{};
    ASSERT_TRUE(estimate_chassis_velocity(modules.data(), feedback.data(), 4, recovered));
    for (size_t axis = 0; axis < 3; ++axis) {
      EXPECT_NEAR(recovered[axis], motion[axis], 1e-9);
    }
  }
}

TEST_F(SwerveControllerTest, NonfiniteNonplanarFieldStopsEvenUnderIgnorePolicy)
{
  initialize({{"unsupported_command_policy", "ignore"}});
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  send(0.075, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN());
  for (size_t i = 0; i < count_; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
                                                                             }
}

TEST(SwerveKinematicsTest, AsymmetricLayoutsRecoverMotionWithSteeringRates)
{
  using namespace swervebot_controllers;
  for (size_t count : {3u, 4u, 6u}) {
    std::vector<ModuleGeometry> modules;
    std::vector<ModuleState> states;
    const double vx = -0.2, vy = 0.13, omega = -0.4;
    for (size_t i = 0; i < count; ++i) {
      const double phase = 2 * M_PI * i / count;
      ModuleGeometry module{0.3 * std::cos(phase) + 0.03, 0.2 * std::sin(phase),
        0.05 + 0.01 * i, 0.01 * static_cast<double>(i) - 0.02, true, 0.0, 0.0, 10.0};
      const double px = vx - omega * module.y, py = vy + omega * module.x;
      const double angle = std::atan2(py, px), rate = 0.1 * i;
      modules.push_back(module);
      states.push_back({angle,
          (px * std::cos(angle) + py * std::sin(angle) - module.offset * (omega + rate)) /
          module.radius, rate});
    }
    std::array<double, 3> recovered{};
    ASSERT_TRUE(estimate_chassis_velocity(modules.data(), states.data(), count, recovered));
    EXPECT_NEAR(recovered[0], vx, 1e-10);
    EXPECT_NEAR(recovered[1], vy, 1e-10);
    EXPECT_NEAR(recovered[2], omega, 1e-10);
  }
}

TEST(SwerveKinematicsTest, HeadingSelectionUsesClosestFeasibleEquivalent)
{
  using namespace swervebot_controllers;
  ModuleGeometry module{0.1, 0.1, 0.075, 0.0, false, 0.0, 2 * M_PI, 10.0};
  double target = 0.0;
  ASSERT_TRUE(select_heading(module, -0.2, 6.0, target));
  EXPECT_NEAR(target, 2 * M_PI - 0.2, 1e-12);
  module.minimum = -0.1;
  module.maximum = 0.1;
  EXPECT_FALSE(select_heading(module, M_PI_2, 0.0, target));
  module.continuous = true;
  ASSERT_TRUE(select_heading(module, 0.1, 10 * M_PI, target));
  EXPECT_NEAR(target, 10 * M_PI + 0.1, 1e-12);
}

TEST_F(SwerveControllerTest, UrdfLimitsNarrowTravelAndSlewRate)
{
  joint_limits::JointLimits limit;
  limit.has_position_limits = true;
  limit.min_position = -0.1;
  limit.max_position = 0.1;
  limit.has_velocity_limits = true;
  limit.max_velocity = 1.0;
  initialize({}, 4, {{"a0", limit}});
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.2, 0.01);
  EXPECT_NEAR(command(1), 0.01, 1e-8);
  send(0.0, 0.2);
  for (size_t i = 0; i < count_; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
                                                                             }
}

TEST_F(SwerveControllerTest, RejectsDisjointUrdfTravel)
{
  joint_limits::JointLimits limit;
  limit.has_position_limits = true;
  limit.min_position = 2.0;
  limit.max_position = 3.0;
  initialize({}, 4, {{"a0", limit}});
  EXPECT_EQ(controller->on_configure(state), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, RejectsContinuousFlagOnBoundedUrdfJoint)
{
  joint_limits::JointLimits limit;
  limit.has_position_limits = true;
  limit.min_position = -M_PI_2;
  limit.max_position = M_PI_2;
  initialize({{"steering_continuous", std::vector<bool>(4, true)}}, 4, {{"a0", limit}});
  EXPECT_EQ(controller->on_configure(state), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, SteeringCommandsRespectConfiguredRate)
{
  initialize({{"steering_velocity_limits", std::vector<double>(4, 1.0)}});
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.2, 0.0, 0.3);
  for (size_t i = 0; i < count_; ++i) {
    EXPECT_LE(std::abs(command(2 * i + 1)), 0.01);
  }
}

TEST_F(SwerveControllerTest, OverflowingTargetsStopWithoutNonfiniteHardwareWrites)
{
  initialize();
  ASSERT_EQ(controller->on_configure(state), CallbackReturn::SUCCESS);
  bind();
  activate();
  send(0.075);
  // Publish without send(), since this deliberately causes update() to return an error.
  geometry_msgs::msg::Twist message;
  message.linear.x = std::numeric_limits<double>::max();
  for (size_t i = 0; i < 10; ++i) {
    publisher->publish(message);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  executor.spin_some();
  allocations = 0;
  track_allocations = true;
  const auto result = update();
  track_allocations = false;
  EXPECT_EQ(result, return_type::ERROR);
  EXPECT_EQ(allocations, 0u);
  for (size_t i = 0; i < count_; ++i) {
    EXPECT_DOUBLE_EQ(command(2 * i), 0.0);
    EXPECT_TRUE(std::isfinite(command(2 * i + 1)));
  }
}
