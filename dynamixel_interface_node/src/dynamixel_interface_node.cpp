#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <map>
#include <mutex>
// #include <cmath>
#include <chrono>
#include <thread>
//  #include <random>
///////////////////////
#include <cstdio>
#include <memory>
#include <string>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_interface_node.hpp"

#include <limits>

const int32_t INVALID_POSITION = std::numeric_limits<int32_t>::min(); // this is for signaling the error in reading data from dynamixels
const int16_t INVALID_LOAD = std::numeric_limits<int16_t>::min();     // this is for signaling the error in reading data from dynamixels

// Default serial settings    //default BAUDRATE is 1e6
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

// Logs
#define LOG_NAME "dynamixel_interface_node"
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;

class DynamixelMotor
{
public:
  // Constructor
  DynamixelMotor(uint8_t id) : id_(id)
  {

    initializeMotor(); // Call the initialization method in the constructor

    // test feature, setting homingOffset to zero
    setProfileAcceleration(0);
  }

  void initializeMotor()
  {
    // Set Extended Position Control Mode using the new setter
    setControlMode(EXTENDED_POSITION_CONTROL_MODE);

    // Set Profile Velocity using the setter function
    setProfileVelocity(profile_velocity_);

    // Set Goal Velocity using the setter function
    setGoalVelocity(goal_velocity_);

    // Set Profile Acceleration using the setter function
    setProfileAcceleration(profile_acceleration_);

    // Enable Torque using the new setter
    setTorqueEnable(true);

    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Connection initialized for motor ID %d", id_);
  }

  ~DynamixelMotor()
  {
    // Disable Torque using the new setter
    setTorqueEnable(false);
    // Remove the motor ID from the map
    auto it = motors_.find(id_);
    if (it != motors_.end())
    {
      motors_.erase(it);
      RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Motor ID %d removed from motors map.", id_);
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger(LOG_NAME), "Motor ID %d not found in motors map during destruction.", id_);
    }
  }

  // Method to initialize the motor parameters
  void initialize(uint32_t profile_velocity, uint32_t profile_acceleration, int32_t goal_velocity,
                  int32_t min_position_limit, int32_t max_position_limit, int32_t max_abs_Velocity,
                  int32_t position_offset, double factor, int32_t position, int16_t load_limit)
  {
    profile_velocity_ = profile_velocity;
    profile_acceleration_ = profile_acceleration;
    goal_velocity_ = goal_velocity;
    min_position_limit_ = min_position_limit;
    max_position_limit_ = max_position_limit;
    max_abs_Velocity_ = max_abs_Velocity;
    position_offset_ = position_offset;
    factor_ = factor;
    position_ = position;
    load_limit_ = load_limit;

    // Set Profile Velocity using the setter function
    // setProfileVelocity(profile_velocity_);

    initializeMotor();
  }

  void setControlMode(uint8_t mode)
  {
    int result = setValuesToDynamixel(mode, ADDR_OPERATING_MODE);
    if (result == 0)
    {
      RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Control Mode set to %d for motor ID %d", mode, id_);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to set Control Mode for motor ID %d", id_);
    }
  }

  void setTorqueEnable(bool enable)
  {
    int result = setValuesToDynamixel(enable ? 1 : 0, ADDR_TORQUE_ENABLE);
    if (result == 0)
    {
      RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Torque %s for motor ID %d", enable ? "enabled" : "disabled", id_);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to %s Torque for motor ID %d", enable ? "enable" : "disable", id_);
    }
  }

  // Setters for motor parameters with the specified logic
  bool setProfileVelocity(int32_t velocity)
  {
    int32_t adjusted_velocity = adjustVelocity(velocity);
    if (setValuesToDynamixel(adjusted_velocity, ADDR_PROFILE_VELOCITY) == 0)
    {
      profile_velocity_ = adjusted_velocity;
      return true;
    }
    return false;
  }

  bool setProfileAcceleration(int32_t acceleration)
  {
    if (setValuesToDynamixel(acceleration, ADDR_PROFILE_ACCELERATION) == 0)
    {
      profile_acceleration_ = acceleration;
      return true;
    }
    return false;
  }

  bool setHomingOffset(int32_t offset)
  {
    if (setValuesToDynamixel(offset, ADDR_HOMING_OFFSET) == 0)
    {
      // profile_acceleration_  = acceleration;
      return true;
    }
    return false;
  }

  bool setGoalVelocity(int32_t velocity)
  {
    int32_t adjusted_velocity = adjustVelocity(velocity);
    if (setValuesToDynamixel(adjusted_velocity, ADDR_GOAL_VELOCITY) == 0)
    {
      goal_velocity_ = adjusted_velocity;
      return true;
    }
    return false;
  }

  bool setPosition(int32_t position)
  {
    int32_t adjusted_position = adjustPosition(position);
    if (setValuesToDynamixel(adjusted_position, ADDR_GOAL_POSITION) == 0)
    {
      motorLoadLimit();
      position_ = adjusted_position;
      return true;
    }
    return false;
  }

  void motorLoadLimit()
  {
    while (true)
    {
      // Read the initial motor position
      int32_t prevPosition = getPosition();

      // Pause for 0.1 seconds
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // Read the motor position again
      int32_t currentPosition = getPosition();

      // Read the motor load
      int16_t load = getLoad();

      // Check if load is greater than 400 or if positions are almost equal
      if (load != INVALID_LOAD && std::abs(load) > load_limit_)
      {
        setPosition(prevPosition); // Safety mechanism
        // RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to %s Torque for motor ID %d", enable ? "enable" : "disable", id_);
        RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Safety mechanism activated for one of the motors");
        break; // Exit the loop
      }

      if (std::abs(currentPosition - prevPosition) <= 0.1)
      {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Motor has minimal movement, stopping the check.");
        break; // Exit the loop
      }
    }
  }

  int32_t getPosition()
  {
    int32_t real_position = getValuesFromDynamixel();
    if (real_position != INVALID_POSITION)
    {
      // Adjust the position only once
      position_ = static_cast<int32_t>((real_position / factor_) - position_offset_);
    }
    else
    {
      initializeMotor();
      position_ = INVALID_POSITION;
    }
    return position_;
  }

  int16_t getLoad()
  {
    int16_t present_load_ = getLoadFromDynamixel();
    return present_load_;
  }

  // Static methods to manage motors
  static DynamixelMotor &getMotor(uint8_t id)
  {
    auto it = motors_.find(id);
    if (it != motors_.end())
    {
      return *(it->second); // Dereference the unique_ptr to get the motor object
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Motor with ID %d not found!", id);
      throw std::runtime_error("Motor not found");
    }
  }

  static void addMotor(uint8_t id, std::unique_ptr<DynamixelMotor> motor)
  {
    motors_.emplace(id, std::move(motor));
  }

  static std::vector<uint8_t> getMotorIDs()
  {
    std::vector<uint8_t> ids;
    for (const auto &motor_pair : motors_)
    {
      ids.push_back(motor_pair.first);
    }
    return ids;
  }

private:
  uint8_t id_;
  uint32_t profile_velocity_ = 7;
  uint32_t profile_acceleration_ = 1;
  int32_t goal_velocity_ = 5;
  int32_t min_position_limit_ = -90;
  int32_t max_position_limit_ = 90;
  int32_t max_abs_Velocity_ = 10;
  int32_t position_offset_ = 0;
  double factor_ = 1 / 0.087891;
  int32_t position_ = 0;
  int16_t load_limit_ = 0;
  int16_t present_load_ = 0;

  // Static map to hold all motors by their ID
  static std::map<uint8_t, std::unique_ptr<DynamixelMotor>> motors_;
  // Helper function to adjust velocity based on the max_abs_Velocity and factor
  int32_t adjustVelocity(int32_t velocity)
  {
    if (std::abs(velocity) > max_abs_Velocity_)
    {
      velocity = static_cast<int32_t>(std::copysign(max_abs_Velocity_, velocity));
    }
    return static_cast<int32_t>(velocity / 0.229); // * factor_ // not a good practice to change it like this.
  }

  // Helper function to adjust position based on min/max position limits and factor
  int32_t adjustPosition(int32_t position)
  {
    position += position_offset_;
    position = std::max(min_position_limit_, std::min(position, max_position_limit_));
    return static_cast<int32_t>(position * factor_);
  }

  // Placeholder for setting values to Dynamixel, returns 0 on success, other values on failure
  int setValuesToDynamixel(int32_t value, uint16_t address)
  {
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    // Depending on the address, choose the correct write method
    if (address == ADDR_OPERATING_MODE || address == ADDR_TORQUE_ENABLE || address == EXTENDED_POSITION_CONTROL_MODE)
    {
      // Writing 1 byte (e.g., operating mode, torque enable)
      dxl_comm_result = packetHandler->write1ByteTxRx(
          portHandler, id_, address, static_cast<uint8_t>(value), &dxl_error);
    }
    else if (address == ADDR_GOAL_POSITION || address == ADDR_PRESENT_POSITION || address == ADDR_PROFILE_VELOCITY || address == ADDR_GOAL_VELOCITY)
    {
      // Writing 4 bytes (e.g., goal position, profile velocity, goal velocity)
      dxl_comm_result = packetHandler->write4ByteTxRx(
          portHandler, id_, address, static_cast<uint32_t>(value), &dxl_error);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Unknown address %d provided for motor with id %d.", address, id_);
      return -1; // Failure
    }

    // Check for communication success
    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to write value %d to address %d on motor with id %d.", value, address, id_);
      handleCommResult(dxl_comm_result, id_);
      return -1; // Failure
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Succeeded to write value %d to address %d on motor with id %d.", value, address, id_);
      return 0; // Success
    }
  }

  void handleCommResult(int comm_result, uint8_t dxl_id)
  {
    switch (comm_result)
    {
    case COMM_SUCCESS:
      RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Communication successful for motor ID %d", dxl_id);
      break;
    case COMM_PORT_BUSY:
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Port is busy for motor ID %d", dxl_id);
      break;
    case COMM_TX_FAIL:
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to transmit instruction packet for motor ID %d", dxl_id);
      break;
    case COMM_RX_FAIL:
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to receive status packet for motor ID %d", dxl_id);
      break;
    case COMM_TX_ERROR:
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Incorrect instruction packet for motor ID %d", dxl_id);
      break;
    case COMM_RX_WAITING:
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Receiving status packet for motor ID %d", dxl_id);
      break;
    case COMM_RX_TIMEOUT:
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "No status packet received for motor ID %d", dxl_id);
      break;
    case COMM_RX_CORRUPT:
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Incorrect status packet for motor ID %d", dxl_id);
      break;
    case COMM_NOT_AVAILABLE:
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Communication not available for motor ID %d", dxl_id);
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Unknown error for motor ID %d", dxl_id);
      break;
    }
  }

  // Placeholder for getting values from Dynamixel, returns the real value or -1 on failure
  int32_t getValuesFromDynamixel()
  {
    int32_t present_position = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
    dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        id_,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position),
        &dxl_error);

    // Check for communication success
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to get present position for motor with id %d.", id_);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "%s", packetHandler->getRxPacketError(dxl_error));
      }
      return INVALID_POSITION; // Return invalid position on error
    }

    // RCLCPP_INFO(
    //     rclcpp::get_logger(LOG_NAME),
    //     "Get [ID: %d] [Present Position: %d]",
    //     id_,
    //     present_position);

    return present_position;
  }

  // Placeholder for getting values from Dynamixel, returns the real value or -1 on failure
  int16_t getLoadFromDynamixel()
  {
    int16_t present_load = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
    dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        id_,
        ADDR_PRESENT_LOAD,
        reinterpret_cast<uint16_t *>(&present_load),
        &dxl_error);

    // Check for communication success
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to get present load for motor with id %d.", id_);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "%s", packetHandler->getRxPacketError(dxl_error));
      }
      return INVALID_LOAD; // Return invalid position on error
    }

    // RCLCPP_INFO(
    //     rclcpp::get_logger(LOG_NAME),
    //     "Get [ID: %d] [Present Position: %d]",
    //     id_,
    //     present_position);

    return present_load;
  }

public:
  static const uint32_t BAUDRATE = 1000000;
  static constexpr float PROTOCOL_VERSION = 2.0;

  static const uint16_t ADDR_OPERATING_MODE = 11;
  static const uint16_t ADDR_TORQUE_ENABLE = 64;
  static const uint16_t ADDR_GOAL_POSITION = 116;
  static const uint16_t ADDR_PRESENT_POSITION = 132;
  static const uint16_t ADDR_PROFILE_VELOCITY = 112;
  static const uint16_t ADDR_GOAL_VELOCITY = 104;
  static const uint8_t EXTENDED_POSITION_CONTROL_MODE = 4;
  static const uint16_t ADDR_PRESENT_LOAD = 126;
  static const uint16_t ADDR_PROFILE_ACCELERATION = 108;
  static const uint16_t ADDR_HOMING_OFFSET = 20;
};

// Define the static member
std::map<uint8_t, std::unique_ptr<DynamixelMotor>> DynamixelMotor::motors_;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("dynamixel_interface_node")
  {

    // RCLCPP_INFO(this->get_logger(), "I am here at line 367");
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/motorCnt", 10, std::bind(&MyNode::listener_callback, this, std::placeholders::_1));

    position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/dynaMotor_position", 10);

    ID_list_ = DynamixelMotor::getMotorIDs();

    // RCLCPP_INFO(this->get_logger(), "Motor ID %d is in ID_list", ID_list_[0]);

    for (uint8_t id : ID_list_)
    {
      RCLCPP_INFO(this->get_logger(), "**********Motor ID: %d", static_cast<int>(id));
    }

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MyNode::publish_motor_positions, this));
  }

private:
  void listener_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: [%f, %f]", msg->data[0], msg->data[1]);

    auto cmdsToMotors = construct_matrix_from_data(msg->data, msg->layout);

    if (cmdsToMotors.size() > 0)
    {
      RCLCPP_INFO(this->get_logger(), "Constructed Matrix");
      process_matrix(cmdsToMotors);
    }
  }

  std::vector<std::vector<double>> construct_matrix_from_data(
      const std::vector<double> &data,
      const std_msgs::msg::MultiArrayLayout &layout)
  {

    std::vector<std::string> labels;
    for (const auto &dim : layout.dim)
    {
      labels.push_back(dim.label);
    }

    std::vector<std::vector<std::string>> valid_combinations = {
        {"ID", "Position"},
        {"ID", "Position", "Velocity"},
        {"ID", "Velocity"}};

    if (std::find(valid_combinations.begin(), valid_combinations.end(), labels) == valid_combinations.end())
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid label combination");
      return {};
    }

    int num_rows = layout.dim[0].size;
    int num_cols = labels.size();

    std::vector<std::vector<double>> matrix(num_rows, std::vector<double>(num_cols, 0.0));
    // Populate the matrix based on the number of columns
    for (int i = 0; i < num_rows; ++i)
    {
      for (int j = 0; j < num_cols; ++j)
      {
        int index = i * num_cols + j;
        matrix[i][j] = data[index];
      }
    }

    // Filter rows based on ID list
    auto it = std::remove_if(matrix.begin(), matrix.end(), [this](const std::vector<double> &row)
                             { return std::find(ID_list_.begin(), ID_list_.end(), static_cast<int>(row[0])) == ID_list_.end(); });
    matrix.erase(it, matrix.end());

    if (matrix.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No valid ID found in the data.");
      return {};
    }

    // Dynamic label index determination
    int position_index = -1;
    int velocity_index = -1;

    for (size_t i = 0; i < labels.size(); ++i)
    {
      if (labels[i] == "Position")
      {
        position_index = i;
      }
      else if (labels[i] == "Velocity")
      {
        velocity_index = i;
      }
    }

    // Apply type casting
    for (auto &row : matrix)
    {
      // int id_val = static_cast<int>(row[0]);

      if (position_index != -1)
      { // Position processing
        row[position_index] = static_cast<int32_t>(row[position_index]);
      }

      if (velocity_index != -1)
      {
        if (velocity_index == 2)
        { // Velocity is in the third place
          row[velocity_index] = static_cast<uint32_t>(std::abs(row[velocity_index]));
        }
        else if (velocity_index == 1)
        { // Velocity is in the second place
          RCLCPP_WARN(this->get_logger(), "This part of the code has not been developed so far! change the next function as well!");
          return {};
          // row[velocity_index] = static_cast<int32_t>(row[velocity_index]);
        }
      }
    }

    return matrix;
  }

  void process_matrix(const std::vector<std::vector<double>> &matrix)
  {

    for (const auto &row : matrix)
    {
      int motor_id = static_cast<int>(row[0]);

      // Lock the mutex before accessing active_tasks_
      {
        std::lock_guard<std::mutex> lock(active_tasks_mutex_);
        if (active_tasks_.find(motor_id) != active_tasks_.end())
        {
          RCLCPP_INFO(this->get_logger(), "Motor ID %d is already processing. Skipping new command.", motor_id);
          continue;
        }

        // Mark the motor as active
        active_tasks_[motor_id] = true;
      }

      // Submit a new task (no need to hold the lock while creating the thread)
      std::thread(&MyNode::send_command_to_motor, this, row).detach();

      // Print a message indicating that a thread has been created for the motor ID
      RCLCPP_INFO(this->get_logger(), "Thread created for Motor ID %d.", motor_id);
    }

    //// No thread , and no locking

    // for (const auto &row : matrix)
    // {
    //   // int motor_id = static_cast<int>(row[0]);

    //   // Directly call the send_command_to_motor function for each motor
    //   send_command_to_motor(row);
    // }
  }

  void send_command_to_motor(const std::vector<double> &row)
  {
    int motor_id = static_cast<int>(row[0]);
    int32_t position = row[1];

    uint32_t velocity = row.size() > 2 ? row[2] : 0.0;

    RCLCPP_INFO(this->get_logger(), "these go to dynamixel, first float: [%f, %d]", row[1], position);
    // Check if setting profile velocity is successful
    if (row.size() > 2)
    {
      if (!DynamixelMotor::getMotor(motor_id).setProfileVelocity(velocity))
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to set profile velocity for motor ID %d", motor_id);
        return; // Exit early if there's an error
      }
    }

    // Check if setting position is successful
    if (!DynamixelMotor::getMotor(motor_id).setPosition(position))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to set position for motor ID %d", motor_id);
      return; // Exit early if there's an error
    }

    // Lock the mutex before modifying active_tasks_
    {
      std::lock_guard<std::mutex> lock(active_tasks_mutex_);
      active_tasks_.erase(motor_id); // Remove from active tasks after processing
    }

    RCLCPP_INFO(this->get_logger(), "Motor ID %d processing completed.", motor_id);
  }

  void publish_motor_positions()
  {
    std::vector<double> positions;
    for (int motor_id : ID_list_)
    {

      int32_t position = DynamixelMotor::getMotor(motor_id).getPosition();
      int16_t load = DynamixelMotor::getMotor(motor_id).getLoad();

      if (position != INVALID_POSITION)
      {
        positions.push_back(static_cast<double>(motor_id * 10000));
        positions.push_back(static_cast<double>(position));
        positions.push_back(static_cast<double>(load));

        // if (load != INVALID_LOAD && std::abs(load) > 400)
        // {
        //    DynamixelMotor::getMotor(motor_id).setPosition(position); // safety mechanism
        //   RCLCPP_ERROR(this->get_logger(), "Safety mechansim activated for motor ID %d", motor_id);

        // }
      }
    }

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = positions;
    // Set the layout
    msg.layout.dim.resize(3); // three dimensions: ID and Position and Load

    msg.layout.dim[0].label = "ID";
    msg.layout.dim[0].size = ID_list_.size();       // Number of motors
    msg.layout.dim[0].stride = 3 * ID_list_.size(); // Each ID has a corresponding Position

    msg.layout.dim[1].label = "Position";
    msg.layout.dim[1].size = ID_list_.size(); // Number of motors
    msg.layout.dim[1].stride = 2;             // Each Position corresponds to an ID

    msg.layout.dim[2].label = "Load";
    msg.layout.dim[2].size = ID_list_.size(); // Number of motors
    msg.layout.dim[2].stride = 1;

    position_publisher_->publish(msg);

    // RCLCPP_INFO(this->get_logger(), "Published motor positions.");
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<uint8_t> ID_list_;
  std::map<int, bool> active_tasks_;
  std::mutex active_tasks_mutex_;
};

int main(int argc, char *argv[])
{

  // ************** Setting up connection *****************//

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(Dynamixel_Motor_XC430::PROTOCOL_VERSION); //

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to open the port!");
    return -1;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(Dynamixel_Motor_XC430::BAUDRATE);
  if (dxl_comm_result == false)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to set the baudrate!");
    return -1;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Succeeded to set the baudrate.");
  }

  // ************** Defining and Initializign Motors *****************//

  // Create and add motor instances
  // (profile_velocity (rev/min), profile_acceleration, goal_velocity,
  //               min_position_limit (deg), max_position_limit, max_abs_Velocity, position_offset,  factor, position,  load_limit))

  // // Roll (MX-64(2.0)) = ID 0 , Location = Second Motor   ACHTUNG: CURRENTLY NOT WORKING  // Roll (jnt_wrist2X)
  // DynamixelMotor motor0(0);
  // motor0.initialize(60, 0, 120, -80, 80, 1000, 10, 3 / 0.229, 0);
  // DynamixelMotor::addMotor(0, motor0);

  // Pitch (XC430-W240(T240BB)) = ID 1 , Location = First Motor // Pitch (jnt_wrist1Y)
  // DynamixelMotor motor1(1);
  // motor1.initialize(3, 0, 5, -219, -22, 10, -120, 20 / 0.087891, 0);
  // DynamixelMotor::addMotor(1, motor1);

  auto motor51 = std::make_unique<DynamixelMotor>(51);
  motor51->initialize(10, 2, 5, -180, 180, 10, 0, 20 / 0.087891, 0, 300); // min (v0): -55    max (v90):-140   Delta:85, up: - ,  down: +
  DynamixelMotor::addMotor(51, std::move(motor51));

  auto motor52 = std::make_unique<DynamixelMotor>(52);
  motor52->initialize(1, 0.5, 1, -360, 360, 10, 0, 1 / 0.087891, 0, 300); // min: 225 (straight)   left: +  right : -
  DynamixelMotor::addMotor(52, std::move(motor52));

  // //// *********** testing *************///////

  // // // Yaw (XC430-W240(T240BB)) = ID 2 , Location = Third Motor  // Yaw (jnt_wrist3Z)
  // // DynamixelMotor motor2(2);
  // // motor2.initialize(3, 0, 5, -180, 180, 10, 0, 1 / 0.087891, 0);
  // // DynamixelMotor::addMotor(2, std::move(motor2));

  auto motor53 = std::make_unique<DynamixelMotor>(53);
  motor53->initialize(2, 1, 5, -360, 360, 10, 0, 1 / 0.087891, 200); // : 180 (home)  +_ 90  right:-  left: +
  DynamixelMotor::addMotor(53, std::move(motor53));

  // /////////******* *//////////////
  // // Gripper (XC430-W240(T240BB)) = ID 3 , Location = Fourth Motor  // Gripper (jaw)

  // // DynamixelMotor motor3(3);
  // // motor3.initialize(3, 0, 5, -620, 440, 10, 0, 1 / 0.087891, 0);
  // // DynamixelMotor::addMotor(3, motor3);

  auto motor54 = std::make_unique<DynamixelMotor>(54); // min (fopen) :-370, -720  max(fclose): 695, 279   delta: 1065  ~ 1000  ->  opening : -D Close: +D
  motor54->initialize(20, 5, 5, -1000, 1000, 25, 0, 1 / 0.087891, 150);
  DynamixelMotor::addMotor(54, std::move(motor54));

  // // Set and get position
  // motor1.setPosition(45);
  // int32_t pos = motor1.getPosition();
  // //  DynamixelMotor::getMotor(1).setPosition(45);

  // std::vector<uint8_t> motor_ids = DynamixelMotor::getMotorIDs();
  // for (uint8_t id : motor_ids)
  // {
  //   std::cout << "Motor ID: " << static_cast<int>(id) << std::endl;
  // }
  // DynamixelMotor::getMotor(2).setPosition(45);

  // std::vector<uint8_t>
  //     motor_ids = DynamixelMotor::getMotorIDs();

  // Get and print all motor IDs
  std::vector<uint8_t> motor_ids = DynamixelMotor::getMotorIDs();
  if (motor_ids.empty())
  {
    throw std::runtime_error(" There is no motor initiated!");
  }

  // if (motor_ids.empty())
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("dynamixel_interface"), "No motors found! Stopping execution.");
  //   // Add logic to stop further execution, for example:
  //   rclcpp::shutdown(); // Stops the ROS node
  //   return;             // Exit the current function
  // }

  // ************** Ros node *****************//

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}