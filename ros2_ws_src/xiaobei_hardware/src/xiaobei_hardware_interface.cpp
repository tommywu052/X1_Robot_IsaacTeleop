#include "xiaobei_hardware/xiaobei_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scservo_sdk/SCServo.h" 
#include <map>

namespace xiaobei_hardware
{

SMS_STS st; 

// =================================================================
//                      机器人关节校准表
// =================================================================
struct JointConfig {
    int id;             // 舵机真实ID
    double offset;      // 零点偏移 (摆正时舵机的脉冲数，默认2048)
    double direction;   // 方向: 1.0 (正向) 或 -1.0 (反向)
};

std::map<std::string, JointConfig> joint_map;

// 根据你的描述生成的预设配置
void init_joint_config() {
    // ⚠️ 注意：这里的配置是基于你的描述推测的。
    // 如果某个关节运动反了，请把对应的 1.0 改成 -1.0 (或者反过来)
    // 如果关节初始位置歪了，请修改 2048.0 这个数值

    // === 左臂 (Left Arm) IDs: 1-6 ===
    // 描述: ID1(前+), ID2(里+), ID3(外旋+), ID4(前+), ID5(外旋+), ID6(外+)
    joint_map["j_1"] = {1, 2048.0, 1.0};  // 左肩前后
    joint_map["j_2"] = {2, 2048.0, 1.0};  // 左肩左右
    joint_map["j_3"] = {3, 2048.0, 1.0};  // 左大臂旋转
    joint_map["j_4"] = {4, 2048.0, 1.0};  // 左肘
    joint_map["j_5"] = {5, 2048.0, 1.0};  // 左小臂旋转
    joint_map["j_6"] = {6, 2048.0, 1.0};  // 左手腕
    joint_map["j_7"] = {7, 2048.0, -1.0};  // 左夹爪 (dir=-1: servo7 角度限位[1000..2048],合須往下→1396,與右爪一致)

    // === 右臂 (Right Arm) IDs: 51-56 ===
    // 描述: ID51(后+), ID52(外+), ID53(里旋+), ID54(后+), ID55(里旋+), ID56(里+)
    // 注意：这里的方向描述和左臂很多是反的，所以我预设为 -1.0
    joint_map["j_51"] = {51, 2048.0, -1.0}; // 右肩前后 (预设反向)
    joint_map["j_52"] = {52, 2048.0, -1.0}; // 右肩左右 (预设反向)
    joint_map["j_53"] = {53, 2048.0, -1.0}; // 右大臂旋转
    joint_map["j_54"] = {54, 2048.0, -1.0}; // 右肘
    joint_map["j_55"] = {55, 2048.0, -1.0}; // 右小臂旋转
    joint_map["j_56"] = {56, 2048.0, -1.0}; // 右手腕
    joint_map["j_57"] = {57, 2048.0, -1.0}; // 右夹爪

    // === 头部 (Head) ===
    joint_map["j_101"] = {101, 2048.0, 1.0};
    joint_map["j_102"] = {102, 2048.0, 1.0};
}

hardware_interface::CallbackReturn XiaobeiHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 初始化校准表
  init_joint_config();

  serial_port_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  hw_states_position_.resize(info_.joints.size(), 0.0);
  hw_states_velocity_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_states_effort_.resize(info_.joints.size(), 0.0);

  RCLCPP_INFO(rclcpp::get_logger("XiaobeiHardware"), "Opening Port: %s at %d", serial_port_.c_str(), baud_rate_);
  
  if(!st.begin(baud_rate_, serial_port_.c_str())){
      RCLCPP_ERROR(rclcpp::get_logger("XiaobeiHardware"), "Failed to open serial port!");
      return hardware_interface::CallbackReturn::ERROR;
  }

  // Build sync-read id list (only joints present in joint_map) and allocate the
  // sync-read buffer. One broadcast then reads pos+speed (4 bytes) from ALL servos.
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    auto it = joint_map.find(info_.joints[i].name);
    if (it == joint_map.end()) continue;
    rd_ids_.push_back((unsigned char)it->second.id);
    rd_idx_.push_back((int)i);
  }
  if (!rd_ids_.empty()) {
    st.syncReadBegin((u8)rd_ids_.size(), 4, 20);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> XiaobeiHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> XiaobeiHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn XiaobeiHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("XiaobeiHardware"), "Activating: hold current pose; torque only on responding servos.");
  // Safe start: only servos that actually respond get their goal set to the
  // CURRENT position and torque enabled. Servos that do not respond are left
  // torque-OFF (free), so a comms problem can never cause an unexpected motion.
  int ok_count = 0;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    std::string jn = info_.joints[i].name;
    if (joint_map.find(jn) == joint_map.end()) continue;
    JointConfig cfg = joint_map[jn];
    int pos = st.ReadPos(cfg.id);
    if (pos != -1) {
      double radians = cfg.direction * (pos - cfg.offset) * (3.1415926 / 2048.0);
      hw_states_position_[i] = radians;
      hw_commands_[i] = radians;
      st.WritePosEx(cfg.id, pos, 0, 0);
      st.EnableTorque(cfg.id, 1);
      ok_count++;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("XiaobeiHardware"), "Servo id %d (%s) no response; torque left OFF.", cfg.id, jn.c_str());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("XiaobeiHardware"), "Activated: %d servos responding/holding.", ok_count);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XiaobeiHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("XiaobeiHardware"), "Deactivating...");
  for (auto const& [name, config] : joint_map) {
      st.EnableTorque(config.id, 0);
  }
  st.syncReadEnd();
  st.end();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type XiaobeiHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // One broadcast sync-read fetches position+speed (4 bytes starting at
  // PRESENT_POSITION_L) for ALL servos in a single bus transaction, instead of
  // a separate request/response round-trip per servo. This is the main win for
  // loop rate over a high-latency USB link (usbipd).
  if (rd_ids_.empty()) {
    return hardware_interface::return_type::OK;
  }
  st.syncReadPacketTx(rd_ids_.data(), (u8)rd_ids_.size(), SMS_STS_PRESENT_POSITION_L, 4);
  for (std::size_t k = 0; k < rd_ids_.size(); k++) {
    int i = rd_idx_[k];
    JointConfig cfg = joint_map[info_.joints[i].name];
    unsigned char sbuf[4];
    if (st.syncReadPacketRx(rd_ids_[k], sbuf) == 4) {
      int pos = (sbuf[1] << 8) | sbuf[0];
      if (pos & (1 << 15)) pos = -(pos & ~(1 << 15));
      int speed = (sbuf[3] << 8) | sbuf[2];
      if (speed & (1 << 15)) speed = -(speed & ~(1 << 15));
      hw_states_position_[i] = cfg.direction * (pos - cfg.offset) * (3.1415926 / 2048.0);
      hw_states_velocity_[i] = cfg.direction * speed * (3.1415926 / 2048.0);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XiaobeiHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<u8> sync_ids; std::vector<s16> sync_pos; std::vector<u16> sync_spd; std::vector<u8> sync_acc;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
      std::string joint_name = info_.joints[i].name;
      
      int id_temp = 0;
      double offset = 2048.0;
      double direction = 1.0;

      if (joint_map.find(joint_name) != joint_map.end()) {
          id_temp = joint_map[joint_name].id;
          offset = joint_map[joint_name].offset;
          direction = joint_map[joint_name].direction;
      } else {
          try {
              id_temp = std::stoi(joint_name);
          } catch (...) { continue; }
      }

      double target_radians = hw_commands_[i];
      
      // 核心公式: 目标脉冲 = (目标弧度 / 方向 / 系数) + 零点偏移
      // 简化为: 目标脉冲 = (目标弧度 * 方向 * 转换率) + 偏移
      int target_pos = (int)(target_radians * direction * (2048.0 / 3.1415926) + offset);
      
      if(target_pos < 0) target_pos = 0;
      if(target_pos > 4095) target_pos = 4095;

      sync_ids.push_back((u8)id_temp);
      sync_pos.push_back((s16)target_pos);
      sync_spd.push_back(0);
      sync_acc.push_back(0);
  }
  if (!sync_ids.empty()) {
      st.SyncWritePosEx(sync_ids.data(), (u8)sync_ids.size(), sync_pos.data(), sync_spd.data(), sync_acc.data());
  }
  return hardware_interface::return_type::OK;
}

}  // namespace xiaobei_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  xiaobei_hardware::XiaobeiHardwareInterface, hardware_interface::SystemInterface)