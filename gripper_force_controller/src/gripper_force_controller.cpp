#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace gripper_force_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GripperCommand = control_msgs::action::GripperCommand;

class GripperForceController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    // 只寫入一個關節的 position 指令
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {joint_name_ + "/position"}};
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    // 讀關節目前 position（可依需要加 velocity/effort）
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {joint_name_ + "/position"}};
  }

  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration& dt) override {
    // 100~500 Hz 被 controller_manager 叫用：這裡做力控制
    if (!active_) return controller_interface::return_type::OK;

    // 讀狀態
    const double q = state_interfaces_[0].get_value();

    // 簡化：APPROACH/BACKOFF/HOLD_PI 的有限狀態機
    step_fsm(q, dt.seconds(), last_force_g_);

    // 寫命令（關節角度或寬度換算後的角度）
    command_interfaces_[0].set_value(q_cmd_);

    // 回饋 action（非必須每回合，但可定期）
    publish_feedback(q, last_force_g_);

    return controller_interface::return_type::OK;
  }

  CallbackReturn on_init() override {
    // 使用 auto_declare_parameters_if_not_declared 來安全地宣告參數
    // 如果參數已在 YAML 中定義，則不會重複宣告；否則，會使用這裡的預設值。
    auto_declare<std::string>("joint", "robotiq_85_left_knuckle_joint");
    auto_declare<std::string>("force_topic", "/arduino/force");
    auto_declare<int>("force_index", 2);
    auto_declare<double>("approach_speed_rad_s", -0.2);
    auto_declare<double>("contact_threshold_g", 250.0);
    auto_declare<double>("backoff_dist_rad", 0.05);
    auto_declare<double>("force_target_g", 200.0);
    auto_declare<double>("kp", 0.001);
    auto_declare<double>("ki", 0.0005);
    auto_declare<double>("deadband_g", 50.0);
    auto_declare<double>("rad_per_g", 1e-4);
    auto_declare<double>("i_clamp_rad", 0.1);
    auto_declare<double>("max_dq_rad_per_s", 0.5);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    joint_name_ = get_node()->get_parameter("joint").as_string();
    force_topic_ = get_node()->get_parameter("force_topic").as_string();
    force_index_ = get_node()->get_parameter("force_index").as_int();
    approach_speed_rad_s_ = get_node()->get_parameter("approach_speed_rad_s").as_double();
    contact_threshold_g_ = get_node()->get_parameter("contact_threshold_g").as_double();
    backoff_dist_rad_ = get_node()->get_parameter("backoff_dist_rad").as_double();
    force_target_g_ = get_node()->get_parameter("force_target_g").as_double();
    kp_ = get_node()->get_parameter("kp").as_double();
    ki_ = get_node()->get_parameter("ki").as_double();
    deadband_g_ = get_node()->get_parameter("deadband_g").as_double();
    rad_per_g_ = get_node()->get_parameter("rad_per_g").as_double();
    i_clamp_rad_ = get_node()->get_parameter("i_clamp_rad").as_double();
    max_dq_rad_per_s_ = get_node()->get_parameter("max_dq_rad_per_s").as_double();

    // 訂閱力感測 topic（或改成從硬體介面取）
    force_sub_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
      force_topic_, 10, [this](const std_msgs::msg::Float32MultiArray& msg){
        if (msg.data.size() > static_cast<size_t>(force_index_)) {
          last_force_g_ = msg.data[force_index_];
        } else {
          RCLCPP_WARN_ONCE(get_node()->get_logger(), "Force message data size (%zu) is smaller than requested index (%d). Using 0.0.", msg.data.size(), force_index_);
          last_force_g_ = 0.0;
        }
      });

    // 建立 ActionServer
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<GripperCommand>(
      get_node()->get_node_base_interface(),
      get_node()->get_node_clock_interface(),
      get_node()->get_node_logging_interface(),
      get_node()->get_node_waitables_interface(),
      "~/gripper_cmd",
      std::bind(&GripperForceController::handle_goal, this, _1, _2),
      std::bind(&GripperForceController::handle_cancel, this, _1),
      std::bind(&GripperForceController::handle_accepted, this, _1));

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    active_ = true;
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    active_ = false;
    // 當控制器停用時，釋放所有資源和狀態
    if (goal_handle_) {
      goal_handle_->abort(std::make_shared<GripperCommand::Result>());
      goal_handle_.reset();
    }
    return CallbackReturn::SUCCESS;
  }

private:
  // --- Action handlers ---
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const GripperCommand::Goal> goal) {
    // goal->command.position 可用來放「起始寬度/角度」或忽略；max_effort 可當保護門檻
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>)
  { cancel_requested_ = true; return rclcpp_action::CancelResponse::ACCEPT; }
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>> gh) {
    goal_handle_ = gh;
    cancel_requested_ = false;
    // 當新目標被接受時，重置FSM
    reset_fsm();
  }

  void publish_feedback(double q, double f_g) {
    if (!goal_handle_) return;
    auto fb = std::make_shared<GripperCommand::Feedback>();
    fb->position = q;       // 這裡用“關節角”或“等效寬度”，自定一致即可
    fb->effort   = 0.0;     // 若能取馬達電流可映射
    goal_handle_->publish_feedback(fb);
  }

  // --- 你的力控制/FSM ---
  void reset_fsm() {
    state_ = FSM_STATE::APPROACH;
    I_ = 0.0;
    // 以目前關節角度作為初始指令，避免跳動
    q_cmd_ = state_interfaces_.empty() ? 0.0 : state_interfaces_[0].get_value();
  }

  void step_fsm(double q, double dt, double force_g) {
    if (dt <= 0) return;

    switch(state_) {
      case FSM_STATE::APPROACH: // 接近
        q_cmd_ = q + approach_speed_rad_s_ * dt;
        if (force_g > contact_threshold_g_) {
          state_ = FSM_STATE::BACKOFF;
          // 計算退讓目標位置
          backoff_target_q_ = q + backoff_dist_rad_; // 假設正值是張開
        }
        break;
      case FSM_STATE::BACKOFF: // 接觸後退讓
        // 往目標位置移動，使用與接近相反的方向
        q_cmd_ = q - approach_speed_rad_s_ * dt;
        if (q >= backoff_target_q_) { // 假設關節值變大是張開
          state_ = FSM_STATE::HOLD_PI;
        }
        break;
      case FSM_STATE::HOLD_PI: // PI力控
      default:
        {
          double e = force_target_g_ - force_g;
          I_ += ki_ * e * dt;
          // 積分抗飽和
          I_ = (I_ < -i_clamp_rad_) ? -i_clamp_rad_ : (I_ > i_clamp_rad_ ? i_clamp_rad_ : I_);
          double dq = -(kp_ * e + I_) * rad_per_g_; // 誤差→角度微調
          // 速率限制
          dq = (dq < -max_dq_rad_per_s_ * dt) ? -max_dq_rad_per_s_ * dt : (dq > max_dq_rad_per_s_ * dt ? max_dq_rad_per_s_ * dt : dq);
          q_cmd_ = q + dq;
          // 完成條件（例如穩定在 ±deadband_g_）
          if (goal_handle_ && std::abs(e) < deadband_g_) {
            control_msgs::action::GripperCommand::Result res;
            res.reached_goal = true; res.stalled = false; res.position = q_cmd_; res.effort = 0.0;
            goal_handle_->succeed(std::make_shared<decltype(res)>(res));
            goal_handle_.reset();
          }
          if (cancel_requested_ && goal_handle_) {
            goal_handle_->canceled(std::make_shared<GripperCommand::Result>());
            goal_handle_.reset();
          }
        }
        break;
    }
  }

  // FSM 狀態定義
  enum FSM_STATE {
    APPROACH = 0,
    BACKOFF = 1,
    HOLD_PI = 2
  };
private:
  // 介面/狀態
  bool active_{false};
  std::string joint_name_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_sub_;
  double last_force_g_{0.0};

  // Action
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>> goal_handle_;
  bool cancel_requested_{false};

  // 參數/控制
  std::string force_topic_;
  int force_index_;
  double kp_, ki_, force_target_g_;
  double deadband_g_, rad_per_g_, i_clamp_rad_, max_dq_rad_per_s_;

  // FSM 參數與狀態
  double approach_speed_rad_s_;
  double contact_threshold_g_;
  double backoff_dist_rad_;
  double backoff_target_q_;

  // 指令與狀態變數
  double q_cmd_{0.0};
  FSM_STATE state_{FSM_STATE::APPROACH};
  double I_{0.0}; // PI控制器積分項
};

} // namespace

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gripper_force_controller::GripperForceController,
                       controller_interface::ControllerInterface)
