#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/send_script.hpp"
#include "tm_msgs/msg/feedback_state.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("demo_get_feedback")
    {
      subscription_ = this->create_subscription<tm_msgs::msg::FeedbackState>(
      "feedback_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    
    void topic_callback(const tm_msgs::msg::FeedbackState::SharedPtr msg) const
    {
      if(msg->joint_pos.size() == 6){
        RCLCPP_INFO_STREAM(this->get_logger(),"FeedbackState: joint pos = (" << 
                msg->joint_pos[0] << ", " << 
                msg->joint_pos[1] << ", " << 
                msg->joint_pos[2] << ", " <<
                msg->joint_pos[3] << ", " << 
                msg->joint_pos[4] << ", " << 
                msg->joint_pos[5] << ")"); 
      }
      if(msg->joint_vel.size() == 6){
        RCLCPP_INFO_STREAM(this->get_logger(),"FeedbackState: joint vel = (" << 
                msg->joint_vel[0] << ", " << 
                msg->joint_vel[1] << ", " << 
                msg->joint_vel[2] << ", " <<
                msg->joint_vel[3] << ", " << 
                msg->joint_vel[4] << ", " << 
                msg->joint_vel[5] << ")"); 
      }
      
    }
    rclcpp::Subscription<tm_msgs::msg::FeedbackState>::SharedPtr subscription_;
    
};

bool send_cmd(std::string cmd, std::shared_ptr<rclcpp::Node> node, rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client){
  auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
  request->id = "demo";
  request->script = cmd;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if(result.get()->ok){
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"OK");
    } else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"not OK");
    }
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_send_script");
  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client =
    node->create_client<tm_msgs::srv::SendScript>("send_script");
  
  std::string cmd = "PTP(\"JPP\",0,0,90,0,90,0,35,200,0,false)";
  
  send_cmd(cmd, node, client);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
