/******************************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Waco Giken Co., Ltd.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Waco Giken nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*!****************************************************************************
@file           ros2ABH3Cmd.cpp
*******************************************************************************
@brief          ABH3用ROS2 指令ノード サンプルソフト
*******************************************************************************
@date           2021.07.01
@author         T.Furusawa
@note           ・初版
******************************************************************************/

/******************************************************************************
  インクルードファイル 
******************************************************************************/
#include "canABH3++.hpp"
#include <rclcpp/rclcpp.hpp>
#include <abh3_can_interface/msg/abh3_velocity.hpp>
#include <abh3_can_interface/msg/abh3_current.hpp>
#include <abh3_can_interface/msg/abh3_input.hpp>
#include <abh3_can_interface/srv/abh3_input.hpp>
#include <mutex>
#include <vector>
#include <string>
#include <sstream>

/******************************************************************************
  クラス定義
******************************************************************************/
class CmdSubPub
{
  private:
    rclcpp::Node::SharedPtr node_ = nullptr;

    rclcpp::Subscription<abh3_can_interface::msg::Abh3Velocity>::SharedPtr sub_vel_;
    rclcpp::Subscription<abh3_can_interface::msg::Abh3Current>::SharedPtr sub_cur_;
    rclcpp::Subscription<abh3_can_interface::msg::Abh3Input>::SharedPtr  sub_inp_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Velocity>::SharedPtr pub_yx_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Velocity>::SharedPtr pub_ab_;
    rclcpp::Service<abh3_can_interface::srv::Abh3Input>::SharedPtr srv_;

    canABH3 abh_;
    std::string device_;
    int abh3_id_;
    int host_id_;
    int priority_;
    int timeout_;

    std::mutex mutex_;

    const std::vector<std::string>   cmd_str  = {"SERVO",    "START",    "DIR",     "CADD",      "CDIR",     "PCLR",     "TORQUE",   "M/S",      "BRAKE",    "RESET"   };
    const std::vector<std::uint32_t> cmd_mask = {0x00001001, 0x00002002, 0x00004004, 0x00008008, 0x00800080, 0x00080100, 0x03000000, 0x04000000, 0x10000000, 0x80000000};
    const std::uint32_t cmd_mask_ay   =          0x9500018f;
    const std::uint32_t cmd_mask_bx   =          0x9688f000;

  public:
    void callbackVel(const std::shared_ptr<abh3_can_interface::msg::Abh3Velocity> msg);
    void callbackCur(const std::shared_ptr<abh3_can_interface::msg::Abh3Current> msg);
    void callbackInp(const std::shared_ptr<abh3_can_interface::msg::Abh3Input>  msg);
    void serviceInp(const std::shared_ptr<rmw_request_id_t> header,
                    const std::shared_ptr<abh3_can_interface::srv::Abh3Input::Request> req,
                    const std::shared_ptr<abh3_can_interface::srv::Abh3Input::Response> res
                    );

    CmdSubPub()
    {
      node_ = rclcpp::Node::make_shared("abh3Cmd");
      pub_yx_ = node_->create_publisher<abh3_can_interface::msg::Abh3Velocity>("vel_fbk_yx", rclcpp::QoS(10));
      pub_ab_ = node_->create_publisher<abh3_can_interface::msg::Abh3Velocity>("vel_fbk_ab", rclcpp::QoS(10));
      sub_vel_ = node_->create_subscription<abh3_can_interface::msg::Abh3Velocity>("vel_cmd", rclcpp::QoS(10), std::bind(&CmdSubPub::callbackVel, this, std::placeholders::_1));
      sub_cur_ = node_->create_subscription<abh3_can_interface::msg::Abh3Current>("cur_cmd", rclcpp::QoS(10), std::bind(&CmdSubPub::callbackCur, this, std::placeholders::_1));
      sub_inp_ = node_->create_subscription<abh3_can_interface::msg::Abh3Input>("inp_data", rclcpp::QoS(10), std::bind(&CmdSubPub::callbackInp, this, std::placeholders::_1));
      srv_ = node_->create_service<abh3_can_interface::srv::Abh3Input>("inp_service", std::bind(&CmdSubPub::serviceInp, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

      device_ = node_->declare_parameter<std::string>("device", "can0");
      abh3_id_ = node_->declare_parameter<int>("abh3_id", 1);
      host_id_ = node_->declare_parameter<int>("host_id", 2);
      priority_ = node_->declare_parameter<int>("priority", 0);
      timeout_ = node_->declare_parameter<int>("timeout", 1000);
      int result = abh_.port_init((char *)device_.c_str(), abh3_id_, host_id_, priority_, 0, timeout_);
      if (result) {
        RCLCPP_ERROR(node_->get_logger(), "CAN-Bus port [%s] not open: %d", device_.c_str(), result);
        exit(0);
      }
      result = abh_.cmd_init();
      if (result) {
        RCLCPP_ERROR(node_->get_logger(), "Command Init. CAN-Bus fail: %d", result);
        exit(0);
      }
    }

    ~CmdSubPub()
    {
      abh_.finish();
    }
    
    void Reset_CAN()
    {
      abh_.finish();
      int result = abh_.port_init((char *)device_.c_str(), abh3_id_, host_id_, priority_, 0, timeout_);
      if (result) {
        RCLCPP_ERROR(node_->get_logger(), "CAN-Bus port [%s] not open: %d", device_.c_str(), result);
        exit(0);
      }
      result = abh_.cmd_init();
      if (result) {
        RCLCPP_ERROR(node_->get_logger(), "Command Init. CAN-Bus fail: %d", result);
        exit(0);
      }
    }

    void Spin()
    {
      rclcpp::spin(node_);
    }
};

/******************************************************************************
  コールバック関数 / サービス関数
******************************************************************************/
void CmdSubPub::callbackVel(const std::shared_ptr<abh3_can_interface::msg::Abh3Velocity> msg)
{
  std::lock_guard<std::mutex>lock(mutex_);

  int result = abh_.cmd(cnvVel2CAN(msg->vel_ay), cnvVel2CAN(msg->vel_bx));
  if (result) {
    RCLCPP_ERROR(node_->get_logger(), "Velocity Command Packet Error: %d", result);
    this->Reset_CAN();
  }
  else {
    abh3_can_interface::msg::Abh3Velocity fbk;
    fbk.vel_ay = cnvCAN2Vel(abh_.getSingleDP0FbkY());
    fbk.vel_bx = cnvCAN2Vel(abh_.getSingleDP0FbkX());
    pub_yx_->publish(fbk);
    fbk.vel_ay = cnvCAN2Vel(abh_.getSingleDP0FbkA());
    fbk.vel_bx = cnvCAN2Vel(abh_.getSingleDP0FbkB());
    pub_ab_->publish(fbk);
  }
}

void CmdSubPub::callbackCur(const std::shared_ptr<abh3_can_interface::msg::Abh3Current> msg)
{
  std::lock_guard<std::mutex>lock(mutex_);

  int result = abh_.cmd(cnvCur2CAN(msg->cur_ay), cnvCur2CAN(msg->cur_bx));
  if (result) {
    RCLCPP_ERROR(node_->get_logger(), "Current Command Packet Error: %d", result);
    this->Reset_CAN();
  }
  else {
    abh3_can_interface::msg::Abh3Velocity fbk;
    fbk.vel_ay = cnvCAN2Vel(abh_.getSingleDP0FbkY());
    fbk.vel_bx = cnvCAN2Vel(abh_.getSingleDP0FbkX());
    pub_yx_->publish(fbk);
    fbk.vel_ay = cnvCAN2Vel(abh_.getSingleDP0FbkA());
    fbk.vel_bx = cnvCAN2Vel(abh_.getSingleDP0FbkB());
    pub_ab_->publish(fbk);
  }
}

void CmdSubPub::callbackInp(const std::shared_ptr<abh3_can_interface::msg::Abh3Input> msg)
{
  std::lock_guard<std::mutex>lock(mutex_);
  int result = abh_.inSet(msg->data, msg->mask);
  if (result) {
    RCLCPP_ERROR(node_->get_logger(), "Input Command Packet Error: %d", result);
    this->Reset_CAN();
  }
  else {
    abh3_can_interface::msg::Abh3Velocity fbk;
    fbk.vel_ay = cnvCAN2Vel(abh_.getSingleDP0FbkY());
    fbk.vel_bx = cnvCAN2Vel(abh_.getSingleDP0FbkX());
    pub_yx_->publish(fbk);
    fbk.vel_ay = cnvCAN2Vel(abh_.getSingleDP0FbkA());
    fbk.vel_bx = cnvCAN2Vel(abh_.getSingleDP0FbkB());
    pub_ab_->publish(fbk);
  }
}

std::vector<std::string> split(const std::string &str, char sep)
{
    std::vector<std::string> v;
    std::stringstream ss(str);
    std::string buffer;
    while( std::getline(ss, buffer, sep) ) {
        v.push_back(buffer);
    }
    return v;
}

void CmdSubPub::serviceInp(const std::shared_ptr<rmw_request_id_t> header,
                           const std::shared_ptr<abh3_can_interface::srv::Abh3Input::Request> req,
                           const std::shared_ptr<abh3_can_interface::srv::Abh3Input::Response> res)
{
  std::vector<std::string> cmd;
  int32_t data = 0x00000000, mask = 0x00000000;

  (void)(header); // headerは使用しない

  res->responce.clear();
  cmd = split(req->command.c_str(), ' ');

  if (cmd.size() == 3) {

    if (cmd[0] == "SELECT") {
      mask = 0x00070070;
      if ("0" <= cmd[1] && cmd[1] <= "7") { data += atoi(cmd[1].c_str()) <<  4;} else {goto Invalid;}
      if ("0" <= cmd[2] && cmd[2] <= "7") { data += atoi(cmd[2].c_str()) << 16;} else {goto Invalid;}
    }
    else {
      unsigned int i=0;
      for (; i<cmd_str.size(); i++){
        if (cmd[0] == cmd_str[i]){
          break;
        }
      }
      if (i < cmd_str.size()){
        mask = cmd_mask[i];
        if (cmd[1] == "ON" || cmd[1] == "1") { data |= cmd_mask[i] & cmd_mask_ay;} else if (cmd[1] == "OFF" || cmd[1] == "0") {} else {goto Invalid;} 
        if (cmd[2] == "ON" || cmd[2] == "1") { data |= cmd_mask[i] & cmd_mask_bx;} else if (cmd[2] == "OFF" || cmd[2] == "0") {} else {goto Invalid;} 
      }
      else {
        goto Invalid;
      }
    }

    RCLCPP_INFO(node_->get_logger(), "%6s: %8x %8x", cmd[0].c_str(), data, mask); 
    std::lock_guard<std::mutex>lock(mutex_);
    int result = abh_.inSet(data, mask);
    if (result) {
      RCLCPP_ERROR(node_->get_logger(), "Input Command Packet Error: %d", result);
      this->Reset_CAN();
      res->responce = "Packet Error: " + std::to_string(result);
    }
  }
  else {
Invalid:
    res->responce = "Invalid Command.";
  }
}
 
/******************************************************************************
  メイン関数
******************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  CmdSubPub cmd;

  cmd.Spin();

  rclcpp::shutdown();
  return 0;
}