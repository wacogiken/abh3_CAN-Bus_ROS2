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
@file           ros2ABH3Brd.cpp
*******************************************************************************
@brief          ABH3用ROS2 ブロードキャストノード サンプルソフト
*******************************************************************************
@date           2021.06.24
@author         T.Furusawa
@note           ・初版
******************************************************************************/

/******************************************************************************
  インクルードファイル 
******************************************************************************/
#include "canABH3++.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <abh3_can_interface/msg/abh3_brd0.hpp>
#include <abh3_can_interface/msg/abh3_brd1.hpp>
#include <abh3_can_interface/msg/abh3_brd2.hpp>
#include <abh3_can_interface/msg/abh3_brd3.hpp>
#include <abh3_can_interface/msg/abh3_brd4.hpp>
#include <abh3_can_interface/msg/abh3_brd5.hpp>
#include <abh3_can_interface/msg/abh3_brd6.hpp>

/******************************************************************************
  クラス定義
******************************************************************************/
class BrdSubPub
{
  private:
    rclcpp::Node::SharedPtr node_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Brd0>::SharedPtr pub0_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Brd1>::SharedPtr pub1_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Brd2>::SharedPtr pub2_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Brd3>::SharedPtr pub3_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Brd4>::SharedPtr pub4_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Brd5>::SharedPtr pub5_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Brd6>::SharedPtr pub6_;

    canABH3 abh_;
    std::string device_;
    int abh3_id_;
    int host_id_;
    int priority_;
    int brd_group_;
    int timeout_;
    int mask_;

  public:
    void callback(const std_msgs::msg::Int32::SharedPtr msg);

    BrdSubPub()
    {
      node_ = rclcpp::Node::make_shared("abh3Brd");

      mask_ = node_->declare_parameter<int>("mask", 0x3f);
      if (mask_ & 0x01) {pub0_ = node_->create_publisher<abh3_can_interface::msg::Abh3Brd0>("brd0_data", rclcpp::QoS(10));}
      if (mask_ & 0x02) {pub1_ = node_->create_publisher<abh3_can_interface::msg::Abh3Brd1>("brd1_data", rclcpp::QoS(10));}
      if (mask_ & 0x04) {pub2_ = node_->create_publisher<abh3_can_interface::msg::Abh3Brd2>("brd2_data", rclcpp::QoS(10));}
      if (mask_ & 0x08) {pub3_ = node_->create_publisher<abh3_can_interface::msg::Abh3Brd3>("brd3_data", rclcpp::QoS(10));}
      if (mask_ & 0x10) {pub4_ = node_->create_publisher<abh3_can_interface::msg::Abh3Brd4>("brd4_data", rclcpp::QoS(10));}
      if (mask_ & 0x20) {pub5_ = node_->create_publisher<abh3_can_interface::msg::Abh3Brd5>("brd5_data", rclcpp::QoS(10));}
      if (mask_ & 0x40) {pub6_ = node_->create_publisher<abh3_can_interface::msg::Abh3Brd6>("brd6_data", rclcpp::QoS(10));}
      sub_ = node_->create_subscription<std_msgs::msg::Int32>("brd_num", rclcpp::QoS(10), std::bind(&BrdSubPub::callback, this, std::placeholders::_1));

      device_ = node_->declare_parameter<std::string>("device", "can0");
      abh3_id_ = node_->declare_parameter<int>("abh3_id", 1);
      host_id_ = node_->declare_parameter<int>("host_id", 2);
      priority_ = node_->declare_parameter<int>("priority", 0);
      brd_group_ = node_->declare_parameter<int>("brd_group", 5);
      timeout_ = node_->declare_parameter<int>("timeout", 1000);

      int result = abh_.port_init((char *)device_.c_str(), abh3_id_, host_id_, priority_, brd_group_, timeout_);
      if (result) {
        RCLCPP_ERROR(node_->get_logger(), "CAN-Bus port [%s] not open: %d", device_.c_str(), result);
        exit(0);
      }
    }

    ~BrdSubPub()
    {
      abh_.finish();
    }
    
    void Reset_CAN()
    {
      abh_.finish();
      int result = abh_.port_init((char *)device_.c_str(), abh3_id_, host_id_, priority_, brd_group_, timeout_);
      if (result) {
        RCLCPP_ERROR(node_->get_logger(), "CAN-Bus port [%s] not open: %d", device_.c_str(), result);
        exit(0);
      }
    }

    void Spin()
    {
      rclcpp::spin(node_);
    }
};

/******************************************************************************
  コールバック関数
******************************************************************************/
void BrdSubPub::callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (0 <= msg->data && msg->data <= 6 && mask_ & (1<<msg->data)) {
    int result = abh_.reqBRD(brd_group_*8+msg->data);
    if (result) {
      RCLCPP_ERROR(node_->get_logger(), "Broadcast Packet Error: %d", result);
      this->Reset_CAN();
    }
    else {
      switch(msg->data) {
        case 0:
        {
          abh3_can_interface::msg::Abh3Brd0 brd0;
          brd0.error = abh_.getBroad0Error();
          brd0.alarm = abh_.getBroad0Alarm();
          pub0_->publish(brd0);
        }
        break;
        case 1:
        {
          abh3_can_interface::msg::Abh3Brd1 brd1;
          brd1.control = abh_.getBroad1Control();
          brd1.in_out  = abh_.getBroad1In_Out();
          pub1_->publish(brd1);
        }
        break;
        case 2:
        {
          abh3_can_interface::msg::Abh3Brd2 brd2;
          brd2.vel_cmd_ay = cnvCAN2Vel(abh_.getBroad2VelCmdAY());
          brd2.vel_cmd_bx = cnvCAN2Vel(abh_.getBroad2VelCmdBX());
          brd2.vel_fbk_ay = cnvCAN2Vel(abh_.getBroad2VelFbkAY());
          brd2.vel_fbk_bx = cnvCAN2Vel(abh_.getBroad2VelFbkBX());
          pub2_->publish(brd2);
        }
        break;
        case 3:
        {
          abh3_can_interface::msg::Abh3Brd3 brd3;
          brd3.cur_cmd_ay = cnvCAN2Cur(abh_.getBroad3CurCmdAY());
          brd3.cur_cmd_bx = cnvCAN2Cur(abh_.getBroad3CurCmdBX());
          brd3.load_a = cnvCAN2Load(abh_.getBroad3LoadA());
          brd3.load_b = cnvCAN2Load(abh_.getBroad3LoadB());
          pub3_->publish(brd3);
        }
        break;
        case 4:
        {
          abh3_can_interface::msg::Abh3Brd4 brd4;
          brd4.pulse_a = abh_.getBroad4pulseA();
          brd4.pulse_b = abh_.getBroad4pulseB();
          pub4_->publish(brd4);
        }
        break;
        case 5:
        {
          abh3_can_interface::msg::Abh3Brd5 brd5;
          brd5.analog0 = cnvCAN2Analog(abh_.getBroad5Analog0());
          brd5.analog1 = cnvCAN2Analog(abh_.getBroad5Analog1());
          brd5.main_volt    = cnvCAN2Volt(abh_.getBroad5MainVolt());
          brd5.control_volt = cnvCAN2Volt(abh_.getBroad5ControlVolt());
          pub5_->publish(brd5);
        }
        break;
        case 6:
        {
          abh3_can_interface::msg::Abh3Brd6 brd6;
          brd6.monitor0 = abh_.getBroad6Monitor0();
          brd6.monitor1 = abh_.getBroad6Monitor1();
          pub6_->publish(brd6);
        }
        break;
        default:
        break;
      } 
    }
  }
}

/******************************************************************************
  メイン関数
******************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  BrdSubPub brd;

  brd.Spin();

  rclcpp::shutdown();
  return 0;
}