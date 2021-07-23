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
@file           testCMD.cpp
*******************************************************************************
@brief          ABH3用ROS2 ジョイスティック命令ノード サンプルソフト
*******************************************************************************
@date           2021.07.13
@author         T.Furusawa
@note           ・初版
******************************************************************************/

/******************************************************************************
  インクルードファイル 
******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

/******************************************************************************
  クラス定義
******************************************************************************/
class testCMD
{
  private:
    rclcpp::Node::SharedPtr node_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    double gain_x_;
    double gain_a_;

  public:
    void callbackJOY(const std::shared_ptr<sensor_msgs::msg::Joy> joy);

    testCMD()
    {
      node_ = rclcpp::Node::make_shared("testCMD");
      sub_ = node_->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10), std::bind(&testCMD::callbackJOY, this, std::placeholders::_1));
      pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel1", rclcpp::QoS(10));

      gain_x_ = node_->declare_parameter<double>("gainX", 1.0);
      gain_a_ = node_->declare_parameter<double>("gainA", 1.0);
    }

    ~testCMD()
    {
    }

    void Spin()
    {
      rclcpp::spin(node_);
    }
};

/******************************************************************************
  コールバック関数 / サービス関数
******************************************************************************/
// AB -> YX & XA
void testCMD::callbackJOY(const std::shared_ptr<sensor_msgs::msg::Joy> msg)
{
  geometry_msgs::msg::Twist cmdXA;

  cmdXA.linear.x = msg->axes[4] * gain_x_;  // 右スティック前後　走行指令
  cmdXA.angular.z = msg->axes[0] * gain_a_; // 左スティック左右　旋回指令

  pub_->publish(cmdXA);
}

/******************************************************************************
  メイン関数
******************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  testCMD cmd;

  cmd.Spin();

  rclcpp::shutdown();
  return 0;
}