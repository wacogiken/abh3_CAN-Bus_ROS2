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
@file           ros2ABH3Cvn.cpp
*******************************************************************************
@brief          ABH3用ROS2 変換ノード サンプルソフト
*******************************************************************************
@date           2021.07.05
@author         T.Furusawa
@note           ・初版
******************************************************************************/

/******************************************************************************
  インクルードファイル 
******************************************************************************/
#include "canABH3++.hpp"
#include <rclcpp/rclcpp.hpp>
#include <abh3_can_interface/msg/abh3_velocity.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <math.h>

/******************************************************************************
  クラス定義
******************************************************************************/
class CnvSubPub
{
  private:
    rclcpp::Node::SharedPtr node_ = nullptr;
    rclcpp::Subscription<abh3_can_interface::msg::Abh3Velocity>::SharedPtr subAB_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Velocity>::SharedPtr pubAB_YX_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubAB_XA_;
    rclcpp::Subscription<abh3_can_interface::msg::Abh3Velocity>::SharedPtr subYX_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Velocity>::SharedPtr pubYX_AB_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubYX_XA_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subXA_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Velocity>::SharedPtr pubXA_AB_;
    rclcpp::Publisher<abh3_can_interface::msg::Abh3Velocity>::SharedPtr pubXA_YX_;

    double rate_;
    double wheel_;
    double width_; 

  public:
    void callbackAB(const std::shared_ptr<abh3_can_interface::msg::Abh3Velocity> msg);
    void callbackYX(const std::shared_ptr<abh3_can_interface::msg::Abh3Velocity> msg);
    void callbackXA(const std::shared_ptr<geometry_msgs::msg::Twist> msg);

    CnvSubPub()
    {
      node_ = rclcpp::Node::make_shared("abh3Cnv");
      subAB_ = node_->create_subscription<abh3_can_interface::msg::Abh3Velocity>("ab", rclcpp::QoS(10), std::bind(&CnvSubPub::callbackAB, this, std::placeholders::_1));
      pubAB_YX_ = node_->create_publisher<abh3_can_interface::msg::Abh3Velocity>("ab_yx", rclcpp::QoS(10));
      pubAB_XA_ = node_->create_publisher<geometry_msgs::msg::Twist>("ab_xa", rclcpp::QoS(10));

      subYX_ = node_->create_subscription<abh3_can_interface::msg::Abh3Velocity>("yx", rclcpp::QoS(10), std::bind(&CnvSubPub::callbackYX, this, std::placeholders::_1));
      pubYX_AB_ = node_->create_publisher<abh3_can_interface::msg::Abh3Velocity>("yx_ab", rclcpp::QoS(10));
      pubYX_XA_ = node_->create_publisher<geometry_msgs::msg::Twist>("yx_xa", rclcpp::QoS(10));

      subXA_ = node_->create_subscription<geometry_msgs::msg::Twist>("xa", rclcpp::QoS(10), std::bind(&CnvSubPub::callbackXA, this, std::placeholders::_1));
      pubXA_AB_ = node_->create_publisher<abh3_can_interface::msg::Abh3Velocity>("xa_ab", rclcpp::QoS(10));
      pubXA_YX_ = node_->create_publisher<abh3_can_interface::msg::Abh3Velocity>("xa_yx", rclcpp::QoS(10));

      rate_ = node_->declare_parameter<double>("rateNum", 1.0) / node_->declare_parameter<double>("rateDen", 10.0);
      wheel_ = node_->declare_parameter<double>("wheel", 0.1);
      width_ = node_->declare_parameter<double>("width", 0.5);
    }

    ~CnvSubPub()
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
void CnvSubPub::callbackAB(const std::shared_ptr<abh3_can_interface::msg::Abh3Velocity> msg)
{
  abh3_can_interface::msg::Abh3Velocity velYX;
  geometry_msgs::msg::Twist velXA;

  velYX.vel_ay = (msg->vel_ay + msg->vel_bx) / 2.0;
  velYX.vel_bx =  msg->vel_ay - velYX.vel_ay;

  velXA.linear.x  =  velYX.vel_ay / 60.0 * rate_ * wheel_ * M_PI;
  velXA.angular.z = -velYX.vel_bx / 60.0 * rate_ * wheel_ * M_PI / (width_ / 2.0);

  pubAB_YX_->publish(velYX);
  pubAB_XA_->publish(velXA);
}

// YX -> AB & XA 
void CnvSubPub::callbackYX(const std::shared_ptr<abh3_can_interface::msg::Abh3Velocity> msg)
{
  abh3_can_interface::msg::Abh3Velocity velAB;
  geometry_msgs::msg::Twist velXA;

  velAB.vel_ay = msg->vel_ay + msg->vel_bx;
  velAB.vel_bx = msg->vel_ay - msg->vel_bx;

  velXA.linear.x = msg->vel_ay / 60.0 * rate_ * wheel_ * M_PI;
  velXA.angular.z = -msg->vel_bx / 60.0 * rate_ * wheel_ * M_PI / (width_ / 2.0);

  pubYX_AB_->publish(velAB);
  pubYX_XA_->publish(velXA);
}

// XA -> YX & AB
void CnvSubPub::callbackXA(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  abh3_can_interface::msg::Abh3Velocity velYX;
  abh3_can_interface::msg::Abh3Velocity velAB;

  velYX.vel_ay = msg->linear.x * 60.0 / (rate_ * wheel_ * M_PI);
  velYX.vel_bx = -msg->angular.z * 60.0 / (rate_ * wheel_ * M_PI) * (width_ / 2.0);

  velAB.vel_ay = velYX.vel_ay + velYX.vel_bx;
  velAB.vel_bx = velYX.vel_ay - velYX.vel_bx;

  pubXA_YX_->publish(velYX);
  pubXA_AB_->publish(velAB);
}

/******************************************************************************
  メイン関数
******************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  CnvSubPub cmd;

  cmd.Spin();

  rclcpp::shutdown();
  return 0;
}