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
@file           testBRD.cpp
*******************************************************************************
@brief          ABH3用ROS2 ブロードキャストノード テストソフト
*******************************************************************************
@date           2021.07.08
@author         T.Furusawa
@note           ・初版
******************************************************************************/

/******************************************************************************
  インクルードファイル 
******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

/******************************************************************************
  メイン関数
******************************************************************************/
int main(int argc, char **argv)
{
  /*** プロセスの初期化 ***/
  rclcpp::init(argc, argv);

  /*** ノードの初期化 ***/
  // ノードを生成する
  // 引数はノードの名称
  auto node = rclcpp::Node::make_shared("testBRD");

  /*** パブリッシャーの初期化 ***/
  // ノードにパブリッシュするデータを登録する
  // <>の中はパブリッシュするデータの型
  // 引数はトピックの名称とバッファの容量
  auto pub = node->create_publisher<std_msgs::msg::Int32>("brd_num", 10);

  /*** ループ周期の設定 ***/
  // 周期を周波数[Hz]で設定、浮動小数点で設定できる
  // マスクはb0～b6が有効で、1のときパブリッシュする
  int freq = node->declare_parameter<int>("freq", 10);
  int mask = node->declare_parameter<int>("mask", 0x7f);
  rclcpp::WallRate loop(freq);
  RCLCPP_INFO(node->get_logger(), "Frequency: %d[Hz]  Mask: 0x%02x", freq, mask);

  /*** パブリッシュするメッセージの宣言と初期化 ***/
  std_msgs::msg::Int32 msg;
  msg.data = 0;

  /*** CTRL+Cで中断されるまで無限ループ ***/
  // CTRL+Cが押されるとOK()がfalseになる。
  while(rclcpp::ok()) {
    do {
      if (mask & (1<<msg.data)) {
        // マスクが1ならパブリッシュしてスリープ
        pub->publish(msg);
        msg.data++;
        loop.sleep();
        break;
      } 
      msg.data++;
    } while(msg.data < 7);

    if (msg.data > 6) {
      // 0に戻すとき、マスクが0ならスリープ
      msg.data = 0;
      if (mask == 0) {
        loop.sleep();
      }
    }
  }
  RCLCPP_INFO(node->get_logger(), "Publish End");

  /*** プロセスの終了 ***/
  rclcpp::shutdown();
  return 0;
}