// main.cpp
#include <rclcpp/rclcpp.hpp>          // ROS2 の rclcpp
#include <thread>                     // std::this_thread::sleep_for
#include <chrono>                     // std::chrono::seconds
#include "../include/lucia_audio_generator/audiogenerator.hpp"           // AudioGenerator クラス

int main(int argc, char * argv[])
{
  //─── ROS 2 ノードの初期化 ───────────────────────────────────
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("audio_generator_node");

  //─── AudioGenerator の準備 ─────────────────────────────────
  AudioGenerator generator;

  //─── 読み上げたいメッセージリスト ─────────────────────────────
  const std::vector<std::string> messages = {
    "こんにちは",                     // 挨拶
    "バイタルを測定しませんか",      // バイタル測定の呼びかけ
    "ありがとうございました．"        // 終了メッセージ
  };

  //─── 順番にログを出して音声合成を呼び出し ────────────────────
  for (const auto & msg : messages) {
    // 1) ROS2 ログ出力（コメントとして表示）
    RCLCPP_INFO(node->get_logger(), "AudioGenerator: '%s' を再生します", msg.c_str());

    // 2) テキストをセットして音声生成
    generator.setText(msg);
    generator.audio_generator();

    // 3) 次の発話まで少し待機
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  //─── ノードのシャットダウン ─────────────────────────────────
  RCLCPP_INFO(node->get_logger(), "AudioGenerator ノードを終了します");
  rclcpp::shutdown();
  return 0;
}
