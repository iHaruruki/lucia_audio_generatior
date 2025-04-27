// src/tts_bridge_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

using std::placeholders::_1;

class TtsBridge : public rclcpp::Node {
public:
  TtsBridge()
  : Node("tts_bridge")
  {
    // YARP ネットワーク初期化
    if (!yarp::os::Network::checkNetwork(2.0)) {
      RCLCPP_ERROR(get_logger(), "YARP network is not available");
      throw std::runtime_error("YARP unreachable");
    }

    // YARP ポートを open
    cmd_port_.open("/vison/sound:o");
    state_port_.open("/tts_bridge/state:i");

    // soundGenerator モジュールと接続
    if (!yarp::os::Network::connect(cmd_port_.getName(),
                                    "/soundGenerator/command:i")) {
      RCLCPP_ERROR(get_logger(), "cannot connect to /soundGenerator/command:i");
    }
    if (!yarp::os::Network::connect("/soundGenerator/state:o",
                                    state_port_.getName())) {
      RCLCPP_ERROR(get_logger(), "cannot connect from /soundGenerator/state:o");
    }

    // ROS2 トピック購読
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "tts_text", 10,
      std::bind(&TtsBridge::on_tts_request, this, _1));
    RCLCPP_INFO(get_logger(), "TTS bridge ready. Subscribe: 'tts_text'");
  }

private:
  void on_tts_request(const std_msgs::msg::String::SharedPtr msg) {
    auto &out = cmd_port_.prepare();
    out.clear();
    out.addString("#");            // 制御マーカー
    out.addString("generate");     // generate コマンド
    out.addString(msg->data);      // 受信文字列
    cmd_port_.write();
    RCLCPP_INFO(get_logger(), "Sent to soundGenerator: \"%s\"",
                msg->data.c_str());

    // Optional: 再生完了まで待機してログ出力
    yarp::os::Bottle *reply = state_port_.read();
    if (reply) {
      RCLCPP_INFO(get_logger(), "soundGenerator state: %s",
                  reply->toString().c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  yarp::os::BufferedPort<yarp::os::Bottle> cmd_port_;
  yarp::os::BufferedPort<yarp::os::Bottle> state_port_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TtsBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
