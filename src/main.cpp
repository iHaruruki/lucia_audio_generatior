// main.cpp
#include <rclcpp/rclcpp.hpp>            // ROS2 コア
#include <thread>                        // std::this_thread::sleep_for
#include <chrono>                        // std::chrono::seconds
#include <vector>                        // std::vector
#include <string>
#include <mutex>

// YARP
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

class AudioGenerator
{
public:
    AudioGenerator()
    {
        // YARP ネットワークを一度だけ初期化
        static bool networkInitialized = false;
        if (!networkInitialized) {
            yarp::os::Network::init();
            networkInitialized = true;
            RCLCPP_INFO(rclcpp::get_logger("AudioGenerator"),
                        "YARP network initialized");
        }
        // 出力ポートを開いて接続
        port_.open("/test:o");
        if (!yarp::os::Network::connect("/test:o", "/audioGenerator/gui:i")) {
            RCLCPP_ERROR(rclcpp::get_logger("AudioGenerator"),
                         "Failed to connect to /audioGenerator/gui:i");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("AudioGenerator"),
                        "Connected to /audioGenerator/gui:i");
        }
    }

    ~AudioGenerator()
    {
        port_.close();
        RCLCPP_INFO(rclcpp::get_logger("AudioGenerator"),
                    "AudioGenerator port closed");
    }

    void setText(const std::string& newText)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        text_ = newText;
        RCLCPP_DEBUG(rclcpp::get_logger("AudioGenerator"),
                     "Text set to: '%s'", newText.c_str());
    }

    void audio_generator()
    {
        std::lock_guard<std::mutex> lk(mtx_);
        RCLCPP_INFO(rclcpp::get_logger("AudioGenerator"),
                    "Processing audio: '%s'", text_.c_str());
        yarp::os::Bottle& b = port_.prepare();
        b.clear();
        b.addString("#");
        b.addString("generate");
        b.addString(text_);
        port_.write();
        RCLCPP_INFO(rclcpp::get_logger("AudioGenerator"),
                    "Audio command sent");
    }

private:
    std::string text_;
    yarp::os::BufferedPort<yarp::os::Bottle> port_;
    std::mutex mtx_;
};

int main(int argc, char * argv[])
{
    //─── ROS2 ノードの初期化 ─────────────────────────
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("audio_generator_node");

    //─── AudioGenerator インスタンス生成 ─────────────────
    AudioGenerator generator;

    //─── 再生したいメッセージ一覧 ────────────────────────
    const std::vector<std::string> messages = {
        "こんにちは",                // 挨拶
        "バイタルを測定しませんか", // 測定の呼びかけ
        "ありがとうございました．"  // 終了の挨拶
    };

    //─── 各メッセージを順に送信 ────────────────────────
    for (const auto & msg : messages) {
        // 1) ROS2 ログ出力
        RCLCPP_INFO(node->get_logger(),
                    "AudioGenerator: 再生メッセージ -> '%s'", msg.c_str());
        // 2) 音声生成処理
        generator.setText(msg);
        generator.audio_generator();
        // 3) 次の発話まで待機
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    //─── シャットダウン ─────────────────────────────
    RCLCPP_INFO(node->get_logger(),
                "AudioGenerator ノードをシャットダウンします");
    rclcpp::shutdown();
    return 0;
}
