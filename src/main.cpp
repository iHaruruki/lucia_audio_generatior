#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <mutex>

// YARP
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

class AudioGenerator : public rclcpp::Node
{
public:
    AudioGenerator()
    : Node("audio_generator_node")
    {
        // YARP ネットワークを一度だけ初期化
        static bool networkInitialized = false;
        if (!networkInitialized) {
            //yarp::os::Network::init();
            networkInitialized = true;
            RCLCPP_INFO(get_logger(), "YARP network initialized");
        }
        // 出力ポートを開いて接続
        port_.open("/test:o");
        if (!yarp::os::Network::connect("/test:o", "/soundGenerator/command:i")) {
            RCLCPP_ERROR(get_logger(), "Failed to connect to /soundGenerator/command:i");
        } else {
            RCLCPP_INFO(get_logger(), "Connected to /soundGenerator/command:i");
        }
    }

    ~AudioGenerator()
    {
        port_.close();
        RCLCPP_INFO(get_logger(), "AudioGenerator port closed");
    }

    void setText(const std::string& newText)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        text_ = newText;
        RCLCPP_DEBUG(get_logger(), "Text set to: '%s'", newText.c_str());
    }

    void audio_generator()
    {
        std::lock_guard<std::mutex> lk(mtx_);
        RCLCPP_INFO(get_logger(), "Processing audio: '%s'", text_.c_str());
        yarp::os::Bottle& b = port_.prepare();
        b.clear();
        b.addString("#");
        b.addString("generate");
        b.addString(text_);
        port_.write();
        RCLCPP_INFO(get_logger(), "Audio command sent");
    }

private:
    std::string text_;
    yarp::os::BufferedPort<yarp::os::Bottle> port_;
    std::mutex mtx_;
};

int main(int argc, char * argv[])
{
    //─── ROS2 初期化 ───────────────────────────────────
    rclcpp::init(argc, argv);

    //─── AudioGenerator ノード生成 ───────────────────────
    auto generator = std::make_shared<AudioGenerator>();

    //─── 再生したいメッセージ一覧 ────────────────────────
    const std::vector<std::string> messages = {
        "こんにちは",                // 挨拶
        "バイタルを測定しませんか", // 測定の呼びかけ
        "ありがとうございました．"  // 終了の挨拶
    };

    //─── 各メッセージを順に送信 ────────────────────────
    for (const auto & msg : messages) {
        RCLCPP_INFO(generator->get_logger(),
                    "AudioGenerator: '%s'", msg.c_str());
        generator->setText(msg);
        generator->audio_generator();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    //─── シャットダウン ───────────────────────────────
    RCLCPP_INFO(generator->get_logger(),
                "AudioGenerator ノードをシャットダウンします");
    rclcpp::shutdown();
    return 0;
}
