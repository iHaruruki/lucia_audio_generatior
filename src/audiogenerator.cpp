// include/lucia_audio_generator/audiogenerator.hpp
#ifndef AUDIOGENERATOR_HPP
#define AUDIOGENERATOR_HPP

// C++ 標準ライブラリ
#include <string>
#include <mutex>

// YARP
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

// ROS2
#include <rclcpp/rclcpp.hpp>

class AudioGenerator
{
public:
    AudioGenerator();
    ~AudioGenerator();

    // テキストを設定
    void setText(const std::string& newText);
    // 音声生成コマンドを送信
    void audio_generator();

private:
    static bool networkInitialized;
    // ROS2 ロガー
    static rclcpp::Logger logger_;

    std::string text;
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    std::mutex mtx;
};

#endif // AUDIOGENERATOR_HPP


// src/audiogenerator.cpp
#include "../include/lucia_audio_generator/audiogenerator.hpp"

// 静的メンバの定義
bool AudioGenerator::networkInitialized = false;
rclcpp::Logger AudioGenerator::logger_ = rclcpp::get_logger("AudioGenerator");

AudioGenerator::AudioGenerator()
{
    // ROS2 初期化チェック
    if (!rclcpp::initialized()) {
        RCLCPP_WARN(logger_, "rclcpp is not initialized. Call rclcpp::init() before AudioGenerator constructor.");
    }

    // YARP ネットワークを一度だけ初期化
    if (!networkInitialized) {
        yarp::os::Network::init();
        networkInitialized = true;
        RCLCPP_INFO(logger_, "YARP network initialized");
    }

    // 出力ポートを開いて接続
    port.open("/test:o");
    if (!yarp::os::Network::connect("/test:o", "/audioGenerator/gui:i")) {
        RCLCPP_ERROR(logger_, "Failed to connect to /audioGenerator/gui:i");
    } else {
        RCLCPP_INFO(logger_, "Connected to /audioGenerator/gui:i");
    }
}

AudioGenerator::~AudioGenerator()
{
    port.close();
    RCLCPP_INFO(logger_, "AudioGenerator port closed");
}

void AudioGenerator::setText(const std::string& newText)
{
    std::lock_guard<std::mutex> lk(mtx);
    text = newText;
    RCLCPP_DEBUG(logger_, "Text set to: '%s'", newText.c_str());
}

void AudioGenerator::audio_generator()
{
    std::lock_guard<std::mutex> lk(mtx);
    RCLCPP_INFO(logger_, "Processing audio: '%s'", text.c_str());

    auto &b = port.prepare();
    b.clear();
    b.addString("#");
    b.addString("generate");
    b.addString(text);
    port.write();
    RCLCPP_INFO(logger_, "Audio command sent");
}
