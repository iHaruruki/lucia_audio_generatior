// main.cpp
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#include "../include/lucia_audio_generator/audiogenerator.hpp"
#include <yarp/os/Network.h>

int main() {
    // 1) YARP ネットワークを初期化
    yarp::os::Network::init();

    AudioGenerator generator;

    // 再生したいメッセージをリスト化
    std::vector<std::string> messages = {
        "こんにちは",
        "バイタルを測定しませんか",
        "ありがとうございました．"
    };

    // 順番に音声生成を呼び出し
    for (const auto& msg : messages) {
        generator.setText(msg);
        generator.audio_generator();
        // 次の発話まで少し待機（音声合成に時間がかかる場合があります）
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    // 2) YARP ネットワークを終了
    yarp::os::Network::fini();
    return 0;
}
