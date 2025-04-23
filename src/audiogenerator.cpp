#include "../include/lucia_audio_generator/audiogenerator.hpp"

// C++ 標準ライブラリ
#include <utility>

// YARP
using namespace yarp::os;

// 静的メンバの定義
bool AudioGenerator::networkInitialized = false;

AudioGenerator::AudioGenerator()
{
    // ネットワーク初期化を一度だけ
    if (!networkInitialized) {
        Network::init();
        networkInitialized = true;
    }

    // 出力ポートを開いて接続
    port.open("/test:o");
    if (!Network::connect("/test:o", "/audioGenerator/gui:i")) {
        yError() << "[AudioGenerator] Failed to connect to /audioGenerator/gui:i";
    }
}

AudioGenerator::~AudioGenerator()
{
    port.close();
    // ※ Network::fini() はプログラム終了時に main() で呼ぶのがおすすめ
}

void AudioGenerator::setText(const std::string& newText)
{
    std::lock_guard<std::mutex> lk(mtx);
    text = newText;
}

void AudioGenerator::audio_generator()
{
    std::lock_guard<std::mutex> lk(mtx);

    yInfo() << "[AudioGenerator] processing...";
    Bottle& b = port.prepare();
    b.clear();
    b.addString("#");
    b.addString("generate");
    b.addString(text);
    port.write();
}
