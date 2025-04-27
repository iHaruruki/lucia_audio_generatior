#include <yarp/os/all.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char* argv[]) {
    // 1) YARP ネットワーク初期化
    yarp::os::Network yarp;
    if (!yarp.checkNetwork(3.0)) {
        std::cerr << "Error: YARP network not available" << std::endl;
        return 1;
    }

    // 2) 入力ポートを開く（TTS 用クライアント）
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    port.open("/tts_client_cpp");

    // 3) soundGenerator の入力ポートに接続
    yarp::os::Network::connect("/tts_client_cpp", "/soundGenerator/command:i");

    std::cout << "Waiting for text on /tts_client_cpp ..." << std::endl;

    while (true) {
        // 4) 文字列を受信（ブロッキング）
        yarp::os::Bottle* in = port.read();
        if (!in || in->size() == 0) continue;
        std::string text = in->get(0).asString();
        if (text.empty()) continue;

        // 5) 一時 WAV ファイル名を作成
        char tmpname[] = "/tmp/ttsXXXXXX.wav";
        int fd = mkstemps(tmpname, 4);
        if (fd < 0) {
            perror("mkstemps");
            continue;
        }
        close(fd);
        std::string wav_path(tmpname);

        // 6) pico2wave でテキストを WAV 化
        std::string cmd = "pico2wave -w " + wav_path + " \"" + text + "\"";
        int ret = std::system(cmd.c_str());
        if (ret != 0) {
            std::cerr << "Error: pico2wave failed (code=" << ret << ")" << std::endl;
            std::remove(wav_path.c_str());
            continue;
        }

        // 7) soundGenerator にファイル名を送信して再生
        yarp::os::Bottle& out = port.prepare();
        out.clear();
        out.addString(wav_path);
        port.write();

        // 8) 再生後に一時ファイルを削除（少し待ってから）
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::remove(wav_path.c_str());
    }

    return 0;
}
