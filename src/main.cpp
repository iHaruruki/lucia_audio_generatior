#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <iostream>

int main(int argc, char *argv[]) {
    // コマンドライン引数チェック
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " \"Text to speak\"" << std::endl;
        return 1;
    }

    // 引数を一つの文字列にまとめる（スペース含む文を扱うため）
    std::string text;
    for (int i = 1; i < argc; ++i) {
        text += argv[i];
        if (i < argc - 1) text += " ";
    }

    // YARP ネットワーク初期化
    yarp::os::Network yarp;
    if (!yarp.checkNetwork(2.0)) {
        std::cerr << "Error: YARP network not available." << std::endl;
        return 1;
    }

    // コマンド送信用ポート
    yarp::os::BufferedPort<yarp::os::Bottle> cmdPort;
    cmdPort.open("/ttsClient/command:o");

    // サウンドジェネレータの状態受信用ポート
    yarp::os::BufferedPort<yarp::os::Bottle> statePort;
    statePort.open("/ttsClient/state:i");

    // サウンドジェネレータのポートと接続
    if (!yarp::os::Network::connect(cmdPort.getName(), "/soundGenerator/command:i")) {
        std::cerr << "Error: cannot connect to /soundGenerator/command:i" << std::endl;
        return 1;
    }
    if (!yarp::os::Network::connect("/soundGenerator/state:o", statePort.getName())) {
        std::cerr << "Error: cannot connect from /soundGenerator/state:o" << std::endl;
        return 1;
    }

    // メッセージ作成： ["#", "generate", "<text>"]
    yarp::os::Bottle& out = cmdPort.prepare();
    out.clear();
    out.addString("#");
    out.addString("generate");
    out.addString(text);
    cmdPort.write();
    std::cout << "[TTS Client] Sent generate command: \"" << text << "\"" << std::endl;

    // 再生完了 or 状態メッセージをブロック読み込み
    yarp::os::Bottle* in = statePort.read();
    if (in) {
        std::cout << "[TTS Client] Received state: " << in->toString() << std::endl;
    } else {
        std::cerr << "[TTS Client] No response received." << std::endl;
    }

    // クリーンアップ
    cmdPort.close();
    statePort.close();
    return 0;
}
