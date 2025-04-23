#ifndef AUDIOGENERATOR_H
#define AUDIOGENERATOR_H

// C++ 標準ライブラリ
#include <string>
#include <mutex>

// YARP
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>

class AudioGenerator
{
public:
    AudioGenerator();
    ~AudioGenerator();

    // 再生テキストを設定
    void setText(const std::string& newText);

    // 音声生成コマンドを送信
    void audio_generator();

private:
    static bool networkInitialized;
    std::string text;
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    std::mutex mtx;
};

#endif // AUDIOGENERATOR_H
