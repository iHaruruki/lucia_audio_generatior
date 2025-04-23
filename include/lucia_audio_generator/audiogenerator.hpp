#ifndef AUDIOGENERATOR_H
#define AUDIOGENERATOR_H

#include <iostream>
#include <string>

class AudioGenerator {
public:
    std::string text;   // メイン cpp から受け取るための変数

    // `setText()` を追加（メイン cpp からテキストを変更できる）
    void setText(const std::string& newText);

    // メイン cpp から送られたテキストを使用して音声生成コマンドを送信
    void audio_generator();
};

#endif
