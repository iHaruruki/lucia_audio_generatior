// ヘッダでポートをメンバに
class AudioGenerator {
    public:
        AudioGenerator() {
            if (!networkInitialized) {
                Network::init();
                networkInitialized = true;
            }
            port.open("/test:o");
            if (!Network::connect("/test:o", "/audioGenerator/gui:i")) {
                yError() << "Failed to connect to /audioGenerator/gui:i";
            }
        }
        ~AudioGenerator() {
            port.close();
        }

        void setText(const std::string& newText) {
            std::lock_guard<std::mutex> lk(mtx);
            text = newText;
        }

        void audio_generator() {
            std::lock_guard<std::mutex> lk(mtx);
            yInfo() << "processing...";
            Bottle &b = port.prepare();
            b.clear();
            b.addString("#");
            b.addString("generate");
            b.addString(text);
            port.write();
        }

    private:
        static bool networkInitialized;
        std::string text;
        BufferedPort<Bottle> port;
        std::mutex mtx;
    };
    bool AudioGenerator::networkInitialized = false;
