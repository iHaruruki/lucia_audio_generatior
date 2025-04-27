#include <yarp/os/all.h>
#include <iostream>

using namespace yarp::os;

int main() {
    Network yarp;
    BufferedPort<Bottle> server;
    server.open("/info");

    Bottle query, response;
    query.addString("bot");
    query.addString("list");

    // Name Server へ問い合わせ
    yarp.write(yarp.getNameServerName().c_str(), query, response);

    // 返ってきたポート一覧を表示
    for (int i = 1; i < response.size(); ++i) {
        Bottle *port = response.get(i).asList();
        if (port && port->check("name")) {
            std::cout << "Port: "
                      << port->find("name").asString()
                      << std::endl;
        }
    }
    return 0;
}
