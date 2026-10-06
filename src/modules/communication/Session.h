#ifndef SESSION_H
#define SESSION_H

#include "FrameConsole.h"
#include "modules/utils/wifi/WifiPeer.h"

#include <string>

class WifiLink;

// A wifi console client: a frame console whose output and transfer input go through the link.
class Session : public FrameConsole, public WifiPeer {
public:
    Session() {}

    void bind(WifiLink* owner, uint8_t link, const uint8_t ip[4], uint16_t port);
    void release();

    bool fresh = false;                 // it sent something since the client list last came

    int puts(const char* s, int size = 0) override;
    void puts_source(TxSource* s) override;
    bool attach_sink(RxSink* s) override;
    void detach_sink() override;

    const std::string& cwd() const override { return path; }
    void set_cwd(const std::string& p) override { path = p; }

private:
    WifiLink* owner = nullptr;
    std::string path{"/"};
};

#endif
